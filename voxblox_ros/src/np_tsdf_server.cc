#include "voxblox_ros/np_tsdf_server.h"

#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>

#include "voxblox_ros/conversions.h"
#include "voxblox_ros/ros_params.h"

namespace voxblox {

NpTsdfServer::NpTsdfServer(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : NpTsdfServer(
          nh, nh_private, getTsdfMapConfigFromRosParam(nh_private),
          getNpTsdfIntegratorConfigFromRosParam(nh_private),
          getMeshIntegratorConfigFromRosParam(nh_private)) {}

NpTsdfServer::NpTsdfServer(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    const TsdfMap::Config& config,
    const NpTsdfIntegratorBase::Config& integrator_config,  // NOLINT
    const MeshIntegratorConfig& mesh_config)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(true),
      world_frame_("world"),
      icp_corrected_frame_("icp_corrected"),
      pose_corrected_frame_("pose_corrected"),
      max_block_distance_from_body_(std::numeric_limits<FloatingPoint>::max()),
      slice_level_(0.5),
      use_freespace_pointcloud_(false),
      color_map_(new RainbowColorMap()),
      publish_pointclouds_on_update_(false),
      publish_slices_(false),
      publish_pointclouds_(false),
      publish_tsdf_map_(false),
      cache_mesh_(false),
      enable_icp_(false),
      accumulate_icp_corrections_(true),
      pointcloud_queue_size_(1),
      num_subscribers_tsdf_map_(0),
      transformer_(nh, nh_private) {
  getServerConfigFromRosParam(nh_private);

  // Advertise topics.
  surface_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
          "surface_pointcloud", 1, true);
  tsdf_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
          "tsdf_pointcloud", 1, true);
  occupancy_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>(
          "occupied_nodes", 1, true);
  tsdf_slice_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "tsdf_slice", 1, true);

  nh_private_.param(
      "pointcloud_queue_size", pointcloud_queue_size_, pointcloud_queue_size_);
  pointcloud_sub_ = nh_.subscribe(
      "pointcloud", pointcloud_queue_size_, &NpTsdfServer::insertPointcloud,
      this);

  mesh_pub_ = nh_private_.advertise<voxblox_msgs::Mesh>("mesh", 1, true);

  // Publishing/subscribing to a layer from another node (when using this as
  // a library, for example within a planner).
  tsdf_map_pub_ =
      nh_private_.advertise<voxblox_msgs::Layer>("tsdf_map_out", 1, false);
  tsdf_map_sub_ = nh_private_.subscribe(
      "tsdf_map_in", 1, &NpTsdfServer::tsdfMapCallback, this);
  robot_model_pub_ =
      nh_private_.advertise<visualization_msgs::Marker>("Robot_model", 100);
  nh_private_.param("publish_tsdf_map", publish_tsdf_map_, publish_tsdf_map_);

  if (use_freespace_pointcloud_) {
    // points that are not inside an object, but may also not be on a surface.
    // These will only be used to mark freespace beyond the truncation distance.
    freespace_pointcloud_sub_ = nh_.subscribe(
        "freespace_pointcloud", pointcloud_queue_size_,
        &NpTsdfServer::insertFreespacePointcloud, this);
  }

  if (enable_icp_) {
    icp_transform_pub_ = nh_private_.advertise<geometry_msgs::TransformStamped>(
        "icp_transform", 1, true);
    nh_private_.param(
        "icp_corrected_frame", icp_corrected_frame_, icp_corrected_frame_);
    nh_private_.param(
        "pose_corrected_frame", pose_corrected_frame_, pose_corrected_frame_);
  }

  // Initialize TSDF Map and integrator.
  tsdf_map_.reset(new TsdfMap(config));

  std::string method("merged");
  nh_private_.param("method", method, method);
  if (method.compare("simple") == 0) {
    tsdf_integrator_.reset(new SimpleNpTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else if (method.compare("merged") == 0) {
    tsdf_integrator_.reset(new MergedNpTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else if (method.compare("fast") == 0) {
    tsdf_integrator_.reset(new FastNpTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else {
    tsdf_integrator_.reset(new SimpleNpTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  }

  mesh_layer_.reset(new MeshLayer(tsdf_map_->block_size()));
  mesh_integrator_.reset(new MeshIntegrator<TsdfVoxel>(
      mesh_config, tsdf_map_->getTsdfLayerPtr(), mesh_layer_.get()));
  icp_.reset(new ICP(getICPConfigFromRosParam(nh_private)));

  // Advertise services.
  generate_mesh_srv_ = nh_private_.advertiseService(
      "generate_mesh", &NpTsdfServer::generateMeshCallback, this);
  clear_map_srv_ = nh_private_.advertiseService(
      "clear_map", &NpTsdfServer::clearMapCallback, this);
  save_map_srv_ = nh_private_.advertiseService(
      "save_map", &NpTsdfServer::saveMapCallback, this);
  load_map_srv_ = nh_private_.advertiseService(
      "load_map", &NpTsdfServer::loadMapCallback, this);
  publish_pointclouds_srv_ = nh_private_.advertiseService(
      "publish_pointclouds", &NpTsdfServer::publishPointcloudsCallback, this);
  publish_tsdf_map_srv_ = nh_private_.advertiseService(
      "publish_map", &NpTsdfServer::publishTsdfMapCallback, this);

  // If set, use a timer to progressively integrate the mesh.
  double update_mesh_every_n_sec = 1.0f;
  nh_private_.param(
      "update_mesh_every_n_sec", update_mesh_every_n_sec,
      update_mesh_every_n_sec);

  if (update_mesh_every_n_sec > 0.0) {
    update_mesh_timer_ = nh_private_.createTimer(
        ros::Duration(update_mesh_every_n_sec), &NpTsdfServer::updateMeshEvent,
        this);
  } else {
    update_mesh_every_n_ = static_cast<int>(-1.0 * update_mesh_every_n_sec);
  }

  double publish_map_every_n_sec = 1.0;
  nh_private_.param(
      "publish_map_every_n_sec", publish_map_every_n_sec,
      publish_map_every_n_sec);

  if (publish_map_every_n_sec > 0.0) {
    publish_map_timer_ = nh_private_.createTimer(
        ros::Duration(publish_map_every_n_sec), &NpTsdfServer::publishMapEvent,
        this);
  }
}

void NpTsdfServer::getServerConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  // Before subscribing, determine minimum time between messages.
  // 0 by default.
  double min_time_between_msgs_sec = 0.0;
  nh_private.param(
      "min_time_between_msgs_sec", min_time_between_msgs_sec,
      min_time_between_msgs_sec);
  min_time_between_msgs_.fromSec(min_time_between_msgs_sec);

  nh_private.param(
      "max_block_distance_from_body", max_block_distance_from_body_,
      max_block_distance_from_body_);
  nh_private.param("slice_level", slice_level_, slice_level_);
  nh_private.param("world_frame", world_frame_, world_frame_);
  nh_private.param("sensor_frame", sensor_frame_, sensor_frame_);
  nh_private.param(
      "publish_pointclouds_on_update", publish_pointclouds_on_update_,
      publish_pointclouds_on_update_);
  nh_private.param("publish_slices", publish_slices_, publish_slices_);
  nh_private.param(
      "publish_pointclouds", publish_pointclouds_, publish_pointclouds_);
  nh_private.param(
      "use_freespace_pointcloud", use_freespace_pointcloud_,
      use_freespace_pointcloud_);
  nh_private.param(
      "pointcloud_queue_size", pointcloud_queue_size_, pointcloud_queue_size_);
  nh_private.param("enable_icp", enable_icp_, enable_icp_);
  nh_private.param(
      "accumulate_icp_corrections", accumulate_icp_corrections_,
      accumulate_icp_corrections_);

  // Logging
  nh_private.param("verbose", verbose_, verbose_);
  nh_private.param("timing", timing_, timing_);

  // Sensor specific
  nh_private_.param("sensor_is_lidar", sensor_is_lidar_, sensor_is_lidar_);
  nh_private_.param("width", width_, width_);
  nh_private_.param("height", height_, height_);
  nh_private_.param(
      "smooth_thre_ratio", smooth_thre_ratio_, smooth_thre_ratio_);

  if (sensor_is_lidar_) {
    nh_private_.param("fov_up", fov_up_, fov_up_);
    nh_private_.param("fov_down", fov_down_, fov_down_);
    float fov = std::abs(fov_down_) + std::abs(fov_up_);
    fov_down_rad_ = fov_down_ / 180.0f * M_PI;
    fov_rad_ = fov / 180.0f * M_PI;
  } else {
    nh_private_.param("vx", vx_, vx_);
    nh_private_.param("vy", vy_, vy_);
    nh_private_.param("fx", fx_, fx_);
    nh_private_.param("fy", fy_, fy_);
  }

  // Robot model related
  nh_private_.param(
      "publish_robot_model", publish_robot_model_, publish_robot_model_);
  nh_private_.param("robot_model_file", robot_model_file_, robot_model_file_);
  nh_private_.param(
      "robot_model_scale", robot_model_scale_, robot_model_scale_);

  // Mesh settings.
  nh_private.param("mesh_filename", mesh_filename_, mesh_filename_);
  std::string color_mode("");
  nh_private.param("color_mode", color_mode, color_mode);
  color_mode_ = getColorModeFromString(color_mode);

  // Color map for intensity pointclouds.
  std::string intensity_colormap("rainbow");
  float intensity_max_value = kDefaultMaxIntensity;
  nh_private.param(
      "intensity_colormap", intensity_colormap, intensity_colormap);
  nh_private.param(
      "intensity_max_value", intensity_max_value, intensity_max_value);

  // Default set in constructor.
  if (intensity_colormap == "rainbow") {
    color_map_.reset(new RainbowColorMap());
  } else if (intensity_colormap == "inverse_rainbow") {
    color_map_.reset(new InverseRainbowColorMap());
  } else if (intensity_colormap == "grayscale") {
    color_map_.reset(new GrayscaleColorMap());
  } else if (intensity_colormap == "inverse_grayscale") {
    color_map_.reset(new InverseGrayscaleColorMap());
  } else if (intensity_colormap == "ironbow") {
    color_map_.reset(new IronbowColorMap());
  } else {
    ROS_ERROR_STREAM("Invalid color map: " << intensity_colormap);
  }
  color_map_->setMaxValue(intensity_max_value);
}

void NpTsdfServer::processPointCloudMessageAndInsert(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg,
    const Transformation& T_G_C, const bool is_freespace_pointcloud) {
  timing::Timer ptcloud_timer("preprocess/input");
  // Convert the PCL pointcloud into our awesome format.
  // Horrible hack fix to fix color parsing colors in PCL.
  bool color_pointcloud = false;
  bool has_intensity = false;
  bool has_label = false;
  for (size_t d = 0; d < pointcloud_msg->fields.size(); ++d) {
    if (pointcloud_msg->fields[d].name == std::string("rgb")) {
      pointcloud_msg->fields[d].datatype = sensor_msgs::PointField::FLOAT32;
      color_pointcloud = true;
    } else if (pointcloud_msg->fields[d].name == std::string("intensity")) {
      has_intensity = true;
    } else if (pointcloud_msg->fields[d].name == std::string("label")) {
      has_label = true;
      // ROS_INFO("Found semantic/instance label in the point cloud");
    }
  }

  Pointcloud points_C;
  Pointcloud normals_C;
  Colors colors;
  Labels labels;

  // Convert differently depending on RGB or I type.
  if (has_label) {
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pointcloud_pcl(
        new pcl::PointCloud<pcl::PointXYZRGBL>());
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, *pointcloud_pcl);
    convertPointcloud(
        *pointcloud_pcl, color_map_, &points_C, &colors, &labels, true);
    pointcloud_pcl.reset(new pcl::PointCloud<pcl::PointXYZRGBL>());
  } else if (color_pointcloud) {
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  } else if (has_intensity) {
    pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  } else {
    pcl::PointCloud<pcl::PointXYZ> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  }
  ptcloud_timer.Stop();

  // calculate point-wise normal
  timing::Timer range_pre_timer("preprocess/normal_estimation");
  // Preprocess the point cloud: convert to range images
  cv::Mat vertex_map = cv::Mat::zeros(height_, width_, CV_32FC3);
  cv::Mat depth_image(vertex_map.size(), CV_32FC1, -1.0);
  cv::Mat color_image = cv::Mat::zeros(vertex_map.size(), CV_8UC3);
  projectPointCloudToImage(
      points_C, colors, vertex_map, depth_image, color_image);
  cv::Mat normal_image = computeNormalImage(vertex_map, depth_image);
  // Back project to point cloud from range images
  points_C = extractPointCloud(vertex_map, depth_image);
  normals_C = extractNormals(normal_image, depth_image);
  colors = extractColors(color_image, depth_image);
  range_pre_timer.Stop();

  // ICP based pose refinement
  Transformation T_G_C_refined = T_G_C;
  if (enable_icp_) {
    timing::Timer icp_timer("icp");
    if (!accumulate_icp_corrections_) {
      icp_corrected_transform_.setIdentity();
    }
    static Transformation T_offset;
    const size_t num_icp_updates = icp_->runICP(
        tsdf_map_->getTsdfLayer(), points_C, icp_corrected_transform_ * T_G_C,
        &T_G_C_refined);
    if (verbose_) {
      ROS_INFO(
          "ICP refinement performed %zu successful update steps",
          num_icp_updates);
    }
    icp_corrected_transform_ = T_G_C_refined * T_G_C.inverse();

    if (!icp_->refiningRollPitch()) {
      // its already removed internally but small floating point errors can
      // build up if accumulating transforms
      Transformation::Vector6 T_vec = icp_corrected_transform_.log();
      T_vec[3] = 0.0;
      T_vec[4] = 0.0;
      icp_corrected_transform_ = Transformation::exp(T_vec);
    }

    // Publish transforms as both TF and message.
    tf::Transform icp_tf_msg, pose_tf_msg;
    geometry_msgs::TransformStamped transform_msg;

    tf::transformKindrToTF(
        icp_corrected_transform_.cast<double>(), &icp_tf_msg);
    tf::transformKindrToTF(T_G_C.cast<double>(), &pose_tf_msg);
    tf::transformKindrToMsg(
        icp_corrected_transform_.cast<double>(), &transform_msg.transform);
    tf_broadcaster_.sendTransform(tf::StampedTransform(
        icp_tf_msg, pointcloud_msg->header.stamp, world_frame_,
        icp_corrected_frame_));
    tf_broadcaster_.sendTransform(tf::StampedTransform(
        pose_tf_msg, pointcloud_msg->header.stamp, icp_corrected_frame_,
        pose_corrected_frame_));

    transform_msg.header.frame_id = world_frame_;
    transform_msg.child_frame_id = icp_corrected_frame_;
    icp_transform_pub_.publish(transform_msg);

    icp_timer.Stop();
  }

  if (verbose_) {
    ROS_INFO("Integrating a pointcloud with %lu points.", points_C.size());
  }

  ros::WallTime start = ros::WallTime::now();
  // non-projective TSDF integration
  integratePointcloud(
      T_G_C_refined, points_C, normals_C, colors, is_freespace_pointcloud);
  ros::WallTime end = ros::WallTime::now();
  if (verbose_) {
    ROS_INFO(
        "Finished integrating in %f seconds, have %lu blocks.",
        (end - start).toSec(),
        tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks());
  }
  // mesh reconstruction with the counter interval
  if (update_mesh_every_n_ > 0 && frame_count_ != 0 &&
      frame_count_ % update_mesh_every_n_ == 0) {
    updateMesh();
  }

  // timing::Timer block_remove_timer("remove_distant_blocks");
  tsdf_map_->getTsdfLayerPtr()->removeDistantBlocks(
      T_G_C.getPosition(), max_block_distance_from_body_);
  mesh_layer_->clearDistantMesh(
      T_G_C.getPosition(), max_block_distance_from_body_);
  // block_remove_timer.Stop();

  publishRobotMesh(T_G_C_refined);

  // Callback for inheriting classes.
  newPoseCallback(T_G_C);
}

void NpTsdfServer::publishRobotMesh(const Transformation& T_G_C) {
  // publish the robot model with the pose
  visualization_msgs::Marker robot_model;
  robot_model.header.frame_id = world_frame_;
  robot_model.header.stamp = ros::Time();
  robot_model.mesh_resource = "file://" + robot_model_file_;
  robot_model.mesh_use_embedded_materials = true;
  robot_model.scale.x = robot_model.scale.y = robot_model.scale.z =
      robot_model_scale_;
  robot_model.lifetime = ros::Duration();
  robot_model.action = visualization_msgs::Marker::MODIFY;
  robot_model.color.a = robot_model.color.r = robot_model.color.g =
      robot_model.color.b = 1.;
  robot_model.type = visualization_msgs::Marker::MESH_RESOURCE;

  // Change to horizontal camera frame
  Transformation T_G_CH = T_G_C * transformer_.getModelTransform();
  Eigen::Quaternionf quatrot = T_G_CH.getEigenQuaternion();
  Point quat_vec = quatrot.vec();
  robot_model.pose.orientation.x = quat_vec(0);
  robot_model.pose.orientation.y = quat_vec(1);
  robot_model.pose.orientation.z = quat_vec(2);
  robot_model.pose.orientation.w = quatrot.w();
  Point translation = T_G_CH.getPosition();
  robot_model.pose.position.x = translation(0);
  robot_model.pose.position.y = translation(1);
  robot_model.pose.position.z = translation(2);
  robot_model_pub_.publish(robot_model);
}

// Checks if we can get the next message from queue.
bool NpTsdfServer::getNextPointcloudFromQueue(
    std::queue<sensor_msgs::PointCloud2::Ptr>* queue,
    sensor_msgs::PointCloud2::Ptr* pointcloud_msg, Transformation* T_G_C) {
  const size_t kMaxQueueSize = 10;
  if (queue->empty()) {
    return false;
  }
  *pointcloud_msg = queue->front();

  if (transformer_.lookupTransform(
          sensor_frame_, world_frame_, (*pointcloud_msg)->header.stamp,
          T_G_C)) {
    queue->pop();
    return true;
  } else {
    if (queue->size() >= kMaxQueueSize) {
      ROS_ERROR_THROTTLE(
          60,
          "Input pointcloud queue getting too long! Dropping "
          "some pointclouds. Either unable to look up transform "
          "timestamps or the processing is taking too long.");
      while (queue->size() >= kMaxQueueSize) {
        queue->pop();
      }
    }
  }
  return false;
}

void NpTsdfServer::insertPointcloud(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in) {
  if (pointcloud_msg_in->header.stamp - last_msg_time_ptcloud_ >
      min_time_between_msgs_) {
    last_msg_time_ptcloud_ = pointcloud_msg_in->header.stamp;
    // So we have to process the queue anyway... Push this back.
    pointcloud_queue_.push(pointcloud_msg_in);
  }

  Transformation T_G_C;
  sensor_msgs::PointCloud2::Ptr pointcloud_msg;
  bool processed_any = false;
  while (
      getNextPointcloudFromQueue(&pointcloud_queue_, &pointcloud_msg, &T_G_C)) {
    constexpr bool is_freespace_pointcloud = false;
    // main processing entrance
    processPointCloudMessageAndInsert(
        pointcloud_msg, T_G_C, is_freespace_pointcloud);
    processed_any = true;
  }

  if (!processed_any) {
    return;
  }

  if (publish_pointclouds_on_update_) {
    publishPointclouds();
  }

  if (timing_)
    ROS_INFO_STREAM(
        "Frame [" << frame_count_ << "] timings: " << std::endl
                  << timing::Timing::Print());
  if (verbose_)
    ROS_INFO_STREAM(
        "Layer memory: " << tsdf_map_->getTsdfLayer().getMemorySize());
  frame_count_++;
}

void NpTsdfServer::insertFreespacePointcloud(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in) {
  if (pointcloud_msg_in->header.stamp - last_msg_time_freespace_ptcloud_ >
      min_time_between_msgs_) {
    last_msg_time_freespace_ptcloud_ = pointcloud_msg_in->header.stamp;
    // So we have to process the queue anyway... Push this back.
    freespace_pointcloud_queue_.push(pointcloud_msg_in);
  }

  Transformation T_G_C;
  sensor_msgs::PointCloud2::Ptr pointcloud_msg;
  while (getNextPointcloudFromQueue(
      &freespace_pointcloud_queue_, &pointcloud_msg, &T_G_C)) {
    constexpr bool is_freespace_pointcloud = true;
    processPointCloudMessageAndInsert(
        pointcloud_msg, T_G_C, is_freespace_pointcloud);
  }
}

void NpTsdfServer::integratePointcloud(
    const Transformation& T_G_C, const Pointcloud& points_C,
    const Pointcloud& normals_C, const Colors& colors,
    const bool is_freespace_pointcloud) {
  CHECK_EQ(points_C.size(), colors.size());
  CHECK_EQ(points_C.size(), normals_C.size());
  tsdf_integrator_->integratePointCloud(
      T_G_C, points_C, normals_C, colors, is_freespace_pointcloud);
}

void NpTsdfServer::publishAllUpdatedTsdfVoxels() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromTsdfLayer(tsdf_map_->getTsdfLayer(), &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  tsdf_pointcloud_pub_.publish(pointcloud);
}

void NpTsdfServer::publishTsdfSurfacePoints() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  const float surface_distance_thresh =
      tsdf_map_->getTsdfLayer().voxel_size() * 0.75;
  createSurfacePointcloudFromTsdfLayer(
      tsdf_map_->getTsdfLayer(), surface_distance_thresh, &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  surface_pointcloud_pub_.publish(pointcloud);
}

void NpTsdfServer::publishTsdfOccupiedNodes() {
  // Create a pointcloud with distance = intensity.
  visualization_msgs::MarkerArray marker_array;
  createOccupancyBlocksFromTsdfLayer(
      tsdf_map_->getTsdfLayer(), world_frame_, &marker_array);
  occupancy_marker_pub_.publish(marker_array);
}

void NpTsdfServer::publishSlices() {
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromTsdfLayerSlice(
      tsdf_map_->getTsdfLayer(), 2, slice_level_, &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  tsdf_slice_pub_.publish(pointcloud);
}

void NpTsdfServer::publishMap(bool reset_remote_map) {
  if (!publish_tsdf_map_) {
    return;
  }
  int subscribers = this->tsdf_map_pub_.getNumSubscribers();
  if (subscribers > 0) {
    if (num_subscribers_tsdf_map_ < subscribers) {
      // Always reset the remote map and send all when a new subscriber
      // subscribes. A bit of overhead for other subscribers, but better than
      // inconsistent map states.
      reset_remote_map = true;
    }
    const bool only_updated = !reset_remote_map;
    timing::Timer publish_map_timer("map/publish_tsdf");
    voxblox_msgs::Layer layer_msg;
    serializeLayerAsMsg<TsdfVoxel>(
        this->tsdf_map_->getTsdfLayer(), only_updated, &layer_msg);
    if (reset_remote_map) {
      layer_msg.action = static_cast<uint8_t>(MapDerializationAction::kReset);
    }
    this->tsdf_map_pub_.publish(layer_msg);
    publish_map_timer.Stop();
  }
  num_subscribers_tsdf_map_ = subscribers;
}

void NpTsdfServer::publishPointclouds() {
  // Combined function to publish all possible pointcloud messages -- surface
  // pointclouds, updated points, and occupied points.
  publishAllUpdatedTsdfVoxels();
  publishTsdfSurfacePoints();
  publishTsdfOccupiedNodes();
  if (publish_slices_) {
    publishSlices();
  }
}

void NpTsdfServer::updateMesh() {
  if (verbose_) {
    ROS_INFO("Updating mesh.");
  }

  timing::Timer generate_mesh_timer("mesh/update");
  constexpr bool only_mesh_updated_blocks = true;
  constexpr bool clear_updated_flag = true;
  mesh_integrator_->generateMesh(only_mesh_updated_blocks, clear_updated_flag);
  generate_mesh_timer.Stop();

  timing::Timer publish_mesh_timer("mesh/publish");

  voxblox_msgs::Mesh mesh_msg;
  generateVoxbloxMeshMsg(mesh_layer_, color_mode_, &mesh_msg);
  mesh_msg.header.frame_id = world_frame_;
  mesh_pub_.publish(mesh_msg);

  if (cache_mesh_) {
    cached_mesh_msg_ = mesh_msg;
  }

  publish_mesh_timer.Stop();

  if (publish_pointclouds_ && !publish_pointclouds_on_update_) {
    publishPointclouds();
  }
}

bool NpTsdfServer::generateMesh() {
  timing::Timer generate_mesh_timer("mesh/generate");
  const bool clear_mesh = true;
  if (clear_mesh) {
    constexpr bool only_mesh_updated_blocks = false;
    constexpr bool clear_updated_flag = true;
    mesh_integrator_->generateMesh(
        only_mesh_updated_blocks, clear_updated_flag);
  } else {
    constexpr bool only_mesh_updated_blocks = true;
    constexpr bool clear_updated_flag = true;
    mesh_integrator_->generateMesh(
        only_mesh_updated_blocks, clear_updated_flag);
  }
  generate_mesh_timer.Stop();

  timing::Timer publish_mesh_timer("mesh/publish");
  voxblox_msgs::Mesh mesh_msg;
  generateVoxbloxMeshMsg(mesh_layer_, color_mode_, &mesh_msg);
  mesh_msg.header.frame_id = world_frame_;
  mesh_pub_.publish(mesh_msg);

  publish_mesh_timer.Stop();

  if (!mesh_filename_.empty()) {
    timing::Timer output_mesh_timer("mesh/output");
    const bool success = outputMeshLayerAsPly(mesh_filename_, *mesh_layer_);
    output_mesh_timer.Stop();
    if (success) {
      ROS_INFO("Output file as PLY: %s", mesh_filename_.c_str());
    } else {
      ROS_INFO("Failed to output mesh as PLY: %s", mesh_filename_.c_str());
    }
  }

  if (timing_)
    ROS_INFO_STREAM("Mesh Timings: " << std::endl << timing::Timing::Print());
  return true;
}

bool NpTsdfServer::saveMap(const std::string& file_path) {
  // Inheriting classes should add saving other layers to this function.
  return io::SaveLayer(tsdf_map_->getTsdfLayer(), file_path);
}

bool NpTsdfServer::loadMap(const std::string& file_path) {
  // Inheriting classes should add other layers to load, as this will only
  // load
  // the TSDF layer.
  constexpr bool kMulitpleLayerSupport = true;
  bool success = io::LoadBlocksFromFile(
      file_path, Layer<TsdfVoxel>::BlockMergingStrategy::kReplace,
      kMulitpleLayerSupport, tsdf_map_->getTsdfLayerPtr());
  if (success) {
    LOG(INFO) << "Successfully loaded TSDF layer.";
  }
  return success;
}

bool NpTsdfServer::clearMapCallback(
    std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response&
    /*response*/) {  // NOLINT
  clear();
  return true;
}

bool NpTsdfServer::generateMeshCallback(
    std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response&
    /*response*/) {  // NOLINT
  return generateMesh();
}

bool NpTsdfServer::saveMapCallback(
    voxblox_msgs::FilePath::Request& request, voxblox_msgs::FilePath::Response&
    /*response*/) {  // NOLINT
  return saveMap(request.file_path);
}

bool NpTsdfServer::loadMapCallback(
    voxblox_msgs::FilePath::Request& request, voxblox_msgs::FilePath::Response&
    /*response*/) {  // NOLINT
  bool success = loadMap(request.file_path);
  return success;
}

bool NpTsdfServer::publishPointcloudsCallback(
    std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response&
    /*response*/) {  // NOLINT
  publishPointclouds();
  return true;
}

bool NpTsdfServer::publishTsdfMapCallback(
    std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response&
    /*response*/) {  // NOLINT
  publishMap();
  return true;
}

void NpTsdfServer::updateMeshEvent(const ros::TimerEvent& /*event*/) {
  updateMesh();
}

void NpTsdfServer::publishMapEvent(const ros::TimerEvent& /*event*/) {
  publishMap();
}

void NpTsdfServer::clear() {
  tsdf_map_->getTsdfLayerPtr()->removeAllBlocks();
  mesh_layer_->clear();

  // Publish a message to reset the map to all subscribers.
  if (publish_tsdf_map_) {
    constexpr bool kResetRemoteMap = true;
    publishMap(kResetRemoteMap);
  }
}

void NpTsdfServer::tsdfMapCallback(const voxblox_msgs::Layer& layer_msg) {
  timing::Timer receive_map_timer("map/receive_tsdf");

  bool success =
      deserializeMsgToLayer<TsdfVoxel>(layer_msg, tsdf_map_->getTsdfLayerPtr());

  if (!success) {
    ROS_ERROR_THROTTLE(10, "Got an invalid TSDF map message!");
  } else {
    ROS_INFO_ONCE("Got an TSDF map from ROS topic!");
    if (publish_pointclouds_on_update_) {
      publishPointclouds();
    }
  }
}

bool NpTsdfServer::projectPointCloudToImage(
    const Pointcloud& points_C, const Colors& colors,
    cv::Mat& vertex_map,   // corresponding point // NOLINT
    cv::Mat& depth_image,  // Float depth image (CV_32FC1). // NOLINT
    cv::Mat& color_image) const {
  // TODO(py): consider to calculate in parallel to speed up
  for (size_t i = 0; i < points_C.size(); i++) {
    int u, v;
    float depth;
    if (sensor_is_lidar_)
      depth = projectPointToImageLiDAR(points_C[i], &u, &v);
    else
      depth = projectPointToImageCamera(points_C[i], &u, &v);
    if (depth > 0.0) {
      float old_depth = depth_image.at<float>(v, u);
      // save only nearest point for each pixel
      if (old_depth <= 0.0 || old_depth > depth) {
        for (int k = 0; k <= 2; k++) {
          vertex_map.at<cv::Vec3f>(v, u)[k] = points_C[i](k);
        }
        depth_image.at<float>(v, u) = depth;
        // BGR default order
        color_image.at<cv::Vec3b>(v, u)[0] = colors[i].b;
        color_image.at<cv::Vec3b>(v, u)[1] = colors[i].g;
        color_image.at<cv::Vec3b>(v, u)[2] = colors[i].r;
      }
    }
  }
  return false;
}

// point should be in the LiDAR's coordinate system
float NpTsdfServer::projectPointToImageLiDAR(
    const Point& p_C, int* u, int* v) const {
  // All values are ceiled and floored to guarantee that the resulting points
  // will be valid for any integer conversion.
  float depth =
      std::sqrt(p_C.x() * p_C.x() + p_C.y() * p_C.y() + p_C.z() * p_C.z());
  float yaw = std::atan2(p_C.y(), p_C.x());
  float pitch = std::asin(p_C.z() / depth);
  // projections in im coor (percentage)
  float proj_x = 0.5 * (yaw / M_PI + 1.0);
  float proj_y = 1.0 - (pitch - fov_down_rad_) / fov_rad_;
  // scale to image size
  proj_x *= width_;
  proj_y *= height_;
  // round for integer index
  CHECK_NOTNULL(u);
  *u = std::round(proj_x);
  if (*u == width_)
    *u = 0;

  CHECK_NOTNULL(v);
  *v = std::round(proj_y);
  if (std::ceil(proj_y) > height_ - 1 || std::floor(proj_y) < 0) {
    return (-1.0);
  }
  return depth;
}

bool NpTsdfServer::projectPointToImageCamera(
    const Point& p_C, int* u, int* v) const {
  CHECK_NOTNULL(u);
  *u = std::round(p_C.x() * fx_ / p_C.z() + vx_);
  if (*u >= width_ || *u < 0) {
    return false;
  }
  CHECK_NOTNULL(v);
  *v = std::round(p_C.y() * fy_ / p_C.z() + vy_);
  if (*v >= height_ || *v < 0) {
    return false;
  }
  return true;
}

cv::Mat NpTsdfServer::computeNormalImage(
    const cv::Mat& vertex_map, const cv::Mat& depth_image) const {
  cv::Mat normal_image(depth_image.size(), CV_32FC3, 0.0);
  for (int u = 0; u < width_; u++) {
    for (int v = 0; v < height_; v++) {
      Point p;
      p << vertex_map.at<cv::Vec3f>(v, u)[0], vertex_map.at<cv::Vec3f>(v, u)[1],
          vertex_map.at<cv::Vec3f>(v, u)[2];

      float d_p = depth_image.at<float>(v, u);
      // sign of the normal vector
      float sign = 1.0;

      if (d_p > 0) {
        // neighbor x (in ring)
        int n_x_u;
        if (u == width_ - 1)
          n_x_u = 0;
        else
          n_x_u = u + 1;
        Point n_x;
        n_x << vertex_map.at<cv::Vec3f>(v, n_x_u)[0],
            vertex_map.at<cv::Vec3f>(v, n_x_u)[1],
            vertex_map.at<cv::Vec3f>(v, n_x_u)[2];
        float d_n_x = depth_image.at<float>(v, n_x_u);
        if (d_n_x < 0)
          continue;
        // on the boundary, not continous
        if (std::abs(d_n_x - d_p) > smooth_thre_ratio_ * d_p)
          continue;

        // neighbor y
        int n_y_v;
        if (v == height_) {
          n_y_v = v - 1;
          sign *= -1.0;
        } else {
          n_y_v = v + 1;
        }
        Point n_y;
        n_y << vertex_map.at<cv::Vec3f>(n_y_v, u)[0],
            vertex_map.at<cv::Vec3f>(n_y_v, u)[1],
            vertex_map.at<cv::Vec3f>(n_y_v, u)[2];

        float d_n_y = depth_image.at<float>(n_y_v, u);
        if (d_n_y < 0)
          continue;
        // on the boundary, not continous
        if (std::abs(d_n_y - d_p) > smooth_thre_ratio_ * d_p)
          continue;
        Point dx = n_x - p;
        Point dy = n_y - p;

        Point normal = (dx.cross(dy)).normalized() * sign;
        cv::Vec3f& normals = normal_image.at<cv::Vec3f>(v, u);
        for (int k = 0; k <= 2; k++)
          normals[k] = normal(k);
      }
    }
  }
  return normal_image;
}

Pointcloud NpTsdfServer::extractPointCloud(
    const cv::Mat& vertex_map, const cv::Mat& depth_image) const {
  Pointcloud points_C;
  for (int v = 0; v < vertex_map.rows; v++) {
    for (int u = 0; u < vertex_map.cols; u++) {
      cv::Vec3f vertex = vertex_map.at<cv::Vec3f>(v, u);
      if (depth_image.at<float>(v, u) > 0) {
        Point p_C(vertex[0], vertex[1], vertex[2]);
        points_C.push_back(p_C);
      }
    }
  }
  return points_C;
}

Colors NpTsdfServer::extractColors(
    const cv::Mat& color_image, const cv::Mat& depth_image) const {
  Colors colors;
  for (int v = 0; v < color_image.rows; v++) {
    for (int u = 0; u < color_image.cols; u++) {
      // BGR
      cv::Vec3b color = color_image.at<cv::Vec3b>(v, u);
      if (depth_image.at<float>(v, u) > 0) {
        // RGB
        Color c_C(color[2], color[1], color[0]);
        colors.push_back(c_C);
      }
    }
  }
  return colors;
}

Pointcloud NpTsdfServer::extractNormals(
    const cv::Mat& normal_image, const cv::Mat& depth_image) const {
  Pointcloud normals_C;
  for (int v = 0; v < normal_image.rows; v++) {
    for (int u = 0; u < normal_image.cols; u++) {
      cv::Vec3f vertex = normal_image.at<cv::Vec3f>(v, u);
      if (depth_image.at<float>(v, u) > 0) {
        Ray n_C(vertex[0], vertex[1], vertex[2]);
        normals_C.push_back(n_C);
      }
    }
  }
  return normals_C;
}

}  // namespace voxblox

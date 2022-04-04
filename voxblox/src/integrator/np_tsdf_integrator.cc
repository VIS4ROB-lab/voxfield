#include "voxblox/integrator/np_tsdf_integrator.h"

#include <iostream>
#include <list>

namespace voxblox {

NpTsdfIntegratorBase::Ptr NpTsdfIntegratorFactory::create(
    const std::string& integrator_type_name,
    const NpTsdfIntegratorBase::Config& config, Layer<TsdfVoxel>* layer) {
  CHECK(!integrator_type_name.empty());

  int integrator_type = 1;
  for (const std::string& valid_integrator_type_name :
       kTsdfIntegratorTypeNames) {
    if (integrator_type_name == valid_integrator_type_name) {
      return create(static_cast<TsdfIntegratorType>(integrator_type), config,
                    layer);
    }
    ++integrator_type;
  }
  LOG(FATAL) << "Unknown TSDF integrator type: " << integrator_type_name;
  return NpTsdfIntegratorBase::Ptr();
}

NpTsdfIntegratorBase::Ptr NpTsdfIntegratorFactory::create(
    const TsdfIntegratorType integrator_type,
    const NpTsdfIntegratorBase::Config& config, Layer<TsdfVoxel>* layer) {
  CHECK_NOTNULL(layer);
  switch (integrator_type) {
    case TsdfIntegratorType::kSimple:
      return NpTsdfIntegratorBase::Ptr(
          new SimpleNpTsdfIntegrator(config, layer)); // NOLINT
      break;
    case TsdfIntegratorType::kMerged:
      return NpTsdfIntegratorBase::Ptr(
          new MergedNpTsdfIntegrator(config, layer)); // NOLINT
      break;
    case TsdfIntegratorType::kFast:
      return NpTsdfIntegratorBase::Ptr(new FastNpTsdfIntegrator(config, layer));
      break;
    default:
      LOG(FATAL) << "Unknown TSDF integrator type: "
                 << static_cast<int>(integrator_type);
      break;
  }
  return NpTsdfIntegratorBase::Ptr();
}

// Note many functions state if they are thread safe. Unless explicitly stated
// otherwise, this thread safety is based on the assumption that any pointers
// passed to the functions point to objects that are guaranteed to not be
// accessed by other threads.

NpTsdfIntegratorBase::NpTsdfIntegratorBase(const Config& config,
                                       Layer<TsdfVoxel>* layer)
    : config_(config) {
  setLayer(layer);

  if (config_.integrator_threads == 0) {
    LOG(WARNING) << "Automatic core count failed, defaulting to 1 threads";
    config_.integrator_threads = 1;
  }
  // clearing rays have no utility if voxel_carving is disabled
  if (config_.allow_clear && !config_.voxel_carving_enabled) {
    config_.allow_clear = false;
  }
}

void NpTsdfIntegratorBase::setLayer(Layer<TsdfVoxel>* layer) {
  CHECK_NOTNULL(layer);
  layer_ = layer;
  voxel_size_ = layer_->voxel_size();
  block_size_ = layer_->block_size();
  voxels_per_side_ = layer_->voxels_per_side();
  voxel_size_inv_ = 1.0 / voxel_size_;
  block_size_inv_ = 1.0 / block_size_;
  voxels_per_side_inv_ = 1.0 / voxels_per_side_;
}

// Will return a pointer to a voxel located at global_voxel_idx in the tsdf
// layer. Thread safe.
// Takes in the last_block_idx and last_block to prevent unneeded map lookups.
// If the block this voxel would be in has not been allocated, a block in
// temp_block_map_ is created/accessed and a voxel from this map is returned
// instead. Unlike the layer, accessing temp_block_map_ is controlled via a
// mutex allowing it to grow during integration.
// These temporary blocks can be merged into the layer later by calling
// updateLayerWithStoredBlocks()
TsdfVoxel* NpTsdfIntegratorBase::allocateStorageAndGetVoxelPtr(
    const GlobalIndex& global_voxel_idx, Block<TsdfVoxel>::Ptr* last_block,
    BlockIndex* last_block_idx) {
  DCHECK(last_block != nullptr);
  DCHECK(last_block_idx != nullptr);

  const BlockIndex block_idx =
      getBlockIndexFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_inv_);

  if ((block_idx != *last_block_idx) || (*last_block == nullptr)) {
    *last_block = layer_->getBlockPtrByIndex(block_idx);
    *last_block_idx = block_idx;
  }

  // If no block at this location currently exists, we allocate a temporary
  // voxel that will be merged into the map later
  if (*last_block == nullptr) {
    // To allow temp_block_map_ to grow we can only let one thread in at once
    std::lock_guard<std::mutex> lock(temp_block_mutex_);

    typename Layer<TsdfVoxel>::BlockHashMap::iterator it =
        temp_block_map_.find(block_idx);
    if (it != temp_block_map_.end()) {
      *last_block = it->second;
    } else {
      auto insert_status = temp_block_map_.emplace(
          block_idx, std::make_shared<Block<TsdfVoxel>>(
                         voxels_per_side_, voxel_size_,
                         getOriginPointFromGridIndex(block_idx, block_size_)));

      DCHECK(insert_status.second) << "Block already exists when allocating at "
                                   << block_idx.transpose();

      *last_block = insert_status.first->second;
    }
  }

  (*last_block)->setUpdatedAll();

  const VoxelIndex local_voxel_idx =
      getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);

  return &((*last_block)->getVoxelByVoxelIndex(local_voxel_idx));
}

// NOT thread safe
void NpTsdfIntegratorBase::updateLayerWithStoredBlocks() {
  BlockIndex last_block_idx;
  Block<TsdfVoxel>::Ptr block = nullptr;

  for (const std::pair<const BlockIndex, Block<TsdfVoxel>::Ptr>&
           temp_block_pair : temp_block_map_) {
    layer_->insertBlock(temp_block_pair);
  }

  temp_block_map_.clear();
}

// Thread safe.
// Figure out whether the voxel is behind or in front of the surface.
// To do this, project the voxel_center onto the ray from origin to point G.
// Then check if the the magnitude of the vector is smaller or greater than
// the original distance...
float NpTsdfIntegratorBase::computeDistance(const Point& origin,
                                            const Point& point_G,
                                            const Point& voxel_center) const {
  const Point v_voxel_origin = voxel_center - origin;
  const Point v_point_origin = point_G - origin;

  const FloatingPoint dist_G = v_point_origin.norm();
  // projection of a (v_voxel_origin) onto b (v_point_origin)
  const FloatingPoint dist_G_V = v_voxel_origin.dot(v_point_origin) / dist_G;

  const float sdf = static_cast<float>(dist_G - dist_G_V);
  return sdf;
}

// Thread safe.
// Only the first part of computeVoxelWeight
float NpTsdfIntegratorBase::getVoxelWeight(const Point& point_C) const {
  if (config_.use_const_weight) {
    return 1.0f;
  }
  const FloatingPoint dist = std::abs(point_C.norm());
  if (dist > kEpsilon) {
    return 1.0f / std::pow(dist, config_.weight_reduction_exp);
  }
  return 0.0f;
}

// Compute the weight for current measurement
float NpTsdfIntegratorBase::computeVoxelWeight(const Point& point_C,
                                               const float sdf,
                                               const bool with_init_weight,
                                               const float init_weight) const {
  float weight = 1.0;
  if (with_init_weight) {
    weight = init_weight;
  } else {
    // Part 1. Weight reduction with distance
    // (according to sensor noise models).
    // Also Independent of sdf
    if (!config_.use_const_weight) {
      weight /= std::pow(point_C.norm(), config_.weight_reduction_exp);
    }
  }

  // Part 2. weight drop-off
  // Apply weight drop-off if appropriate.
  const float truncation_distance = config_.default_truncation_distance;
  if (config_.use_weight_dropoff) {
    const float dropoff_epsilon =
        config_.weight_dropoff_epsilon > 0.f
            ? config_.weight_dropoff_epsilon
            : config_.weight_dropoff_epsilon * -voxel_size_;
    // for example, weight_dropoff_epsilon = -1.0
    // --> dropoff_epsilon = voxel_size
    if (sdf < -dropoff_epsilon) {
      weight *=
          (truncation_distance + sdf) / (truncation_distance - dropoff_epsilon);
      weight = std::max(weight, 0.f);
    }
  }

  // Part 3. deal with sparse point cloud
  // Compute the updated weight in case we compensate for sparsity. By
  // multiplicating the weight of occupied areas (|sdf| < truncation distance)
  // by a factor, we prevent to easily fade out these areas with the free
  // space parts of other rays which pass through the corresponding voxels.
  // This can be useful for creating a TSDF map from sparse sensor data (e.g.
  // visual features from a SLAM system). By default, this option is disabled.
  if (config_.use_sparsity_compensation_factor) {
    if (std::abs(sdf) < truncation_distance) {
      weight *= config_.sparsity_compensation_factor;
    }
  }
  return weight;
}

// each voxel has a distance and a weight
// once a new distance and weight is calculated, update it as
// a kind of weighted average
void NpTsdfIntegratorBase::updateTsdfVoxelValue(TsdfVoxel *voxel,
                                                const float sdf,
                                                const float weight,
                                                const Color *color) const {
  float new_weight = voxel->weight + weight;
  // it is possible to have weights very close to zero, due to the limited
  // precision of floating points dividing by this small value can cause nans
  if (new_weight < kFloatEpsilon) {
    return;
  }
  // Weighted averaging fusion.
  voxel->distance =
      (voxel->distance * voxel->weight + sdf * weight) / new_weight;
  // voxel->distance = std::max(-config_.default_truncation_distance,
  //                   std::min(config_.default_truncation_distance,
  //                   voxel->distance));

  voxel->weight = std::min(new_weight, config_.max_weight);
  // also take average of the color
  if (color != nullptr) {
    voxel->color =
        Color::blendTwoColors(voxel->color, voxel->weight, *color, weight);
  }
}

void NpTsdfIntegratorBase::updateTsdfVoxelGradient(TsdfVoxel *voxel,
                                                   const Ray normal,
                                                   const float weight) const {
  float new_weight = voxel->weight + weight;
  // it is possible to have weights very close to zero, due to the limited
  // precision of floating points dividing by this small value can cause nans
  if (new_weight < kFloatEpsilon) {
    return;
  }

  if (voxel->gradient.norm() > kFloatEpsilon) {
    voxel->gradient = ((voxel->gradient * voxel->weight + normal * weight) /
                      new_weight).normalized();
  } else {
    // newly assigned, originally zero vector
    voxel->gradient = normal.normalized();
  }
}

// Updates tsdf_voxel. Thread safe. (basic processing unit for each voxel)
// Frames: C: current frame, G: global frame
void NpTsdfIntegratorBase::updateTsdfVoxel(const Transformation& T_G_C,
                                           const Point& origin,
                                           const Point& point_C,
                                           const Point& point_G,
                                           const Ray& normal_C,
                                           const Ray& normal_G,
                                           const GlobalIndex& global_voxel_idx,
                                           const Color& color,
                                           const float init_weight,
                                           TsdfVoxel* tsdf_voxel) {
  DCHECK(tsdf_voxel != nullptr);

  const Point voxel_center =
      getCenterPointFromGridIndex(global_voxel_idx, voxel_size_);

  float sdf = computeDistance(origin, point_G, voxel_center);

  const float truncation_distance = config_.default_truncation_distance;

  // Lookup the mutex that is responsible for this voxel and lock it
  std::lock_guard<std::mutex> lock(mutexes_.get(global_voxel_idx));

  bool in_the_reliable_band = true;
  if (sdf > truncation_distance * config_.reliable_band_ratio)
    in_the_reliable_band = false;

  Ray gradient_C;
  if (config_.normal_available && in_the_reliable_band) {
    float normal_ratio = 1.0f;
    // use current un-updated normal because the weight is unknown
    if (tsdf_voxel->gradient.norm() > kFloatEpsilon) {
      // transform back to sensor(camera)'s frame
      gradient_C = T_G_C.inverse().getRotationMatrix() * tsdf_voxel->gradient;
      if (config_.curve_assumption && normal_C.norm() > kFloatEpsilon) {
        // condition 1: curve assumption
        // [sin(theta+alpha)-sin(theta)]/sin(alpha)]
        float cos_theta = std::abs(gradient_C.dot(point_C) / point_C.norm());
        float cos_alpha = std::abs(gradient_C.dot(normal_C) / normal_C.norm());
        float sin_theta = std::sqrt(1-cos_theta*cos_theta);
        float sin_alpha = std::sqrt(1-cos_alpha*cos_alpha);
        normal_ratio = std::abs(sin_theta * (cos_alpha-1) / sin_alpha + cos_theta); // NOLINT
        if (std::isnan(normal_ratio))
          normal_ratio = cos_theta;
      } else {
        // condition 2: flat surface, cos(theta)
        normal_ratio = std::abs(gradient_C.dot(point_C) / point_C.norm());
      }
    } else {
      // gradient not ready yet, use the first (current) normal vector
      // NOTE(py): kFloatEpsilon is a safe value in case of numerical rounding
      // error
      if (normal_C.norm() > kFloatEpsilon) {
        // current normal is valid
        normal_ratio = std::abs(normal_C.dot(point_C)/point_C.norm());
      }
    }
    // NOTE(py): ruling out extremely large incidence angle
    if (normal_ratio < config_.reliable_normal_ratio_thre)
      return;
    // get the non-projective sdf, if it's still larger than truncation
    // distance, the gradient would not be updated
    sdf *= normal_ratio;
  }
  if (sdf < -truncation_distance) {
    return;
  }
  bool with_init_weight = false;
  if (init_weight > 0)
    with_init_weight = true;
  float weight = computeVoxelWeight(point_C, sdf,
    with_init_weight, init_weight);
  // it is possible to have weights very close to zero, due to the limited
  // precision of floating points dividing by this small value can cause nans
  if (weight < kFloatEpsilon) {
    return;
  }
  // Only merge color and classification data near the surface.
  if (sdf > truncation_distance) {
    // far away, do not interpolate color
    updateTsdfVoxelValue(tsdf_voxel, sdf, weight);
    tsdf_voxel->distance = std::min(truncation_distance, tsdf_voxel->distance);
  } else {
    // close to the surface
    if (config_.normal_available) {
      // only update the gradient close to the surface
      if (normal_G.norm() > kFloatEpsilon)
        updateTsdfVoxelGradient(tsdf_voxel, normal_G, weight);
    }
    // blend color
    updateTsdfVoxelValue(tsdf_voxel, sdf, weight, &color);
  }
}

// Simple Integrator
// main entrance
void SimpleNpTsdfIntegrator::integratePointCloud(const Transformation& T_G_C,
                                                 const Pointcloud& points_C,
                                                 const Pointcloud& normals_C,
                                                 const Colors& colors,
                                                 const bool freespace_points) {
  timing::Timer integrate_timer("integrate_np_tsdf/simple");
  CHECK_EQ(points_C.size(), colors.size());

  std::unique_ptr<ThreadSafeIndex> index_getter(
      ThreadSafeIndexFactory::get(config_.integration_order_mode, points_C));

  std::list<std::thread> integration_threads;
  for (size_t i = 0; i < config_.integrator_threads; ++i) {
    integration_threads.emplace_back(&SimpleNpTsdfIntegrator::integrateFunction,
                                     this, T_G_C, points_C, normals_C, colors,
                                     freespace_points, index_getter.get());
  }

  for (std::thread& thread : integration_threads) {
    thread.join();
  }
  integrate_timer.Stop();

  // timing::Timer insertion_timer("inserting_missed_blocks");
  updateLayerWithStoredBlocks();
  // insertion_timer.Stop();
}

void SimpleNpTsdfIntegrator::integrateFunction(const Transformation& T_G_C,
                                               const Pointcloud& points_C,
                                               const Pointcloud& normals_C,
                                               const Colors& colors,
                                               const bool freespace_points,
                                               ThreadSafeIndex* index_getter) {
  DCHECK(index_getter != nullptr);

  size_t point_idx;
  while (index_getter->getNextIndex(&point_idx)) {
    const Point& point_C = points_C[point_idx];
    const Ray& normal_C = normals_C[point_idx];
    const Color& color = colors[point_idx];
    bool is_clearing;
    if (!isPointValid(point_C, freespace_points, &is_clearing)) {
      continue;
    }

    const Point origin = T_G_C.getPosition();
    const Point point_G = T_G_C * point_C;
    const Ray normal_G = T_G_C.getRotationMatrix() * normal_C;

    RayCaster ray_caster(origin, point_G, is_clearing,
                         config_.voxel_carving_enabled,
                         config_.max_ray_length_m, voxel_size_inv_,
                         config_.default_truncation_distance);

    Block<TsdfVoxel>::Ptr block = nullptr;
    BlockIndex block_idx;
    GlobalIndex global_voxel_idx;
    while (ray_caster.nextRayIndex(&global_voxel_idx)) {
      TsdfVoxel* voxel =
          allocateStorageAndGetVoxelPtr(global_voxel_idx, &block, &block_idx);
      updateTsdfVoxel(T_G_C, origin, point_C, point_G, normal_C, normal_G,
                      global_voxel_idx, color, 0.0, voxel);
    }
  }
}

// Megred "Bundled" Integrator
// main entrance
void MergedNpTsdfIntegrator::integratePointCloud(const Transformation& T_G_C,
                                                 const Pointcloud& points_C,
                                                 const Pointcloud& normals_C,
                                                 const Colors& colors,
                                                 const bool freespace_points) {
  timing::Timer integrate_timer("integrate_np_tsdf/merged");
  CHECK_EQ(points_C.size(), colors.size());
  CHECK_EQ(points_C.size(), normals_C.size());
  // Pre-compute a list of unique voxels to end on.
  // Create a hashmap: VOXEL INDEX -> index in original cloud.
  LongIndexHashMapType<AlignedVector<size_t>>::type voxel_map;
  // This is a hash map (same as above) to all the indices that need to be
  // cleared.
  LongIndexHashMapType<AlignedVector<size_t>>::type clear_map;

  std::unique_ptr<ThreadSafeIndex> index_getter(
      ThreadSafeIndexFactory::get(config_.integration_order_mode, points_C));

  // bundle rays in each voxel with point inside
  // TODO(py): make it multi-thread
  timing::Timer bundle_timer("integrate_np_tsdf/merged/bundle");
  bundleRays(T_G_C, points_C, freespace_points,
             index_getter.get(), &voxel_map, &clear_map);
  bundle_timer.Stop();

  // integrate rays for non-clearing voxel (close to the surface)
  timing::Timer nonclear_timer("integrate_np_tsdf/merged/nonclear");
  integrateRays(T_G_C, points_C, normals_C, colors,
                config_.enable_anti_grazing, false, voxel_map, clear_map);
  nonclear_timer.Stop();

  if (config_.merge_with_clear) {
    timing::Timer clear_timer("integrate_np_tsdf/merged/clear");
    // integrate rays for clearing voxels (away from the surface)
    integrateRays(T_G_C, points_C, normals_C, colors,
                  config_.enable_anti_grazing, true, voxel_map, clear_map);
    clear_timer.Stop();
  }
  integrate_timer.Stop();
}

void MergedNpTsdfIntegrator::bundleRays(
    const Transformation& T_G_C, const Pointcloud& points_C,
    const bool freespace_points, ThreadSafeIndex* index_getter,
    LongIndexHashMapType<AlignedVector<size_t>>::type* voxel_map,
    LongIndexHashMapType<AlignedVector<size_t>>::type* clear_map) {
  DCHECK(voxel_map != nullptr);
  DCHECK(clear_map != nullptr);

  size_t point_idx;
  while (index_getter->getNextIndex(&point_idx)) {
    const Point& point_C = points_C[point_idx];
    bool is_clearing;
    if (!isPointValid(point_C, freespace_points, &is_clearing)) {
      continue;
    }

    const Point point_G = T_G_C * point_C;

    GlobalIndex voxel_index =
        getGridIndexFromPoint<GlobalIndex>(point_G, voxel_size_inv_);

    if (is_clearing) {
      (*clear_map)[voxel_index].push_back(point_idx);
    } else {
      (*voxel_map)[voxel_index].push_back(point_idx);
    }
  }

  VLOG(3) << "Went from " << points_C.size() << " points to "
          << voxel_map->size() << " raycasts  and " << clear_map->size()
          << " clear rays.";
}

void MergedNpTsdfIntegrator::integrateRays(
    const Transformation& T_G_C, const Pointcloud& points_C,
    const Pointcloud& normals_C,
    const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
    const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map,
    const LongIndexHashMapType<AlignedVector<size_t>>::type& clear_map) {
  // if only 1 thread just do function call, otherwise spawn threads
  if (config_.integrator_threads == 1) {
    constexpr size_t thread_idx = 0;
    integrateVoxels(T_G_C, points_C, normals_C, colors, enable_anti_grazing,
                    clearing_ray, voxel_map, clear_map, thread_idx);
  } else {
    std::list<std::thread> integration_threads;
    for (size_t i = 0; i < config_.integrator_threads; ++i) {
      integration_threads.emplace_back(
          &MergedNpTsdfIntegrator::integrateVoxels, this,
          T_G_C, points_C, normals_C, colors,
          enable_anti_grazing, clearing_ray, voxel_map, clear_map, i);
    }
    for (std::thread& thread : integration_threads) {
      thread.join();
    }
  }
  // timing::Timer insertion_timer("inserting_missed_blocks");
  updateLayerWithStoredBlocks();

  // insertion_timer.Stop();
}

void MergedNpTsdfIntegrator::integrateVoxels(
    const Transformation& T_G_C, const Pointcloud& points_C,
    const Pointcloud& normals_C,
    const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
    const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map,
    const LongIndexHashMapType<AlignedVector<size_t>>::type& clear_map,
    size_t thread_idx) {
  LongIndexHashMapType<AlignedVector<size_t>>::type::const_iterator it;
  size_t map_size;
  if (clearing_ray) {
    it = clear_map.begin();
    map_size = clear_map.size();
  } else {
    it = voxel_map.begin();
    map_size = voxel_map.size();
  }

  for (size_t i = 0; i < map_size; ++i) {
    if (((i + thread_idx + 1) % config_.integrator_threads) == 0) {
      integrateVoxel(T_G_C, points_C, normals_C, colors, enable_anti_grazing,
                     clearing_ray, *it, voxel_map);
    }
    ++it;
  }
}

void MergedNpTsdfIntegrator::integrateVoxel(
    const Transformation& T_G_C, const Pointcloud& points_C,
    const Pointcloud& normals_C,
    const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
    const std::pair<GlobalIndex, AlignedVector<size_t>>& kv,
    const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map) {
  if (kv.second.empty()) {
    return;
  }

  const Point& origin = T_G_C.getPosition();
  Color merged_color;
  Point merged_point_C = Point::Zero();
  Ray merged_normal_C = Point::Zero();
  FloatingPoint merged_weight = 0.0;

  for (const size_t pt_idx : kv.second) {
    const Point& point_C = points_C[pt_idx];
    const Ray& normal_C = normals_C[pt_idx];
    const Color& color = colors[pt_idx];
    // only the reduction part
    const float point_weight = getVoxelWeight(point_C);
    if (point_weight < kEpsilon) {
      continue;
    }
    merged_point_C = (merged_point_C * merged_weight + point_C * point_weight) /
                     (merged_weight + point_weight);
    merged_color =
        Color::blendTwoColors(merged_color, merged_weight, color, point_weight);
    if (config_.normal_available) {
      merged_normal_C = merged_normal_C * merged_weight +
        normal_C * point_weight;
      if (merged_normal_C.norm() > kEpsilon) {
        merged_normal_C.normalize();
      }
    }
    merged_weight += point_weight;

    // only take first point when clearing
    if (clearing_ray) {
      break;
    }
  }

  const Point merged_point_G = T_G_C * merged_point_C;
  const Ray merged_normal_G = T_G_C * merged_normal_C;

  RayCaster ray_caster(origin, merged_point_G, clearing_ray,
                       config_.voxel_carving_enabled, config_.max_ray_length_m,
                       voxel_size_inv_, config_.default_truncation_distance);

  GlobalIndex global_voxel_idx;
  while (ray_caster.nextRayIndex(&global_voxel_idx)) {
    if (enable_anti_grazing) {
      // Check if this one is already the block hash map for this
      // insertion. Skip this to avoid grazing.
      if ((clearing_ray || global_voxel_idx != kv.first) &&
          voxel_map.find(global_voxel_idx) != voxel_map.end()) {
        continue;
      }
    }

    Block<TsdfVoxel>::Ptr block = nullptr;
    BlockIndex block_idx;
    TsdfVoxel* voxel =
        allocateStorageAndGetVoxelPtr(global_voxel_idx, &block, &block_idx);

    updateTsdfVoxel(T_G_C, origin, merged_point_C, merged_point_G,
                    merged_normal_C, merged_normal_G,
                    global_voxel_idx, merged_color,
                    merged_weight, voxel);
  }
}

// Fast Integrator
// main entrance
void FastNpTsdfIntegrator::integratePointCloud(const Transformation& T_G_C,
                                               const Pointcloud& points_C,
                                               const Pointcloud& normals_C,
                                               const Colors& colors,
                                               const bool freespace_points) {
  timing::Timer integrate_timer("integrate_np_tsdf/fast");
  CHECK_EQ(points_C.size(), colors.size());
  CHECK_EQ(points_C.size(), normals_C.size());

  integration_start_time_ = std::chrono::steady_clock::now();

  static int64_t reset_counter = 0;
  if ((++reset_counter) >= config_.clear_checks_every_n_frames) {
    reset_counter = 0;
    start_voxel_approx_set_.resetApproxSet();
    voxel_observed_approx_set_.resetApproxSet();
  }

  std::unique_ptr<ThreadSafeIndex> index_getter(
      ThreadSafeIndexFactory::get(config_.integration_order_mode, points_C));

  std::list<std::thread> integration_threads;
  for (size_t i = 0; i < config_.integrator_threads; ++i) {
    integration_threads.emplace_back(&FastNpTsdfIntegrator::integrateFunction,
                                     this, T_G_C, points_C, normals_C, colors,
                                     freespace_points, index_getter.get());
  }

  for (std::thread& thread : integration_threads) {
    thread.join();
  }

  integrate_timer.Stop();

  // timing::Timer insertion_timer("inserting_missed_blocks");
  updateLayerWithStoredBlocks();
  // insertion_timer.Stop();
}

void FastNpTsdfIntegrator::integrateFunction(const Transformation& T_G_C,
                                           const Pointcloud& points_C,
                                           const Pointcloud& normals_C,
                                           const Colors& colors,
                                           const bool freespace_points,
                                           ThreadSafeIndex* index_getter) {
  DCHECK(index_getter != nullptr);

  size_t point_idx;
  while (index_getter->getNextIndex(&point_idx) &&
         (std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now() - integration_start_time_)
              .count() < config_.max_integration_time_s * 1000000)) {
    const Point& point_C = points_C[point_idx];
    const Ray& normal_C = normals_C[point_idx];
    const Color& color = colors[point_idx];
    bool is_clearing;
    if (!isPointValid(point_C, freespace_points, &is_clearing)) {
      continue;
    }

    const Point origin = T_G_C.getPosition();
    const Point point_G = T_G_C * point_C;
    const Ray normal_G = T_G_C.getRotationMatrix() * normal_C;
    // Checks to see if another ray in this scan has already started 'close'
    // to this location. If it has then we skip ray casting this point. We
    // measure if a start location is 'close' to another points by inserting
    // the point into a set of voxels. This voxel set has a resolution
    // start_voxel_subsampling_factor times higher then the voxel size.
    GlobalIndex global_voxel_idx;
    global_voxel_idx = getGridIndexFromPoint<GlobalIndex>(
        point_G, config_.start_voxel_subsampling_factor * voxel_size_inv_);
    if (!start_voxel_approx_set_.replaceHash(global_voxel_idx)) {
      continue;
    }

    constexpr bool cast_from_origin = false;
    RayCaster ray_caster(origin, point_G, is_clearing,
                         config_.voxel_carving_enabled,
                         config_.max_ray_length_m, voxel_size_inv_,
                         config_.default_truncation_distance, cast_from_origin);

    int64_t consecutive_ray_collisions = 0;

    Block<TsdfVoxel>::Ptr block = nullptr;
    BlockIndex block_idx;
    while (ray_caster.nextRayIndex(&global_voxel_idx)) {
      // Check if the current voxel has been seen by any ray cast this scan.
      // If it has increment the consecutive_ray_collisions counter, otherwise
      // reset it. If the counter reaches a threshold we stop casting as the
      // ray is deemed to be contributing too little new information.
      if (!voxel_observed_approx_set_.replaceHash(global_voxel_idx)) {
        ++consecutive_ray_collisions;
      } else {
        consecutive_ray_collisions = 0;
      }
      if (consecutive_ray_collisions > config_.max_consecutive_ray_collisions) {
        break;
      }

      TsdfVoxel* voxel =
          allocateStorageAndGetVoxelPtr(global_voxel_idx, &block, &block_idx);

      updateTsdfVoxel(T_G_C, origin, point_C, point_G, normal_C, normal_G,
                      global_voxel_idx, color, 0.0, voxel);
    }
  }
}

std::string NpTsdfIntegratorBase::Config::print() const {
  std::stringstream ss;
  // clang-format off
  ss << "================== TSDF Integrator Config ====================\n";
  ss << " General: \n";
  ss << " - default_truncation_distance:               " << default_truncation_distance << "\n";
  ss << " - max_weight:                                " << max_weight << "\n";
  ss << " - voxel_carving_enabled:                     " << voxel_carving_enabled << "\n";
  ss << " - min_ray_length_m:                          " << min_ray_length_m << "\n";
  ss << " - max_ray_length_m:                          " << max_ray_length_m << "\n";
  ss << " - use_const_weight:                          " << use_const_weight << "\n";
  ss << " - weight_reduction_exp:                      " << weight_reduction_exp << "\n";
  ss << " - allow_clear:                               " << allow_clear << "\n";
  ss << " - use_weight_dropoff:                        " << use_weight_dropoff << "\n";
  ss << " - weight_dropoff_epsilon:                    " << weight_dropoff_epsilon << "\n";
  ss << " - use_sparsity_compensation_factor:          " << use_sparsity_compensation_factor << "\n";
  ss << " - sparsity_compensation_factor:              "  << sparsity_compensation_factor << "\n";
  ss << " - integrator_threads:                        " << integrator_threads << "\n";
  ss << " - normal_available:                          " << normal_available << "\n";
  ss << " - reliable_band_ratio:                       " << reliable_band_ratio << "\n";
  ss << " - curve_assumption:                          " << curve_assumption << "\n";
  ss << " - reliable_normal_ratio_thre:                " << reliable_normal_ratio_thre << "\n";
  ss << " MergedNpTsdfIntegrator: \n";
  ss << " - enable_anti_grazing:                       " << enable_anti_grazing << "\n";
  ss << " FastNpTsdfIntegrator: \n";
  ss << " - start_voxel_subsampling_factor:            " << start_voxel_subsampling_factor << "\n";
  ss << " - max_consecutive_ray_collisions:            " << max_consecutive_ray_collisions << "\n";
  ss << " - clear_checks_every_n_frames:               " << clear_checks_every_n_frames << "\n";
  ss << " - max_integration_time_s:                    " << max_integration_time_s << "\n";
  ss << "==============================================================\n";
  // clang-format on
  return ss.str();
}

}  // namespace voxblox

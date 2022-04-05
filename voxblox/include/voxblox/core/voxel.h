#ifndef VOXBLOX_CORE_VOXEL_H_
#define VOXBLOX_CORE_VOXEL_H_

#include <climits>
#include <cstdint>
#include <float.h>
#include <string>

#include "voxblox/core/color.h"
#include "voxblox/core/common.h"

namespace voxblox {

#define INF FLT_MAX
#define UNDEF INT_MAX  // Undefined voxel index

struct TsdfVoxel {
  // signed distance, +: in front, -: behind the surface
  float distance = 0.0f;
  // confidence on the signed distance in this voxel
  float weight = 0.0f;
  Color color;
  // ADD(py): for the implementation of signed distance gradient, its direction
  // is from the surface toward the sensor // NOLINT
  Ray gradient = Ray::Zero();
  bool occupied = false;
};

// NOTE(py): the data structure here contains all the necessary varibles and
// intermediate data structure used for our Voxfield and also three baseline
// methods (Voxblox, FIESTA, EDT). Therefore, the memory cost is relatively
// large. For practical application, one can simply select those variables used
// specifically for Voxfield and comment the rest out to get rid of the
// redundant memory cost
struct EsdfVoxel {
  // when finer esdf is on, this distance also includes the inner-voxel part
  float distance = 0.0f;
  // without the inner-voxel part (between voxel centers)
  float raw_distance = 0.0f;
  bool observed = false;

  /**
   * Whether the voxel was copied from the TSDF (false) or created from a pose
   * or some other source (true). This member is not serialized!!!
   * Used mainly for path planning
   */
  bool hallucinated = false;
  // Used only by voxblox
  bool in_queue = false;
  // Whether the ESDF value is fixed as the same value of the colocated TSDF
  bool fixed = false;

  /**
   * Relative direction toward parent. If itself, then either uninitialized
   * or in the fixed frontier. (used only by Voxblox)
   */
  Eigen::Vector3i parent = Eigen::Vector3i::Zero();

  /**
   * Whether the voxel is behind (negative value) or in front of the surface
   * (positive value).
   * Use signed distance instead of unsigned distance in FIESTA
   * The original opensource implementation of FIESTA is unsigned
   */
  bool behind = false;
  // Whether the voxel is newly observed
  bool newly = false;
  /**
   * ESDF mapping error at this voxel
   * only for the visualization (used in mapping error evaluation)
   */
  float error = 0.0f;

  // distance square with the unit of the voxel size (deprecated)
  // int dist_square = 0;

  /**
   * Indicator for update scheduling
   * (used only for voxedt esdf integrator)
   */
  float raise = -1.0f;

  /**
   * Index of this voxel's closest occupied voxel
   * Used by FIESTA and Voxfield
   */
  GlobalIndex coc_idx = GlobalIndex(UNDEF, UNDEF, UNDEF);

  /**
   * Simplified version of a doubly linked list (prev, next, head)
   * Used by FIESTA and Voxfield
   */
  GlobalIndex prev_idx = GlobalIndex(UNDEF, UNDEF, UNDEF);
  GlobalIndex next_idx = GlobalIndex(UNDEF, UNDEF, UNDEF);
  GlobalIndex head_idx = GlobalIndex(UNDEF, UNDEF, UNDEF);

  // Index of this voxel itself
  GlobalIndex self_idx = GlobalIndex(UNDEF, UNDEF, UNDEF);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct OccupancyVoxel {
  float probability_log = 0.0f;
  bool observed = false;
  // check if probability_log > threshold
  bool occupied = false;
  // Fix FIESTA's problem of unsigned distance, use signed distance instead
  bool behind = false;
  bool fixed = false;
};

struct IntensityVoxel {
  float intensity = 0.0f;
  float weight = 0.0f;
};

/// Used for serialization only.
namespace voxel_types {
const std::string kNotSerializable = "not_serializable";
const std::string kTsdf = "tsdf";
const std::string kEsdf = "esdf";
const std::string kOccupancy = "occupancy";
const std::string kIntensity = "intensity";
}  // namespace voxel_types

template <typename Type>
std::string getVoxelType() {
  return voxel_types::kNotSerializable;
}

template <>
inline std::string getVoxelType<TsdfVoxel>() {
  return voxel_types::kTsdf;
}

template <>
inline std::string getVoxelType<EsdfVoxel>() {
  return voxel_types::kEsdf;
}

template <>
inline std::string getVoxelType<OccupancyVoxel>() {
  return voxel_types::kOccupancy;
}

template <>
inline std::string getVoxelType<IntensityVoxel>() {
  return voxel_types::kIntensity;
}

}  // namespace voxblox

#endif  // VOXBLOX_CORE_VOXEL_H_

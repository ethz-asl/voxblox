#ifndef VOXBLOX_CORE_VOXEL_H_
#define VOXBLOX_CORE_VOXEL_H_

#include <cstdint>
#include <string>
#include <vector>

#include "voxblox/core/color.h"
#include "voxblox/core/common.h"

namespace voxblox {

struct TsdfVoxel {
  float distance = 0.0f;
  float weight = 0.0f;
  Color color;

  /**
   * Whether the voxel was copied from the TSDF (false) or created from a pose
   * or some other source (true). This member is not serialized!!!
   */

  // If this voxel is considered ever-free for dynamic object detection.
  bool ever_free = false;

  // Last frame number a lidar point fell into this voxel.
  int last_lidar_occupied = -1;

  // Last frame number this voxel was occupied by the TSDF or a lidar point.
  int last_occupied = 0;

  // Number of consecutive frames this voxel contained lidar points.
  int occ_counter = 0;

  // Set to true if the voxel has already been clustered this frame.
  bool clustering_processed = false;

  // Marks this voxel as dynamic to adjust the next TSDF update.
  bool dynamic = false;
};

struct EsdfVoxel {
  float distance = 0.0f;

  bool observed = false;
  /**
   * Whether the voxel was copied from the TSDF (false) or created from a pose
   * or some other source (true). This member is not serialized!!!
   */
  bool hallucinated = false;
  bool in_queue = false;
  bool fixed = false;

  /**
   * Relative direction toward parent. If itself, then either uninitialized
   * or in the fixed frontier.
   */
  Eigen::Vector3i parent = Eigen::Vector3i::Zero();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct OccupancyVoxel {
  float probability_log = 0.0f;
  bool observed = false;
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

#ifndef VOXBLOX_CORE_VOXEL_H_
#define VOXBLOX_CORE_VOXEL_H_

#include <cstdint>

#include "voxblox/core/common.h"
#include "voxblox/core/color.h"

namespace voxblox {

struct TsdfVoxel {
  float distance = 0.0f;
  float weight = 0.0f;
  Color color;
};

struct EsdfVoxel {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  float distance = 0.0f;
  bool observed = false;
  bool in_queue = false;
  bool fixed = false;
  // Relative direction toward parent. If itself, then either uninitialized
  // or in the fixed frontier.
  Eigen::Vector3i parent = Eigen::Vector3i::Zero();
};

struct OccupancyVoxel {
  float probability_log = 0.0f;
  bool observed = false;
};

// Used for serialization only.
namespace voxel_types {
  const std::string kNotSerializable = "not_serializable";
  const std::string kTsdf = "tsdf";
  const std::string kEsdf = "esdf";
  const std::string kOccupancy = "occupancy";
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

}  // namespace voxblox

#endif  // VOXBLOX_CORE_VOXEL_H_

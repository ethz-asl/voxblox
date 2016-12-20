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

struct LabelVoxel {
  float label_confidence = 0.0f;
  Label label = 0u;
};

// Used for serialization only.
enum class VoxelTypes {
  kNotSerializable = 0,
  kTsdf = 1,
  kEsdf = 2,
  kOccupancy = 3,
  kLabel = 4
};

template <typename Type>
VoxelTypes getVoxelType() {
  return VoxelTypes::kNotSerializable;
}

template <>
inline VoxelTypes getVoxelType<TsdfVoxel>() {
  return VoxelTypes::kTsdf;
}

template <>
inline VoxelTypes getVoxelType<EsdfVoxel>() {
  return VoxelTypes::kEsdf;
}

template <>
inline VoxelTypes getVoxelType<OccupancyVoxel>() {
  return VoxelTypes::kOccupancy;
}

template <>
inline VoxelTypes getVoxelType<LabelVoxel>() {
  return VoxelTypes::kLabel;
}

}  // namespace voxblox

#endif  // VOXBLOX_CORE_VOXEL_H_

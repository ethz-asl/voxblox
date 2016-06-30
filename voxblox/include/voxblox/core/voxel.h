#ifndef VOXBLOX_CORE_VOXEL_H_
#define VOXBLOX_CORE_VOXEL_H_

#include <cstdint>

#include "voxblox/core/common.h"
#include "voxblox/core/color.h"

namespace voxblox {

struct TsdfVoxel {
  float distance = 0.0;
  float weight = 0.0;
  Color color;
};

struct EsdfVoxel {
  float distance = 0.0;
  bool observed = false;
  bool in_queue = false;
};

struct OccupancyVoxel {
  float probability_log = 0.0;
  bool observed = false;
};

// Used for serialization only.
enum class VoxelTypes {
  kNotSerializable = 0,
  kTsdf = 1,
  kEsdf = 2,
  kOccupancy = 3
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

}  // namespace voxblox

#endif  // VOXBLOX_CORE_VOXEL_H_

#ifndef VOXBLOX_CORE_VOXEL_H_
#define VOXBLOX_CORE_VOXEL_H_

#include <cstdint>

#include "voxblox/core/common.h"

namespace voxblox {

struct TsdfVoxel {
  float distance = 0.0;
  float weight = 0.0;
  Color color;
};

struct EsdfVoxel {
  float distance = 0.0;
  bool observed = false;
};

struct OccupancyVoxel {
  float occupancy_probability = 0.5;
  bool observed = false;
};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_VOXEL_H_

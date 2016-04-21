#ifndef VOXBLOX_CORE_VOXEL_H
#define VOXBLOX_CORE_VOXEL_H

#include "voxblox/core/common.h"
#include <cstdint>

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

#endif  // VOXBLOX_CORE_VOXEL_H

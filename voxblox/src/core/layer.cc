#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

template <>
Layer<TsdfVoxel>::Type Layer<TsdfVoxel>::getType() const {
  return kTsdf;
}

template <>
Layer<EsdfVoxel>::Type Layer<EsdfVoxel>::getType() const {
  return kEsdf;
}

template <>
Layer<OccupancyVoxel>::Type Layer<OccupancyVoxel>::getType() const {
  return kOccupancy;
}

}  // namespace voxblox

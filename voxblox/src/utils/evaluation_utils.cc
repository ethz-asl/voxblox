#include "voxblox/utils/evaluation_utils.h"

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

namespace voxblox {
namespace utils {

template <>
bool isObservedVoxel(const TsdfVoxel& voxel) {
  return voxel.weight > 1e-6;
}

template <>
bool isObservedVoxel(const EsdfVoxel& voxel) {
  return voxel.observed;
}

template <>
FloatingPoint getVoxelSdf(const TsdfVoxel& voxel) {
  return voxel.distance;
}

template <>
FloatingPoint getVoxelSdf(const EsdfVoxel& voxel) {
  return voxel.distance;
}

template <>
void setVoxelSdf(const FloatingPoint sdf, TsdfVoxel* voxel) {
  CHECK_NOTNULL(voxel);
  voxel->distance = sdf;
}

template <>
void setVoxelSdf(const FloatingPoint sdf, EsdfVoxel* voxel) {
  CHECK_NOTNULL(voxel);
  voxel->distance = sdf;
}

template <>
void setVoxelWeight(const FloatingPoint weight, TsdfVoxel* voxel) {
  CHECK_NOTNULL(voxel);
  voxel->weight = weight;
}

template <>
void setVoxelWeight(const FloatingPoint weight, EsdfVoxel* voxel) {
  CHECK_NOTNULL(voxel);
  voxel->observed = weight > 0.;
}

}  // namespace utils
}  // namespace voxblox

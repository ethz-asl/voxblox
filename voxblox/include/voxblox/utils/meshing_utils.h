#ifndef VOXBLOX_UTILS_MESHING_UTILS_H_
#define VOXBLOX_UTILS_MESHING_UTILS_H_

#include "voxblox/core/common.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

namespace utils {

template <typename VoxelType>
bool getSdfIfValid(const VoxelType& voxel, const FloatingPoint min_weight,
                   FloatingPoint* sdf);

template <>
inline bool getSdfIfValid(const TsdfVoxel& voxel,
                          const FloatingPoint min_weight, FloatingPoint* sdf) {
  DCHECK(sdf != nullptr);
  if (voxel.weight <= min_weight) {
    return false;
  }
  *sdf = voxel.distance;
  return true;
}

template <>
inline bool getSdfIfValid(const EsdfVoxel& voxel,
                          const FloatingPoint /*min_weight*/,
                          FloatingPoint* sdf) {
  DCHECK(sdf != nullptr);
  if (!voxel.observed) {
    return false;
  }
  *sdf = voxel.distance;
  return true;
}

template <typename VoxelType>
bool getColorIfValid(const VoxelType& voxel, const FloatingPoint min_weight,
                     Color* color);

template <>
inline bool getColorIfValid(const TsdfVoxel& voxel,
                            const FloatingPoint min_weight, Color* color) {
  DCHECK(color != nullptr);
  if (voxel.weight <= min_weight) {
    return false;
  }
  *color = voxel.color;
  return true;
}

template <>
inline bool getColorIfValid(const EsdfVoxel& voxel,
                            const FloatingPoint /*min_weight*/, Color* color) {
  DCHECK(color != nullptr);
  if (!voxel.observed) {
    return false;
  }
  *color = Color(255u, 255u, 255u);
  return true;
}

}  // namespace utils
}  // namespace voxblox

#endif  // VOXBLOX_UTILS_MESHING_UTILS_H_

#ifndef VOXBLOX_UTILS_VOXEL_UTILS_H_
#define VOXBLOX_UTILS_VOXEL_UTILS_H_

#include "voxblox/core/color.h"
#include "voxblox/core/common.h"
#include "voxblox/core/voxel.h"

namespace voxblox {
template <typename VoxelType>
void mergeVoxelAIntoVoxelB(const VoxelType& voxel_A, VoxelType* voxel_B);

template <>
void mergeVoxelAIntoVoxelB(const TsdfVoxel& voxel_A, TsdfVoxel* voxel_B);

template <>
void mergeVoxelAIntoVoxelB(const EsdfVoxel& voxel_A, EsdfVoxel* voxel_B);

template <>
void mergeVoxelAIntoVoxelB(const OccupancyVoxel& voxel_A,
                           OccupancyVoxel* voxel_B);

}  // namespace voxblox

#endif  // VOXBLOX_UTILS_VOXEL_UTILS_H_

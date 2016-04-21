#ifndef VOXBLOX_CORE_VOXEL_ARRAY_H
#define VOXBLOX_CORE_VOXEL_ARRAY_H

#include "voxblox/core/common.h"

namespace voxblox {

template <typename VoxelType>
struct VoxelArray {
 public:
  VoxelArray(size_t _voxels_per_side, FloatingPoint _voxel_size)
      : voxels_per_side(_voxels_per_side), voxel_size(_voxel_size) {
    num_voxels = voxels_per_side * voxels_per_side * voxels_per_side;
    voxel_size_inv = 1.0 / voxel_size;
    voxels.reset(new VoxelType[num_voxels]);
  }
  ~VoxelArray() {}

  // Base parameters.
  size_t voxels_per_side;
  FloatingPoint voxel_size;

  // Derived, cached parameters.
  size_t num_voxels;
  FloatingPoint voxel_size_inv;

  std::unique_ptr<VoxelType[]> voxels;
};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_VOXEL_ARRAY_H

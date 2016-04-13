#ifndef VOXBLOX_VOXEL_ARRAY_H
#define VOXBLOX_VOXEL_ARRAY_H

namespace voxblox {

template <typename VoxelType>
struct VoxelArray {
 public:
  VoxelArray(size_t voxels_per_side_) {
    // Create.
    voxels_per_side = voxels_per_side_;
    num_voxels = voxels_per_side * voxels_per_side * voxels_per_side;
    inv_voxels_per_side = 1.0 / voxels_per_side;

    voxels.reset(new VoxelType[num_voxels]);
  }

  size_t voxels_per_side;
  size_t num_voxels;
  float inv_voxels_per_side;

  std::unique_ptr<VoxelType> voxels;
};

}  // namespace voxblox

#endif  // VOXBLOX_VOXEL_ARRAY_H

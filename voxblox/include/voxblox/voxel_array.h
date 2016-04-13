#ifndef VOXBLOX_VOXEL_ARRAY_H
#define VOXBLOX_VOXEL_ARRAY_H

namespace voxblox {

template <typename VoxelType>
class VoxelArray {
 public:
  VoxelArray(size_t voxels_per_side_) {
    // Create.

    voxels_ = new voxels_[voxels_per_side_ * voxels_per_side_ * voxels_per_side_]
  }


  size_t voxels_per_side_;
  size_t num_voxels_;

  std::unique_ptr<VoxelType> voxels_;
};

}  // namespace voxblox

#endif  // VOXBLOX_VOXEL_ARRAY_H


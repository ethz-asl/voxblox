#include "voxblox/core/block.h"

namespace voxblox {

Block::Block(const BlockProto& proto)
    : voxels_per_side_(proto.voxels_per_side()),
      voxel_size_(proto.voxel_size()),
      origin_(proto.origin_x(), proto.origin_y(), proto.origin_z()) {

  num_voxels_ = voxels_per_side_ * voxels_per_side_ * voxels_per_side_;
  voxel_size_inv_ = 1.0 / voxel_size_;
  block_size_ = voxels_per_side_ * voxel_size_;

  voxels_.reset(new VoxelType[num_voxels_]);
  DeserializeVoxelData<VoxelType>(proto, voxels_.get());
}

void Block::DeserializeVoxelData(const BlockProto& proto, TsdfVoxel* voxels) {}
}  // namespace voxblox

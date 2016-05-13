#ifndef VOXBLOX_CORE_BLOCK_INL_H_
#define VOXBLOX_CORE_BLOCK_INL_H_

#include "./Block.pb.h"

namespace voxblox {

template <typename VoxelType>
Block<VoxelType>::Block(const BlockProto& proto)
    : voxels_per_side_(proto.voxels_per_side()),
      voxel_size_(proto.voxel_size()),
      origin_(proto.origin_x(), proto.origin_y(), proto.origin_z()),
      has_data_(proto.has_data()),
      updated_(false) {
  num_voxels_ = voxels_per_side_ * voxels_per_side_ * voxels_per_side_;
  voxel_size_inv_ = 1.0 / voxel_size_;
  block_size_ = voxels_per_side_ * voxel_size_;

  voxels_.reset(new VoxelType[num_voxels_]);
  DeserializeVoxelData(proto, voxels_.get());
}

template <typename VoxelType>
void Block<VoxelType>::getProto(BlockProto* proto) const {
  CHECK_NOTNULL(proto);

  proto->set_voxels_per_side(voxels_per_side_);
  proto->set_voxel_size(voxel_size_);

  proto->set_origin_x(origin_.x());
  proto->set_origin_y(origin_.y());
  proto->set_origin_z(origin_.z());

  proto->set_has_data(has_data_);

  SerializeVoxelData(voxels_.get(), proto);
}

template <typename VoxelType>
bool Block<VoxelType>::mergeBlock(const Block<VoxelType>& other_block) {
  // TODO(mfehr): implement
  LOG(FATAL) << "NOT IMPLEMENTED";
  return false;
}

template <typename VoxelType>
size_t Block<VoxelType>::getMemorySize() const {
  size_t size = 0u;

  // Calculate size of members
  size += sizeof(voxels_per_side_);
  size += sizeof(voxel_size_);
  size += sizeof(origin_);
  size += sizeof(num_voxels_);
  size += sizeof(voxel_size_inv_);
  size += sizeof(block_size_);

  size += sizeof(has_data_);
  size += sizeof(updated_);

  if (num_voxels_ > 0u) {
    size += num_voxels_ * sizeof(voxels_[0]);
  }
  return size;
}

}  // namespace voxblox

#endif  // VOXBLOX_CORE_BLOCK_INL_H_

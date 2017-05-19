#ifndef VOXBLOX_FAST_CORE_BLOCK_INL_H_
#define VOXBLOX_FAST_CORE_BLOCK_INL_H_

#include "./Block.pb.h"

#include <vector>

namespace voxblox_fast {

template <typename VoxelType>
Block<VoxelType>::Block(const BlockProto& proto)
    : Block(proto.voxels_per_side(), proto.voxel_size(),
            Point(proto.origin_x(), proto.origin_y(), proto.origin_z())) {
  has_data_ = proto.has_data();

  // Convert the data into a vector of integers.
  std::vector<uint32_t> data;
  data.reserve(proto.voxel_data_size());

  for (uint32_t word : proto.voxel_data()) {
    data.push_back(word);
  }

  deserializeFromIntegers(data);
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

  std::vector<uint32_t> data;
  serializeToIntegers(&data);
  // Not quite actually a word since we're in a 64-bit age now, but whatever.
  for (uint32_t word : data) {
    proto->add_voxel_data(word);
  }
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
    size += (num_voxels_ * sizeof(voxels_[0]));
  }
  return size;
}

}  // namespace voxblox

#endif  // VOXBLOX_FAST_CORE_BLOCK_INL_H_

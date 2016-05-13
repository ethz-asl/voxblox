#ifndef VOXBLOX_CORE_BLOCK_INL_H_
#define VOXBLOX_CORE_BLOCK_INL_H_

#include "./Block.pb.h"

namespace voxblox {

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
bool Block<VoxelType>::mergeBlock() {
  // TODO(mfehr): implement
  LOG(FATAL) << "NOT IMPLEMENTED";
  return false;
}

}  // namespace voxblox

#endif  // VOXBLOX_CORE_BLOCK_INL_H_

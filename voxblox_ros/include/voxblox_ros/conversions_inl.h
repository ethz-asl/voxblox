#ifndef VOXBLOX_ROS_CONVERSIONS_INL_H_
#define VOXBLOX_ROS_CONVERSIONS_INL_H_

#include <vector>

namespace voxblox {

template <typename VoxelType>
void serializeLayerAsMsg(const Layer<VoxelType>& layer, bool only_updated,
                         voxblox_msgs::Layer* msg) {
  CHECK_NOTNULL(msg);
  msg->voxels_per_side = layer.voxels_per_side();
  msg->voxel_size = layer.voxel_size();

  msg->layer_type = getVoxelType<VoxelType>();

  BlockIndexList block_list;
  if (only_updated) {
    layer.getAllUpdatedBlocks(&block_list);
    msg->action = voxblox_msgs::Layer::ACTION_UPDATE;
  } else {
    layer.getAllAllocatedBlocks(&block_list);
    msg->action = voxblox_msgs::Layer::ACTION_FULL_MAP;
  }

  voxblox_msgs::Block block_msg;
  msg->blocks.reserve(block_list.size());
  for (const BlockIndex& index : block_list) {
    block_msg.x_index = index.x();
    block_msg.y_index = index.y();
    block_msg.z_index = index.z();

    std::vector<uint32_t> data;
    layer.getBlockByIndex(index).serializeToIntegers(&data);

    block_msg.data = data;
    msg->blocks.push_back(block_msg);
  }
}

template <typename VoxelType>
bool deserializeMsgToLayer(const voxblox_msgs::Layer& msg,
                           Layer<VoxelType>* layer) {
  CHECK_NOTNULL(layer);
  if (getVoxelType<VoxelType>().compare(msg.layer_type) != 0) {
    return false;
  }

  // So we also need to check if the sizes match. If they don't, we can't
  // parse this at all.
  constexpr double kVoxelSizeEpsilon = 1e-5;
  if (msg.voxels_per_side != layer->voxels_per_side() ||
      std::abs(msg.voxel_size - layer->voxel_size()) > kVoxelSizeEpsilon) {
    return false;
  }

  // TODO(helenol): For now treat both actions the same. In the future should
  // clear the map for ACTION_FULL_MAP.
  for (const voxblox_msgs::Block& block_msg : msg.blocks) {
    // Create a new block if it doesn't exist yet, or get the existing one
    // at the correct block index.
    BlockIndex index(block_msg.x_index, block_msg.y_index, block_msg.z_index);
    typename Block<VoxelType>::Ptr block_ptr =
        layer->allocateBlockPtrByIndex(index);

    std::vector<uint32_t> data = block_msg.data;
    block_ptr->deserializeFromIntegers(data);
  }

  return true;
}

}  // namespace voxblox

#endif  // VOXBLOX_ROS_CONVERSIONS_INL_H_

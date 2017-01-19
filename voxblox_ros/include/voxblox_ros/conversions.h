#ifndef VOXBLOX_ROS_CONVERSIONS_H_
#define VOXBLOX_ROS_CONVERSIONS_H_

#include <algorithm>
#include <std_msgs/ColorRGBA.h>
#include <vector>
#include <voxblox_msgs/Layer.h>

#include <voxblox/core/common.h>
#include <voxblox/mesh/mesh.h>
#include <voxblox/core/layer.h>

namespace voxblox {

inline void colorVoxbloxToMsg(const Color& color,
                              std_msgs::ColorRGBA* color_msg) {
  CHECK_NOTNULL(color_msg);
  color_msg->r = color.r / 255.0;
  color_msg->g = color.g / 255.0;
  color_msg->b = color.b / 255.0;
  color_msg->a = color.a / 255.0;
}

inline void colorMsgToVoxblox(const std_msgs::ColorRGBA& color_msg,
                              Color* color) {
  CHECK_NOTNULL(color);
  color->r = static_cast<uint8_t>(color_msg.r * 255.0);
  color->g = static_cast<uint8_t>(color_msg.g * 255.0);
  color->b = static_cast<uint8_t>(color_msg.b * 255.0);
  color->a = static_cast<uint8_t>(color_msg.a * 255.0);
}

// Declarations
template <typename VoxelType>
void serializeLayerAsMsg(const Layer<VoxelType>& layer,
                         voxblox_msgs::Layer* msg);

// Returns true if could parse the data into the existing layer (all parameters
// are compatible), false otherwise.
template <typename VoxelType>
bool deserializeMsgToLayer(const voxblox_msgs::Layer& msg,
                           Layer<VoxelType>* layer);

// Definitions, maybe move to impl file.
template <typename VoxelType>
void serializeLayerAsMsg(const Layer<VoxelType>& layer, bool only_updated,
                         voxblox_msgs::Layer* msg) {
  msg->voxels_per_side = layer.voxels_per_side();
  msg->voxel_size = layer.voxel_size();

  msg->layer_type = static_cast<uint8_t>(getVoxelType<VoxelType>());

  BlockIndexList block_list;
  if (only_updated) {
    layer.getAllUpdatedBlocks(&block_list);
    msg->action = voxblox_msgs::Layer::ACTION_UPDATE;
  } else {
    layer.getAllAllocatedBlocks(&block_list);
    msg->action = voxblox_msgs::Layer::ACTION_FULL_MAP;
  }

  voxblox_msgs::Block block_msg;
  for (const BlockIndex& index : block_list) {
    block_msg.x_index = index.x();
    block_msg.y_index = index.y();
    block_msg.z_index = index.z();

    std::vector<uint32_t> data;
    layer.getBlockByIndex(index).serializeToIntegers(&data);

    std::copy(data.begin(), data.end(), block_msg.data.begin());
    // TODO(helenol): is this super slow???
    msg->blocks.push_back(block_msg);
  }
}

template <typename VoxelType>
bool deserializeMsgToLayer(const voxblox_msgs::Layer& msg,
                           Layer<VoxelType>* layer) {
  if (msg.layer_type != static_cast<uint8_t>(getVoxelType<VoxelType>())) {
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

    std::vector<uint32_t> data;
    std::copy(block_msg.data.begin(), block_msg.data.end(), data.begin());
    block_ptr->deserializeFromIntegers(data);
  }

  return true;
}

}  // namespace voxblox

#endif  // VOXBLOX_ROS_VISUALIZATION_H_

#ifndef VOXBLOX_FAST_CORE_LAYER_INL_H_
#define VOXBLOX_FAST_CORE_LAYER_INL_H_

#include <fstream>  // NOLINT
#include <utility>
#include <string>

#include <glog/logging.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/message.h>
#include <google/protobuf/message_lite.h>

#include "./Block.pb.h"
#include "./Layer.pb.h"
#include "voxblox_fast/core/block.h"
#include "voxblox_fast/core/voxel.h"
#include "voxblox_fast/utils/protobuf_utils.h"

namespace voxblox_fast {

template <typename VoxelType>
Layer<VoxelType>::Layer(const LayerProto& proto)
    : voxel_size_(proto.voxel_size()),
      voxels_per_side_(proto.voxels_per_side()) {
  CHECK_EQ(getType().compare(proto.type()), 0)
      << "Incorrect voxel type, proto type: " << proto.type()
      << " layer type: " << getType();

  // Derived config parameter.
  block_size_ = voxel_size_ * voxels_per_side_;
  block_size_inv_ = 1.0 / block_size_;

  CHECK_GT(proto.voxel_size(), 0.0);
  CHECK_GT(proto.voxels_per_side(), 0u);
}

template <typename VoxelType>
void Layer<VoxelType>::getProto(LayerProto* proto) const {
  CHECK_NOTNULL(proto);

  CHECK_NE(getType().compare(voxel_types::kNotSerializable), 0)
      << "The voxel type of this layer is not serializable!";

  proto->set_voxel_size(voxel_size_);
  proto->set_voxels_per_side(voxels_per_side_);
  proto->set_type(getType());
}

template <typename VoxelType>
bool Layer<VoxelType>::saveToFile(const std::string& file_path) const {
  constexpr bool kIncludeAllBlocks = true;
  return saveSubsetToFile(file_path, BlockIndexList(), kIncludeAllBlocks);
}

template <typename VoxelType>
bool Layer<VoxelType>::saveSubsetToFile(const std::string& file_path,
                                        BlockIndexList blocks_to_include,
                                        bool include_all_blocks) const {
  CHECK_NE(getType().compare(voxel_types::kNotSerializable), 0)
      << "The voxel type of this layer is not serializable!";

  CHECK(!file_path.empty());
  std::fstream outfile;
  outfile.open(file_path, std::fstream::out | std::fstream::binary);
  if (!outfile.is_open()) {
    LOG(ERROR) << "Could not open file for writing: " << file_path;
    return false;
  }

  // Only serialize the blocks if there are any.
  // Count the number of blocks that need to be serialized.
  size_t num_blocks_to_write = 0u;
  if ((include_all_blocks && !block_map_.empty()) ||
      !blocks_to_include.empty()) {
    for (const BlockMapPair& pair : block_map_) {
      bool write_block_to_file = include_all_blocks;

      if (!write_block_to_file) {
        BlockIndexList::const_iterator it = std::find(
            blocks_to_include.begin(), blocks_to_include.end(), pair.first);
        if (it != blocks_to_include.end()) {
          ++num_blocks_to_write;
        }
      } else {
        ++num_blocks_to_write;
      }
    }
  }
  if (include_all_blocks) {
    CHECK_EQ(num_blocks_to_write, block_map_.size());
  } else {
    CHECK_LE(num_blocks_to_write, block_map_.size());
    CHECK_LE(num_blocks_to_write, blocks_to_include.size());
  }

  // Write the total number of messages to the beginning of this file.
  // One layer header and then all the block maps
  const uint32_t num_messages = 1u + num_blocks_to_write;
  if (!utils::writeProtoMsgCountToStream(num_messages, &outfile)) {
    LOG(ERROR) << "Could not write message number to file.";
    outfile.close();
    return false;
  }

  // Write out the layer header.
  LayerProto proto_layer;
  getProto(&proto_layer);
  if (!utils::writeProtoMsgToStream(proto_layer, &outfile)) {
    LOG(ERROR) << "Could not write layer header message.";
    outfile.close();
    return false;
  }

  // Serialize blocks.
  for (const BlockMapPair& pair : block_map_) {
    bool write_block_to_file = include_all_blocks;
    if (!write_block_to_file) {
      BlockIndexList::const_iterator it = std::find(
          blocks_to_include.begin(), blocks_to_include.end(), pair.first);
      if (it != blocks_to_include.end()) {
        write_block_to_file = true;
      }
    }
    if (write_block_to_file) {
      BlockProto block_proto;
      pair.second->getProto(&block_proto);

      if (!utils::writeProtoMsgToStream(block_proto, &outfile)) {
        LOG(ERROR) << "Could not write block message.";
        outfile.close();
        return false;
      }
    }
  }
  outfile.close();
  return true;
}

template <typename VoxelType>
bool Layer<VoxelType>::addBlockFromProto(const BlockProto& block_proto,
                                         BlockMergingStrategy strategy) {
  CHECK_NE(getType().compare(voxel_types::kNotSerializable), 0)
      << "The voxel type of this layer is not serializable!";

  if (isCompatible(block_proto)) {
    typename BlockType::Ptr block_ptr(new BlockType(block_proto));
    const BlockIndex block_index =
        getGridIndexFromOriginPoint(block_ptr->origin(), block_size_inv_);
    switch (strategy) {
      case BlockMergingStrategy::kProhibit:
        CHECK_EQ(block_map_.count(block_index), 0u)
            << "Block collision at index: " << block_index;
        block_map_[block_index] = block_ptr;
      break;
      case BlockMergingStrategy::kReplace:
        block_map_[block_index] = block_ptr;
        break;
      case BlockMergingStrategy::kDiscard:
        block_map_.insert(std::make_pair(block_index, block_ptr));
        break;
      case BlockMergingStrategy::kMerge: {
        typename BlockHashMap::iterator it = block_map_.find(block_index);
        if (it == block_map_.end()) {
          block_map_[block_index] = block_ptr;
        } else {
          it->second->mergeBlock(*block_ptr);
        }
      } break;
      default:
        LOG(FATAL) << "Unknown BlockMergingStrategy: "
                   << static_cast<int>(strategy);
        return false;
    }
    // Mark that this block has been updated.
    block_map_[block_index]->updated() = true;
  } else {
    LOG(ERROR)
        << "The blocks from this protobuf are not compatible with this layer!";
    return false;
  }
  return true;
}

template <typename VoxelType>
bool Layer<VoxelType>::isCompatible(const LayerProto& layer_proto) const {
  bool compatible = true;
  compatible &= (layer_proto.voxel_size() == voxel_size_);
  compatible &= (layer_proto.voxels_per_side() == voxels_per_side_);
  compatible &= (getType().compare(layer_proto.type()) == 0);
  return compatible;
}

template <typename VoxelType>
bool Layer<VoxelType>::isCompatible(const BlockProto& block_proto) const {
  bool compatible = true;
  compatible &= (block_proto.voxel_size() == voxel_size_);
  compatible &= (block_proto.voxels_per_side() == voxels_per_side_);
  return compatible;
}

template <typename VoxelType>
size_t Layer<VoxelType>::getMemorySize() const {
  size_t size = 0u;

  // Calculate size of members
  size += sizeof(voxel_size_);
  size += sizeof(voxels_per_side_);
  size += sizeof(block_size_);
  size += sizeof(block_size_inv_);

  // Calculate size of blocks
  size_t num_blocks = getNumberOfAllocatedBlocks();
  if (num_blocks > 0u) {
    typename Block<VoxelType>::Ptr block = block_map_.begin()->second;
    size += num_blocks * block->getMemorySize();
  }
  return size;
}

template <typename VoxelType>
std::string Layer<VoxelType>::getType() const {
  return getVoxelType<VoxelType>();
}

}  // namespace voxblox

#endif  // VOXBLOX_FAST_CORE_LAYER_INL_H_

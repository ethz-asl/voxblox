#ifndef VOXBLOX_CORE_LAYER_INL_H_
#define VOXBLOX_CORE_LAYER_INL_H_

#include <fstream>  // NOLINT
#include <limits>
#include <string>
#include <utility>

#include <glog/logging.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/message.h>
#include <google/protobuf/message_lite.h>

#include "voxblox/Block.pb.h"
#include "voxblox/Layer.pb.h"
#include "voxblox/core/block.h"
#include "voxblox/core/voxel.h"
#include "voxblox/utils/protobuf_utils.h"

namespace voxblox {

template <typename VoxelType>
Layer<VoxelType>::Layer(const LayerProto& proto)
    : voxel_size_(proto.voxel_size()),
      voxels_per_side_(proto.voxels_per_side()) {
  CHECK_EQ(getType().compare(proto.type()), 0)
      << "Incorrect voxel type, proto type: " << proto.type()
      << " layer type: " << getType();

  // Derived config parameter.
  CHECK_GT(proto.voxel_size(), 0.0);
  voxel_size_inv_ = 1.0 / voxel_size_;
  block_size_ = voxel_size_ * voxels_per_side_;
  CHECK_GT(block_size_, 0.0);
  block_size_inv_ = 1.0 / block_size_;
  CHECK_GT(proto.voxels_per_side(), 0u);
  voxels_per_side_inv_ = 1.0f / static_cast<FloatingPoint>(voxels_per_side_);
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
Layer<VoxelType>::Layer(const Layer& other) {
  voxel_size_ = other.voxel_size_;
  voxel_size_inv_ = other.voxel_size_inv_;
  voxels_per_side_ = other.voxels_per_side_;
  voxels_per_side_inv_ = other.voxels_per_side_inv_;
  block_size_ = other.block_size_;
  block_size_inv_ = other.block_size_inv_;

  for (const typename BlockHashMap::value_type& key_value_pair :
       other.block_map_) {
    const BlockIndex& block_idx = key_value_pair.first;
    const typename BlockType::Ptr& block_ptr = key_value_pair.second;
    CHECK(block_ptr);

    typename BlockType::Ptr new_block = allocateBlockPtrByIndex(block_idx);
    CHECK(new_block);

    for (size_t linear_idx = 0u; linear_idx < block_ptr->num_voxels();
         ++linear_idx) {
      const VoxelType& voxel = block_ptr->getVoxelByLinearIndex(linear_idx);
      VoxelType& new_voxel = new_block->getVoxelByLinearIndex(linear_idx);
      new_voxel = voxel;
    }
  }
}

template <typename VoxelType>
bool Layer<VoxelType>::saveToFile(const std::string& file_path,
                                  bool clear_file) const {
  constexpr bool kIncludeAllBlocks = true;
  return saveSubsetToFile(file_path, BlockIndexList(), kIncludeAllBlocks,
                          clear_file);
}

template <typename VoxelType>
bool Layer<VoxelType>::saveSubsetToFile(const std::string& file_path,
                                        BlockIndexList blocks_to_include,
                                        bool include_all_blocks,
                                        bool clear_file) const {
  CHECK_NE(getType().compare(voxel_types::kNotSerializable), 0)
      << "The voxel type of this layer is not serializable!";

  CHECK(!file_path.empty());
  std::fstream outfile;
  // Will APPEND to the current file in case outputting multiple layers on the
  // same file, depending on the flag.
  std::ios_base::openmode file_flags = std::fstream::out | std::fstream::binary;
  if (!clear_file) {
    file_flags |= std::fstream::app | std::fstream::ate;
  } else {
    file_flags |= std::fstream::trunc;
  }
  outfile.open(file_path, file_flags);
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
  saveBlocksToStream(include_all_blocks, blocks_to_include, &outfile);

  outfile.close();
  return true;
}

template <typename VoxelType>
bool Layer<VoxelType>::saveBlocksToStream(bool include_all_blocks,
                                          BlockIndexList blocks_to_include,
                                          std::fstream* outfile_ptr) const {
  CHECK_NOTNULL(outfile_ptr);
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

      if (!utils::writeProtoMsgToStream(block_proto, outfile_ptr)) {
        LOG(ERROR) << "Could not write block message.";
        return false;
      }
    }
  }
  return true;
}

template <typename VoxelType>
bool Layer<VoxelType>::addBlockFromProto(const BlockProto& block_proto,
                                         BlockMergingStrategy strategy) {
  CHECK_NE(getType().compare(voxel_types::kNotSerializable), 0)
      << "The voxel type of this layer is not serializable!";

  if (isCompatible(block_proto)) {
    typename BlockType::Ptr block_ptr(new BlockType(block_proto));
    const BlockIndex block_index = getGridIndexFromOriginPoint<BlockIndex>(
        block_ptr->origin(), block_size_inv_);
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
    block_map_[block_index]->updated().set();
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
  compatible &= (std::fabs(layer_proto.voxel_size() - voxel_size_) <
                 std::numeric_limits<FloatingPoint>::epsilon());
  compatible &= (layer_proto.voxels_per_side() == voxels_per_side_);
  compatible &= (getType().compare(layer_proto.type()) == 0);

  if (!compatible) {
    LOG(WARNING)
        << "Voxel size of the loaded map is: " << layer_proto.voxel_size()
        << " but the current map is: " << voxel_size_ << " check passed? "
        << (std::fabs(layer_proto.voxel_size() - voxel_size_) <
            std::numeric_limits<FloatingPoint>::epsilon())
        << "\nVPS of the loaded map is: " << layer_proto.voxels_per_side()
        << " but the current map is: " << voxels_per_side_ << " check passed? "
        << (layer_proto.voxels_per_side() == voxels_per_side_)
        << "\nLayer type of the loaded map is: " << getType()
        << " but the current map is: " << layer_proto.type()
        << " check passed? " << (getType().compare(layer_proto.type()) == 0)
        << "\nAre the maps using the same floating-point type? "
        << (layer_proto.voxel_size() == voxel_size_) << std::endl;
  }
  return compatible;
}

template <typename VoxelType>
bool Layer<VoxelType>::isCompatible(const BlockProto& block_proto) const {
  bool compatible = true;
  compatible &= (std::fabs(block_proto.voxel_size() - voxel_size_) <
                 std::numeric_limits<FloatingPoint>::epsilon());
  compatible &=
      (block_proto.voxels_per_side() == static_cast<int>(voxels_per_side_));
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

#endif  // VOXBLOX_CORE_LAYER_INL_H_

#ifndef VOXBLOX_CORE_LAYER_INL_H_
#define VOXBLOX_CORE_LAYER_INL_H_

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
#include "voxblox/core/block.h"

namespace voxblox {

template <typename VoxelType>
Layer<VoxelType>::Layer(const LayerProto& proto)
    : voxel_size_(proto.voxel_size()),
      voxels_per_side_(proto.voxels_per_side()) {
  CHECK_EQ(getType(), static_cast<Type>(proto.type()))
      << "Incorrect voxel type, proto type: " << proto.type()
      << " layer type: " << static_cast<int>(getType());

  // Derived config parameter.
  block_size_ = voxel_size_ * voxels_per_side_;
  block_size_inv_ = 1.0 / block_size_;

  addBlocksFromProtoToLayer(proto, BlockMergingStrategy::kProhibit);
}

template <typename VoxelType>
Layer<VoxelType>::Layer(const std::string file_path) {
  CHECK(!file_path.empty());

  LayerProto proto_layer;
  readProtoLayerFromFile(file_path, &proto_layer);

  // Main config parameter.
  voxel_size_ = proto_layer.voxel_size();
  voxels_per_side_ = proto_layer.voxels_per_side();

  // Derived config parameter.
  block_size_ = voxel_size_ * voxels_per_side_;
  block_size_inv_ = 1.0 / block_size_;

  addBlocksFromProtoToLayer(proto_layer, BlockMergingStrategy::kProhibit);
}

template <typename VoxelType>
void Layer<VoxelType>::getProto(const BlockIndexList& blocks_to_include,
                                bool include_all, LayerProto* proto) const {
  CHECK_NOTNULL(proto);

  proto->set_voxel_size(voxel_size_);
  proto->set_voxels_per_side(voxels_per_side_);
  proto->set_type(static_cast<int32_t>(getType()));

  if (!include_all && blocks_to_include.empty()) {
    // If no blocks should be included, the conversion is done.
    return;
  }

  for (const BlockMapPair& pair : block_map_) {
    if (include_all) {
      pair.second->getProto(proto->add_blocks());
    } else {
      // Check if this block should be included.
      BlockIndexList::const_iterator it = std::find(
          blocks_to_include.begin(), blocks_to_include.end(), pair.first);
      if (it != blocks_to_include.end()) {
        pair.second->getProto(proto->add_blocks());
      }
    }
  }
}

template <typename VoxelType>
void Layer<VoxelType>::getProto(LayerProto* proto) const {
  constexpr bool kIncludeAllBlocks = true;
  getProto(BlockIndexList(), kIncludeAllBlocks, proto);
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
  CHECK(!file_path.empty());

  LayerProto proto_layer;
  getProto(blocks_to_include, include_all_blocks, &proto_layer);

  std::fstream layer_file;
  layer_file.open(file_path.c_str(), std::fstream::out | std::fstream::trunc |
                                         std::fstream::binary);
  if (!layer_file.is_open()) {
    LOG(ERROR) << "Couldn't open " << file_path << " for writing.";
    return false;
  }
  const bool success = proto_layer.SerializeToOstream(&layer_file);
  layer_file.close();
  if (!success) {
    LOG(ERROR) << "Failed to save Layer to file: " << file_path;
    return false;
  }
  return true;
}

template <typename VoxelType>
bool Layer<VoxelType>::loadBlocksFromFile(const std::string& file_path,
                                          BlockMergingStrategy strategy) {
  CHECK(!file_path.empty());

  LayerProto proto_layer;
  readProtoLayerFromFile(file_path, &proto_layer);

  if (isCompatible(proto_layer)) {
    return addProtoBlocksToLayer(proto_layer, strategy);
  } else {
    LOG(ERROR) << "The blocks loaded from '" << file_path
               << "' are not compatible with this layer!";
    return false;
  }

  return true;
}

template <typename VoxelType>
bool Layer<VoxelType>::addBlocksFromProtoToLayer(
    const LayerProto& proto_layer, BlockMergingStrategy strategy) {
  if (isCompatible(proto_layer)) {
    const size_t num_blocks = proto_layer.blocks_size();
    const size_t num_blocks_total = block_map_.size();
    block_map_.reserve(num_blocks_total);
    for (size_t block_idx = 0u; block_idx < num_blocks; ++block_idx) {
      typename BlockType::Ptr block_ptr(
          new BlockType(proto_layer.blocks(block_idx)));
      const BlockIndex block_index =
          computeBlockIndexFromCoordinates(block_ptr->origin());
      switch (strategy) {
        case BlockMergingStrategy::kProhibit: {
          typename BlockHashMap::iterator it = block_map_.find(block_index);
          if (it == block_map_.end()) {
            block_map_[block_index] = block_ptr;
          } else {
            LOG(ERROR) << "New index: " << block_index.transpose()
                       << " collides with old index: " << it->first.transpose();
          }
        }

        /*
                  CHECK_EQ(block_map_.count(block_index), 0u) << "New index: "
           << block_index;
                  block_map_[block_index] = block_ptr;
                  */
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
          }
          break;
        default:
          LOG(FATAL) << "Unknown BlockMergingStrategy: "
                     << static_cast<int>(strategy);
          return false;
      }
    }
    return true;
  } else {
    LOG(ERROR)
        << "The blocks from this protobuf are not compatible with this layer!";
    return false;
  }
}

template <typename VoxelType>
bool Layer<VoxelType>::isCompatible(const LayerProto& layer_proto) const {
  bool compatible = true;
  compatible &= (layer_proto.voxel_size() == voxel_size_);
  compatible &= (layer_proto.voxels_per_side() == voxels_per_side_);
  compatible &= (layer_proto.type() == static_cast<int32_t>(getType()));
  return compatible;
}

template <typename VoxelType>
bool Layer<VoxelType>::readProtoLayerFromFile(const std::string& file_path,
                                              LayerProto* proto_layer) const {
  CHECK_NOTNULL(proto_layer);
  std::ifstream layer_file;
  layer_file.open(file_path, std::fstream::in);
  if (!layer_file.is_open()) {
    LOG(ERROR) << "Couldn't open " << file_path << " for reading.";
    return false;
  }

  google::protobuf::io::IstreamInputStream zero_copy_input(&layer_file);
  google::protobuf::io::CodedInputStream decoder(&zero_copy_input);
  decoder.SetTotalBytesLimit(kMaxLayerSizeInBytes, -1);
  const bool success = proto_layer->ParseFromCodedStream(&decoder) &&
                       decoder.ConsumedEntireMessage() && layer_file.eof();
  layer_file.close();
  if (!success) {
    LOG(ERROR) << "Failed to parse Layer from file: " << file_path;
  }
  return success;
}

template <typename VoxelType>
size_t Layer<VoxelType>::getMemorySize() const {
  size_t size = 0u;

  // Calculate size of members
  size += sizeof(voxel_size_);
  size += sizeof(voxels_per_side_);
  size += sizeof(block_size_);
  size += sizeof(block_size_inv_);

  // Calculate size of the hash mab.
  for (unsigned i = 0; i < block_map_.bucket_count(); ++i) {
    size_t bucket_size = block_map_.bucket_size(i);
    if (bucket_size == 0) {
      size++;
    } else {
      size += bucket_size;
    }
  }

  // Calculate size of blocks
  size_t num_blocks = getNumberOfAllocatedBlocks();
  if (num_blocks > 0u) {
    typename Block<VoxelType>::Ptr block = block_map_.begin()->second;
    size += num_blocks * block->getMemorySize();
  }
  return size;
}

}  // namespace voxblox

#endif  // VOXBLOX_CORE_LAYER_INL_H_

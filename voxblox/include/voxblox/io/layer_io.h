#ifndef VOXBLOX_CORE_IO_LAYER_IO_H_
#define VOXBLOX_CORE_IO_LAYER_IO_H_

#include <string>

#include <glog/logging.h>

#include "./Block.pb.h"
#include "./Layer.pb.h"
#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/utils/protobuf_utils.h"

namespace voxblox {
namespace io {

template <typename VoxelType>
bool LoadLayer(const std::string& file_path,
               typename Layer<VoxelType>::Ptr* layer_ptr) {
  CHECK_NOTNULL(layer_ptr);
  CHECK(!file_path.empty());

  // Open and check the file
  std::fstream proto_file;
  proto_file.open(file_path, std::fstream::in);
  if (!proto_file.is_open()) {
    LOG(ERROR) << "Could not open protobuf file to load layer: " << file_path;
    return false;
  }

  // Unused byte offset result.
  uint32_t tmp_byte_offset;

  // Get number of messages
  uint32_t num_protos;
  if (!utils::readProtoMsgCountToStream(&proto_file, &num_protos,
                                        &tmp_byte_offset)) {
    LOG(ERROR) << "Could not read number of messages.";
    return false;
  }

  if (num_protos == 0u) {
    LOG(WARNING) << "Empty protobuf file!";
    return false;
  }

  // Get header and create the layer if compatible
  LayerProto layer_proto;
  if (!utils::readProtoMsgFromStream(&proto_file, &layer_proto,
                                     &tmp_byte_offset)) {
    LOG(ERROR) << "Could not read layer protobuf message.";
    return false;
  }

  if ((layer_proto.voxel_size()      <= 0) ||
      (layer_proto.voxels_per_side() <= 0) ||
      (getVoxelType<VoxelType>().compare(layer_proto.type()))) {
    LOG(ERROR) << "Invalid parameter in layer protobuf message. Check the format.";
    return false;
  }

  *layer_ptr = aligned_shared<Layer<VoxelType> >(layer_proto);
  CHECK(*layer_ptr);

  // Read all blocks and add them to the layer.
  const size_t num_blocks = num_protos - 1;
  for (uint32_t block_idx = 0u; block_idx < num_blocks; ++block_idx) {
    BlockProto block_proto;
    if (!utils::readProtoMsgFromStream(&proto_file, &block_proto,
                                       &tmp_byte_offset)) {
      LOG(ERROR) << "Could not read block protobuf message number "
                 << block_idx;
      return false;
    }

    if (!(*layer_ptr)
             ->addBlockFromProto(
                 block_proto,
                 Layer<VoxelType>::BlockMergingStrategy::kProhibit)) {
      LOG(ERROR) << "Could not add the block protobuf message to the layer!";
      return false;
    }
  }

  return true;
}

template <typename VoxelType>
bool LoadBlocksFromFile(
    const std::string& file_path,
    typename Layer<VoxelType>::BlockMergingStrategy strategy,
    Layer<VoxelType>* layer_ptr) {
  CHECK_NOTNULL(layer_ptr);
  CHECK(!file_path.empty());

  // Open and check the file
  std::fstream proto_file;
  proto_file.open(file_path, std::fstream::in);
  if (!proto_file.is_open()) {
    LOG(ERROR) << "Could not open protobuf file to load layer: " << file_path;
    return false;
  }

  // Unused byte offset result.
  uint32_t tmp_byte_offset;

  // Get number of messages
  uint32_t num_protos;
  if (!utils::readProtoMsgCountToStream(&proto_file, &num_protos,
                                        &tmp_byte_offset)) {
    LOG(ERROR) << "Could not read number of messages.";
    return false;
  }

  if (num_protos == 0u) {
    LOG(WARNING) << "Empty protobuf file!";
    return false;
  }

  // Get header and check if it is compatible with existing layer.
  LayerProto layer_proto;
  if (!utils::readProtoMsgFromStream(&proto_file, &layer_proto,
                                     &tmp_byte_offset)) {
    LOG(ERROR) << "Could not read layer protobuf message.";
    return false;
  }
  if (!layer_ptr->isCompatible(layer_proto)) {
    LOG(ERROR) << "The layer information read from file is not compatible with "
                  "the current layer!";
    return false;
  }

  // Read all blocks and add them to the layer.
  const size_t num_blocks = num_protos - 1;
  for (uint32_t block_idx = 0u; block_idx < num_blocks; ++block_idx) {
    BlockProto block_proto;
    if (!utils::readProtoMsgFromStream(&proto_file, &block_proto,
                                       &tmp_byte_offset)) {
      LOG(ERROR) << "Could not read block protobuf message number "
                 << block_idx;
      return false;
    }

    if (!layer_ptr->addBlockFromProto(block_proto, strategy)) {
      LOG(ERROR) << "Could not add the block protobuf message to the layer!";
      return false;
    }
  }
  return true;
}

/*TODO(mereweth@jpl.nasa.gov) - why doesn't this compile when template is
 * specialized?
 */
// NOTE(mereweth@jpl.nasa.gov) - for convenience with Python bindings
template <typename VoxelType>
Layer<VoxelType> LoadLayer(const std::string& file_path) {
  if (file_path.empty()) {
    throw std::runtime_error(std::string("Empty file path: ")
                             + file_path);
  }

  typename Layer<VoxelType>::Ptr layer_ptr;

  if (!LoadLayer<VoxelType>(file_path, &layer_ptr)) {
    // TODO(mereweth@jpl.nasa.gov) - throw std exception for Python to catch?
    throw std::runtime_error(std::string("Could not load layer from: ")
                             + file_path);
  }

  CHECK(layer_ptr);

  return *layer_ptr;
}

template <typename VoxelType>
bool SaveLayer(const Layer<VoxelType>& layer, const std::string& file_path) {
  CHECK(!file_path.empty());
  return layer.saveToFile(file_path);
}

template <typename VoxelType>
bool SaveLayerSubset(const Layer<VoxelType>& layer,
                     const std::string& file_path,
                     BlockIndexList blocks_to_include,
                     bool include_all_blocks) {
  CHECK(!file_path.empty());
  return layer.saveSubsetToFile(file_path, blocks_to_include,
                                include_all_blocks);
}

}  // namespace io

}  // namespace voxblox

#endif  // VOXBLOX_CORE_IO_LAYER_IO_H_

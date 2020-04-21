#ifndef VOXBLOX_IO_LAYER_IO_INL_H_
#define VOXBLOX_IO_LAYER_IO_INL_H_

#include <fstream>
#include <string>

#include "voxblox/Block.pb.h"
#include "voxblox/Layer.pb.h"
#include "voxblox/utils/protobuf_utils.h"

namespace voxblox {
namespace io {

template <typename VoxelType>
bool LoadBlocksFromFile(
    const std::string& file_path,
    typename Layer<VoxelType>::BlockMergingStrategy strategy,
    bool multiple_layer_support, Layer<VoxelType>* layer_ptr) {
  CHECK_NOTNULL(layer_ptr);
  CHECK(!file_path.empty());

  // Open and check the file
  std::fstream proto_file;
  proto_file.open(file_path, std::fstream::in);
  if (!proto_file.is_open()) {
    LOG(ERROR) << "Could not open protobuf file to load layer: " << file_path;
    return false;
  }

  // Byte offset result, used to keep track where we are in the file if
  // necessary.
  uint64_t tmp_byte_offset = 0;

  bool layer_found = false;

  do {
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

    if (layer_ptr->isCompatible(layer_proto)) {
      layer_found = true;
    } else if (!multiple_layer_support) {
      LOG(ERROR)
          << "The layer information read from file is not compatible with "
             "the current layer!";
      return false;
    } else {
      // Figure out how much offset to jump forward by. This is the number of
      // blocks * the block size... Actually maybe we just read them in? This
      // is probably easiest. Then if there's corrupted blocks, we abort.
      // We just don't add these to the layer.
      const size_t num_blocks = num_protos - 1;
      for (uint32_t block_idx = 0u; block_idx < num_blocks; ++block_idx) {
        BlockProto block_proto;
        if (!utils::readProtoMsgFromStream(&proto_file, &block_proto,
                                           &tmp_byte_offset)) {
          LOG(ERROR) << "Could not read block protobuf message number "
                     << block_idx;
          return false;
        }
      }
      continue;
    }

    // Read all blocks and add them to the layer.
    const size_t num_blocks = num_protos - 1;
    if (!LoadBlocksFromStream(num_blocks, strategy, &proto_file, layer_ptr,
                              &tmp_byte_offset)) {
      return false;
    }
  } while (multiple_layer_support && !layer_found && !proto_file.eof());
  return layer_found;
}

template <typename VoxelType>
bool LoadBlocksFromFile(
    const std::string& file_path,
    typename Layer<VoxelType>::BlockMergingStrategy strategy,
    Layer<VoxelType>* layer_ptr) {
  constexpr bool multiple_layer_support = false;
  return LoadBlocksFromFile(file_path, strategy, multiple_layer_support,
                            layer_ptr);
}

template <typename VoxelType>
bool LoadBlocksFromStream(
    const size_t num_blocks,
    typename Layer<VoxelType>::BlockMergingStrategy strategy,
    std::fstream* proto_file_ptr, Layer<VoxelType>* layer_ptr,
    uint64_t* tmp_byte_offset_ptr) {
  CHECK_NOTNULL(proto_file_ptr);
  CHECK_NOTNULL(layer_ptr);
  CHECK_NOTNULL(tmp_byte_offset_ptr);
  // Read all blocks and add them to the layer.
  for (uint32_t block_idx = 0u; block_idx < num_blocks; ++block_idx) {
    BlockProto block_proto;
    if (!utils::readProtoMsgFromStream(proto_file_ptr, &block_proto,
                                       tmp_byte_offset_ptr)) {
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

template <typename VoxelType>
bool LoadLayer(const std::string& file_path, const bool multiple_layer_support,
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

  // Byte offset result, used to keep track where we are in the file if
  // necessary.
  uint64_t tmp_byte_offset = 0;

  bool layer_found = false;

  do {
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

    if ((layer_proto.voxel_size() <= 0.0f) ||
        (layer_proto.voxels_per_side() == 0u)) {
      LOG(ERROR)
          << "Invalid parameter in layer protobuf message. Check the format.";
      return false;
    }

    if (getVoxelType<VoxelType>().compare(layer_proto.type()) == 0) {
      layer_found = true;
    } else if (!multiple_layer_support) {
      LOG(ERROR)
          << "The layer information read from file is not compatible with "
             "the current layer!";
      return false;
    } else {
      // Figure out how much offset to jump forward by. This is the number of
      // blocks * the block size... Actually maybe we just read them in? This
      // is probably easiest. Then if there's corrupted blocks, we abort.
      // We just don't add these to the layer.
      const size_t num_blocks = num_protos - 1;
      for (uint32_t block_idx = 0u; block_idx < num_blocks; ++block_idx) {
        BlockProto block_proto;
        if (!utils::readProtoMsgFromStream(&proto_file, &block_proto,
                                           &tmp_byte_offset)) {
          LOG(ERROR) << "Could not read block protobuf message number "
                     << block_idx;
          return false;
        }
      }
      continue;
    }

    *layer_ptr = aligned_shared<Layer<VoxelType> >(layer_proto);
    CHECK(*layer_ptr);

    // Read all blocks and add them to the layer.
    const size_t num_blocks = num_protos - 1;
    if (!LoadBlocksFromStream(
            num_blocks, Layer<VoxelType>::BlockMergingStrategy::kProhibit,
            &proto_file, (*layer_ptr).get(), &tmp_byte_offset)) {
      return false;
    }
  } while (multiple_layer_support && !layer_found && !proto_file.eof());
  return layer_found;
}

// By default loads without multiple layer support (i.e., only checks the first
// layer in the file).
template <typename VoxelType>
bool LoadLayer(const std::string& file_path,
               typename Layer<VoxelType>::Ptr* layer_ptr) {
  constexpr bool multiple_layer_support = false;
  return LoadLayer<VoxelType>(file_path, multiple_layer_support, layer_ptr);
}

template <typename VoxelType>
bool SaveLayer(const Layer<VoxelType>& layer, const std::string& file_path,
               bool clear_file) {
  CHECK(!file_path.empty());
  return layer.saveToFile(file_path, clear_file);
}

template <typename VoxelType>
bool SaveLayerSubset(const Layer<VoxelType>& layer,
                     const std::string& file_path,
                     const BlockIndexList& blocks_to_include,
                     bool include_all_blocks) {
  CHECK(!file_path.empty());
  return layer.saveSubsetToFile(file_path, blocks_to_include,
                                include_all_blocks);
}

}  // namespace io
}  // namespace voxblox

#endif  // VOXBLOX_IO_LAYER_IO_INL_H_

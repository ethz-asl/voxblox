#ifndef VOXBLOX_TANGO_INTERFACE_IO_TANGO_LAYER_IO_H_
#define VOXBLOX_TANGO_INTERFACE_IO_TANGO_LAYER_IO_H_

#include <string>

#include <glog/logging.h>

#include "./Volume.pb.h"
#include "./MapHeader.pb.h"
#include "voxblox/core/common.h"
#include "voxblox/utils/protobuf_utils.h"

#include "voxblox_tango_interface/core/tango_layer_interface.h"

namespace voxblox {
namespace io {

/* NOTE(mereweth@jpl.nasa.gov) - when adding functions here, inline them
 * or split into hpp/cpp
 */

/* TODO(mereweth@jpl.nasa.gov) - if we load into NTSDF voxel later, what to call
 * those functions?
 */

inline bool TangoLoadLayer(const std::string& file_path,
                           TangoLayerInterface::Ptr* layer_ptr) {
  CHECK_NOTNULL(layer_ptr);

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
  tsdf2::MapHeaderProto layer_proto;
  if (!utils::readProtoMsgFromStream(&proto_file, &layer_proto,
                                     &tmp_byte_offset)) {
    LOG(ERROR) << "Could not read layer protobuf message.";
    return false;
  }
  *layer_ptr = aligned_shared<TangoLayerInterface >(layer_proto);
  CHECK(*layer_ptr);

  // Read all blocks and add them to the layer.
  const size_t num_blocks = num_protos - 1;
  for (uint32_t block_idx = 0u; block_idx < num_blocks; ++block_idx) {
    tsdf2::VolumeProto block_proto;
    if (!utils::readProtoMsgFromStream(&proto_file, &block_proto,
                                       &tmp_byte_offset)) {
      LOG(ERROR) << "Could not read block protobuf message number "
                 << block_idx;
      return false;
    }

    if (!(*layer_ptr)->addBlockFromProto(
             block_proto, Layer<TsdfVoxel>::BlockMergingStrategy::kProhibit)) {
      LOG(ERROR) << "Could not add the block protobuf message to the layer!";
      return false;
    }
  }

  return true;
}

inline bool TangoLoadBlocksFromFile(
    const std::string& file_path,
    Layer<TsdfVoxel>::BlockMergingStrategy strategy,
    TangoLayerInterface* layer_ptr) {
  CHECK_NOTNULL(layer_ptr);

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
  tsdf2::MapHeaderProto layer_proto;
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
    tsdf2::VolumeProto block_proto;
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

/*TODO(mereweth@jpl.nasa.gov) - best name for this function?
 * This function may be useful when debugging a malformed protobuf dump
 * Also prevents segfaulting Python using file_path TangoLayerInterface constructor
 */
inline TangoLayerInterface::Ptr TangoLoadOrCreateLayerHeader(
                                                const std::string& file_path,
                                                FloatingPoint voxel_size,
                                                size_t voxels_per_side,
                                                unsigned int max_ntsdf_voxel_weight,
                                                FloatingPoint meters_to_ntsdf) {

  bool success = true;

  // Open and check the file
  std::fstream proto_file;
  proto_file.open(file_path, std::fstream::in);
  if (!success ||
      !proto_file.is_open()) {
    LOG(ERROR) << "Could not open protobuf file to load layer: " << file_path;
    success = false;
  }

  // Unused byte offset result.
  uint32_t tmp_byte_offset;

  // Get number of messages
  uint32_t num_protos;
  if (!success ||
      !utils::readProtoMsgCountToStream(&proto_file, &num_protos,
                                        &tmp_byte_offset)) {
    LOG(ERROR) << "Could not read number of messages.";
    success = false;
  }

  if (!success ||
      (num_protos == 0u)) {
    LOG(WARNING) << "Empty protobuf file!";
    success = false;
  }

  // Get header and create the layer if compatible
  tsdf2::MapHeaderProto layer_proto;
  if (!success ||
      !utils::readProtoMsgFromStream(&proto_file, &layer_proto,
                                     &tmp_byte_offset)) {
    LOG(ERROR) << "Could not read layer protobuf message.";
    success = false;
  }

  TangoLayerInterface::Ptr layer_ptr;
  if (success) {
    layer_ptr = aligned_shared<TangoLayerInterface>(layer_proto);
  }
  else {
    layer_ptr = std::make_shared<TangoLayerInterface>(voxel_size,
                                                      voxels_per_side,
                                                      max_ntsdf_voxel_weight,
                                                      meters_to_ntsdf);
  }
  CHECK(layer_ptr);

  return layer_ptr;
}

}  // namespace io

}  // namespace voxblox

#endif  // VOXBLOX_TANGO_INTERFACE_IO_TANGO_LAYER_IO_H_

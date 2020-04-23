#ifndef VOXBLOX_IO_LAYER_IO_H_
#define VOXBLOX_IO_LAYER_IO_H_

#include <string>

#include <glog/logging.h>

#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"

namespace voxblox {
namespace io {

/**
 * By default, loads blocks without multiple layer support.
 * Loading blocks assumes that the layer is already setup and allocated.
 */
template <typename VoxelType>
bool LoadBlocksFromFile(
    const std::string& file_path,
    typename Layer<VoxelType>::BlockMergingStrategy strategy,
    Layer<VoxelType>* layer_ptr);

/**
 * By default, loads blocks without multiple layer support.
 * Loading the full layer allocates a layer of the correct size.
 */
template <typename VoxelType>
bool LoadBlocksFromFile(
    const std::string& file_path,
    typename Layer<VoxelType>::BlockMergingStrategy strategy,
    bool multiple_layer_support, Layer<VoxelType>* layer_ptr);

template <typename VoxelType>
bool LoadBlocksFromStream(
    const size_t num_blocks,
    typename Layer<VoxelType>::BlockMergingStrategy strategy,
    std::fstream* proto_file_ptr, Layer<VoxelType>* layer_ptr,
    uint64_t* tmp_byte_offset_ptr);

/**
 * Unlike LoadBlocks above, this actually allocates the layer as well.
 * By default loads without multiple layer support (i.e., only checks the first
 * layer in the file).
 */
template <typename VoxelType>
bool LoadLayer(const std::string& file_path,
               typename Layer<VoxelType>::Ptr* layer_ptr);

/**
 * Unlike LoadBlocks, this actually allocates the layer as well.
 * By default loads without multiple layer support (i.e., only checks the first
 * layer in the file).
 */
template <typename VoxelType>
bool LoadLayer(const std::string& file_path, const bool multiple_layer_support,
               typename Layer<VoxelType>::Ptr* layer_ptr);

/**
 * By default, clears (truncates) the output file. Set clear_file to false in
 * case writing the second (or subsequent) layer into the same file.
 */
template <typename VoxelType>
bool SaveLayer(const Layer<VoxelType>& layer, const std::string& file_path,
               bool clear_file = true);

/// Saves only some parts of the layer to the file. Clears the file by default.
template <typename VoxelType>
bool SaveLayerSubset(const Layer<VoxelType>& layer,
                     const std::string& file_path,
                     const BlockIndexList& blocks_to_include,
                     bool include_all_blocks);

}  // namespace io
}  // namespace voxblox

#include "voxblox/io/layer_io_inl.h"

#endif  // VOXBLOX_IO_LAYER_IO_H_

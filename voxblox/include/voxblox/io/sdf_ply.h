#ifndef VOXBLOX_CORE_IO_SDF_PLY_H_
#define VOXBLOX_CORE_IO_SDF_PLY_H_

#include <algorithm>
#include <string>

#include "voxblox/io/ply_writer.h"
#include "voxblox/core/map.h"

namespace voxblox {

namespace io {

enum PlyOutputTypes {
  // The full SDF colorized by the distance in each voxel.
  kSdfDistanceColor = 0,
  // Isosurface colorized by ???
  kSdfIsosurface
};

template <typename MapType>
bool outputMapAsPly(const MapType& map, const std::string& filename,
                    PlyOutputTypes type) {
  return false;
}

template <>
bool outputMapAsPly<TsdfMap>(const TsdfMap& map, const std::string& filename,
                             PlyOutputTypes type) {
  // Create a PlyWriter.
  PlyWriter writer(filename);

  if (type == kSdfDistanceColor) {
    // In this case, we get all the allocated voxels and color them based on
    // distance value.
    size_t num_blocks = map.getNumberOfAllocatedBlocks();
    // This function is block-specific:
    size_t num_voxels_per_block = map.getTsdfVoxelsPerBlock();
    size_t vps = map.getTsdfVoxelsPerSide();

    // Maybe this isn't strictly true, since actually we may have stuff with 0
    // weight...
    size_t total_voxels = num_blocks * num_voxels_per_block;
    const bool has_color = true;
    writer.addVerticesWithProperties(total_voxels, has_color);
    if (!writer.writeHeader()) {
      return false;
    }

    BlockIndexList blocks;
    map.getAllAllocatedBlocks(&blocks);

    int observed_voxels = 0;

    // Iterate over all blocks.
    const float max_distance = 20;
    for (const BlockIndex& index : blocks) {
      // Iterate over all voxels in said blocks.
      const TsdfBlock& block = map.getBlockByIndex(index);

      VoxelIndex voxel_index = VoxelIndex::Zero();
      for (voxel_index.x() = 0; voxel_index.x() < vps; ++voxel_index.x()) {
        for (voxel_index.y() = 0; voxel_index.y() < vps; ++voxel_index.y()) {
          for (voxel_index.z() = 0; voxel_index.z() < vps; ++voxel_index.z()) {
            const TsdfVoxel& voxel =
                block.getTsdfVoxelByVoxelIndex(voxel_index);

            float distance = voxel.distance;
            float weight = voxel.weight;

            // Get back the original coordinate of this voxel.
            Point coord =
                block.getCoordinatesOfTsdfVoxelByVoxelIndex(voxel_index);

            // Decide how to color this.
            // Distance > 0 = blue, distance < 0 = red.
            Color distance_color = Color::blendTwoColors(
                Color(255, 0, 0, 0),
                std::max<float>(1 - distance / max_distance, 0.0),
                Color(0, 0, 255, 0),
                std::max<float>(1 + distance / max_distance, 0.0));
            uint8_t color[3];
            if (weight > 0) {
              color[0] = distance_color.r;
              color[1] = distance_color.g;
              color[2] = distance_color.b;
              observed_voxels++;
            } else {
              color[0] = 0;
              color[1] = 0;
              color[2] = 0;
            }
            writer.writeVertex(coord, color);
          }
        }
      }
    }
    LOG(INFO) << "Number of observed voxels: " << observed_voxels;
    writer.closeFile();
    return true;
  }
  return false;
}

}  // namespace io

}  // namespace voxblox

#endif  // VOXBLOX_CORE_IO_SDF_PLY_H_

#ifndef VOXBLOX_CORE_IO_SDF_PLY_H_
#define VOXBLOX_CORE_IO_SDF_PLY_H_

#include <algorithm>
#include <string>

#include "voxblox/core/layer.h"
#include "voxblox/io/mesh_ply.h"
#include "voxblox/io/ply_writer.h"
#include "voxblox/mesh/mesh.h"
#include "voxblox/mesh/mesh_integrator.h"
#include "voxblox/mesh/mesh_layer.h"

namespace voxblox {

namespace io {

enum PlyOutputTypes {
  // The full SDF colorized by the distance in each voxel.
  kSdfColoredDistanceField,
  // Output isosurface, i.e. the mesh for sdf voxel types.
  kSdfIsosurface
};

template <typename VoxelType>
bool getColorFromVoxel(const VoxelType& voxel, const FloatingPoint max_distance,
                       Color* color);

template <>
bool getColorFromVoxel(const TsdfVoxel& voxel, const FloatingPoint max_distance,
                       Color* color) {
  CHECK_NOTNULL(color);

  static constexpr float kTolerance = 1e-6;
  if (voxel.weight <= kTolerance) {
    return false;
  }

  // Decide how to color this.
  // Distance > 0 = blue, distance < 0 = red.
  Color distance_color = Color::blendTwoColors(
      Color(255, 0, 0, 0),
      std::max<float>(1 - voxel.distance / max_distance, 0.0),
      Color(0, 0, 255, 0),
      std::max<float>(1 + voxel.distance / max_distance, 0.0));

  *color = distance_color;
  return true;
}

template <>
bool getColorFromVoxel(const EsdfVoxel& voxel, const FloatingPoint max_distance,
                       Color* color) {
  CHECK_NOTNULL(color);
  if (!voxel.observed) {
    return false;
  }

  // Decide how to color this.
  // Distance > 0 = blue, distance < 0 = red.
  Color distance_color = Color::blendTwoColors(
      Color(255, 0, 0, 0),
      std::max<float>(1 - voxel.distance / max_distance, 0.0),
      Color(0, 0, 255, 0),
      std::max<float>(1 + voxel.distance / max_distance, 0.0));

  *color = distance_color;
  return true;
}

template <typename VoxelType>
bool outputLayerAsPly(const Layer<VoxelType>& layer,
                      const std::string& filename, PlyOutputTypes type) {
  // Create a PlyWriter.
  PlyWriter writer(filename);

  switch (type) {
    case PlyOutputTypes::kSdfColoredDistanceField: {
      // In this case, we get all the allocated voxels and color them based on
      // distance value.
      size_t num_blocks = layer.getNumberOfAllocatedBlocks();
      // This function is block-specific:
      size_t vps = layer.voxels_per_side();
      size_t num_voxels_per_block = vps * vps * vps;

      // Maybe this isn't strictly true, since actually we may have stuff with 0
      // weight...
      size_t total_voxels = num_blocks * num_voxels_per_block;
      const bool has_color = true;
      writer.addVerticesWithProperties(total_voxels, has_color);
      if (!writer.writeHeader()) {
        return false;
      }

      BlockIndexList blocks;
      layer.getAllAllocatedBlocks(&blocks);

      int observed_voxels = 0;

      // Iterate over all blocks.
      const float max_distance = 20;
      for (const BlockIndex& index : blocks) {
        // Iterate over all voxels in said blocks.
        const Block<VoxelType>& block = layer.getBlockByIndex(index);

        VoxelIndex voxel_index = VoxelIndex::Zero();
        for (voxel_index.x() = 0; voxel_index.x() < vps; ++voxel_index.x()) {
          for (voxel_index.y() = 0; voxel_index.y() < vps; ++voxel_index.y()) {
            for (voxel_index.z() = 0; voxel_index.z() < vps;
                 ++voxel_index.z()) {
              const VoxelType& voxel = block.getVoxelByVoxelIndex(voxel_index);

              // Get back the original coordinate of this voxel.
              Point coord = block.computeCoordinatesFromVoxelIndex(voxel_index);

              Color color;
              if (getColorFromVoxel(voxel, max_distance, &color)) {
                ++observed_voxels;
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
    case PlyOutputTypes::kSdfIsosurface: {
      typename MeshIntegrator<VoxelType>::Config mesh_config;
      MeshLayer::Ptr mesh(new MeshLayer(layer.block_size()));
      MeshIntegrator<VoxelType> mesh_integrator(mesh_config, &layer,
                                                mesh.get());

      constexpr bool only_mesh_updated_blocks = false;
      constexpr bool clear_updated_flag = false;
      mesh_integrator.generateMesh(only_mesh_updated_blocks,
                                   clear_updated_flag);

      return outputMeshLayerAsPly(filename, *mesh);
    }
    default:
      LOG(FATAL) << "Unknown layer to ply output type: "
                 << static_cast<int>(type);
  }
  return false;
}

}  // namespace io

}  // namespace voxblox

#endif  // VOXBLOX_CORE_IO_SDF_PLY_H_

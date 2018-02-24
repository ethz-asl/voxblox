#ifndef VOXBLOX_IO_SDF_PLY_H_
#define VOXBLOX_IO_SDF_PLY_H_

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
  kSdfIsosurface,
  // Output isosurface, i.e. the mesh for sdf voxel types.
  // Close vertices are connected and zero surface faces are removed.
  kSdfIsosurfaceConnected
};

template <typename VoxelType>
bool getColorFromVoxel(const VoxelType& voxel, const FloatingPoint max_distance,
                       Color* color);

template <>
bool getColorFromVoxel(const TsdfVoxel& voxel, const FloatingPoint max_distance,
                       Color* color);

template <>
bool getColorFromVoxel(const EsdfVoxel& voxel, const FloatingPoint max_distance,
                       Color* color);

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

      BlockIndexList blocks;
      layer.getAllAllocatedBlocks(&blocks);

      int observed_voxels = 0;


      std::vector<std::pair<Point, Color>> points;

      // Iterate over all blocks.
      const float max_distance = layer.voxel_size() * 10;
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

              Color color(255u,255u,255u,0u);
              if (getColorFromVoxel(voxel, max_distance, &color)) {
                ++observed_voxels;
                points.emplace_back(coord, color);
              }
            }
          }
        }
      }

      writer.addVerticesWithProperties(observed_voxels, has_color);
      if (!writer.writeHeader()) {
        return false;
      }

      for(const std::pair<Point, Color>& point : points){
        writer.writeVertex(point.first, point.second);
      }

      LOG(INFO) << "Number of observed voxels: " << observed_voxels;
      writer.closeFile();
      return true;
    }
    case PlyOutputTypes::kSdfIsosurface: {
      typename MeshIntegrator<VoxelType>::Config mesh_config;
      MeshLayer::Ptr mesh(new MeshLayer(layer.block_size()));
      MeshIntegrator<VoxelType> mesh_integrator(mesh_config, layer, mesh.get());

      constexpr bool only_mesh_updated_blocks = false;
      constexpr bool clear_updated_flag = false;
      mesh_integrator.generateMesh(only_mesh_updated_blocks,
                                   clear_updated_flag);
      constexpr bool kConnectedMesh = false;
      return outputMeshLayerAsPly(filename, kConnectedMesh, *mesh);
    }
    case PlyOutputTypes::kSdfIsosurfaceConnected: {
      typename MeshIntegrator<VoxelType>::Config mesh_config;
      MeshLayer::Ptr mesh(new MeshLayer(layer.block_size()));
      MeshIntegrator<VoxelType> mesh_integrator(mesh_config, layer, mesh.get());

      constexpr bool only_mesh_updated_blocks = false;
      constexpr bool clear_updated_flag = false;
      mesh_integrator.generateMesh(only_mesh_updated_blocks,
                                   clear_updated_flag);
      constexpr bool kConnectedMesh = true;
      return outputMeshLayerAsPly(filename, kConnectedMesh, *mesh);
    }

    default:
      LOG(FATAL) << "Unknown layer to ply output type: "
                 << static_cast<int>(type);
  }
  return false;
}

}  // namespace io

}  // namespace voxblox

#endif  // VOXBLOX_IO_SDF_PLY_H_

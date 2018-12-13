#ifndef VOXBLOX_IO_SDF_PLY_H_
#define VOXBLOX_IO_SDF_PLY_H_

#include <algorithm>
#include <string>

#include "voxblox/core/layer.h"
#include "voxblox/io/mesh_ply.h"
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

/**
 * Convert a voxel to a colored point. The sdf_color_range determines the range
 * that is covered by the rainbow colors. All absolute distance values that
 * exceed this range will receive tha max/min color of the rainbow range. The
 * sdf_max_value determines if a point is generated for this value or not. Only
 * SDF values within this max value result in a colored point.
 */
template <typename VoxelType>
bool getColorFromVoxel(const VoxelType& voxel, const float sdf_color_range,
                       const float sdf_max_value, Color* color);

/**
 * Convert a voxel to a colored point. The sdf_color_range determines the range
 * that is covered by the rainbow colors. All absolute distance values that
 * exceed this range will receive tha max/min color of the rainbow range. The
 * sdf_max_value determines if a point is generated for this value or not. Only
 * SDF values within this max value result in a colored point.
 */
template <>
bool getColorFromVoxel(const TsdfVoxel& voxel, const float sdf_color_range,
                       const float sdf_max_value, Color* color);

/**
 * Convert a voxel to a colored point. The sdf_color_range determines the range
 * that is covered by the rainbow colors. All absolute distance values that
 * exceed this range will receive tha max/min color of the rainbow range. The
 * sdf_max_value determines if a point is generated for this value or not. Only
 * SDF values within this max value result in a colored point.
 */
template <>
bool getColorFromVoxel(const EsdfVoxel& voxel, const float sdf_color_range,
                       const float sdf_max_value, Color* color);

/**
 * This function converts all voxels with positive weight/observed into points
 * colored by a color map based on the SDF value. The parameter sdf_color_range
 * is used to determine the range of the rainbow color map which is used to
 * visualize the SDF values. If an SDF value is outside this range, it will be
 * truncated to the limits of the range. sdf_max_value determines if a point is
 * generated for this value or not. Only SDF values within this max value result
 * in a colored point. If this threshold is set to a negative value, all points
 * will be generated independent of the SDF value.
 */
template <typename VoxelType>
bool convertVoxelGridToPointCloud(const Layer<VoxelType>& layer,
                                  const float sdf_color_range,
                                  const float sdf_max_value,
                                  voxblox::Mesh* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  CHECK_GT(sdf_color_range, 0.0f);

  BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  // Iterate over all blocks.
  for (const BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const Block<VoxelType>& block = layer.getBlockByIndex(index);

    const int vps = block.voxels_per_side();

    VoxelIndex voxel_index = VoxelIndex::Zero();
    for (voxel_index.x() = 0; voxel_index.x() < vps; ++voxel_index.x()) {
      for (voxel_index.y() = 0; voxel_index.y() < vps; ++voxel_index.y()) {
        for (voxel_index.z() = 0; voxel_index.z() < vps; ++voxel_index.z()) {
          const VoxelType& voxel = block.getVoxelByVoxelIndex(voxel_index);

          // Get back the original coordinate of this voxel.
          const Point coord =
              block.computeCoordinatesFromVoxelIndex(voxel_index);

          Color color;
          if (getColorFromVoxel(voxel, sdf_color_range, sdf_max_value,
                                &color)) {
            point_cloud->vertices.push_back(coord);
            point_cloud->colors.push_back(color);
          }
        }
      }
    }
  }
  return point_cloud->size() > 0u;
}

template <typename VoxelType>
bool convertVoxelGridToPointCloud(const Layer<VoxelType>& layer,
                                  const float sdf_color_range,
                                  voxblox::Mesh* point_cloud) {
  constexpr float kInvalidSdfMaxValue = -1.0f;
  return convertVoxelGridToPointCloud<VoxelType>(
      layer, sdf_color_range, kInvalidSdfMaxValue, point_cloud);
}

/**
 * Converts the layer to a mesh by extracting its ISO surface using marching
 * cubes. This function returns false if the mesh is empty. The mesh can either
 * be extracted as a set of distinct triangles, or the function can try to
 * connect all identical vertices to create a connected mesh.
 */
template <typename VoxelType>
bool convertLayerToMesh(
    const Layer<VoxelType>& layer, const MeshIntegratorConfig& mesh_config,
    voxblox::Mesh* mesh, const bool connected_mesh = true,
    const FloatingPoint vertex_proximity_threshold = 1e-10) {
  CHECK_NOTNULL(mesh);

  MeshLayer mesh_layer(layer.block_size());
  MeshIntegrator<VoxelType> mesh_integrator(mesh_config, layer, &mesh_layer);

  // Generate mesh layer.
  constexpr bool only_mesh_updated_blocks = false;
  constexpr bool clear_updated_flag = false;
  mesh_integrator.generateMesh(only_mesh_updated_blocks, clear_updated_flag);

  // Extract mesh from mesh layer, either by simply concatenating all meshes
  // (there is one per block) or by connecting them.
  if (connected_mesh) {
    mesh_layer.getConnectedMesh(mesh, vertex_proximity_threshold);
  } else {
    mesh_layer.getMesh(mesh);
  }
  return mesh->size() > 0u;
}
template <typename VoxelType>
bool convertLayerToMesh(
    const Layer<VoxelType>& layer, voxblox::Mesh* mesh,
    const bool connected_mesh = true,
    const FloatingPoint vertex_proximity_threshold = 1e-10) {
  MeshIntegratorConfig mesh_config;
  return convertLayerToMesh(layer, mesh_config, mesh, connected_mesh,
                            vertex_proximity_threshold);
}

/**
 * Output the layer to ply file. Depending on the ply output type, this either
 * exports all voxel centers colored by th SDF values or extracts the ISO
 * surface as mesh. The parameter sdf_color_range is used to color the points
 * for modes that use an SDF-based point cloud coloring function.
 */
template <typename VoxelType>
bool outputLayerAsPly(const Layer<VoxelType>& layer,
                      const std::string& filename, PlyOutputTypes type,
                      const float sdf_color_range = 0.3f,
                      const float max_sdf_value_to_output = 0.3f) {
  CHECK(!filename.empty());
  switch (type) {
    case PlyOutputTypes::kSdfColoredDistanceField: {
      voxblox::Mesh point_cloud;
      if (!convertVoxelGridToPointCloud(
              layer, sdf_color_range, max_sdf_value_to_output, &point_cloud)) {
        return false;
      }

      return outputMeshAsPly(filename, point_cloud);
    }
    case PlyOutputTypes::kSdfIsosurface: {
      constexpr bool kConnectedMesh = false;

      voxblox::Mesh mesh;
      if (!convertLayerToMesh(layer, &mesh, kConnectedMesh)) {
        return false;
      }
      return outputMeshAsPly(filename, mesh);
    }
    case PlyOutputTypes::kSdfIsosurfaceConnected: {
      constexpr bool kConnectedMesh = true;

      voxblox::Mesh mesh;
      if (!convertLayerToMesh(layer, &mesh, kConnectedMesh)) {
        return false;
      }
      return outputMeshAsPly(filename, mesh);
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

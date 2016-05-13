#ifndef VOXBLOX_MESH_MESH_INTEGRATOR_H_
#define VOXBLOX_MESH_MESH_INTEGRATOR_H_

#include <algorithm>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/mesh/marching_cubes.h"
#include "voxblox/mesh/mesh_layer.h"
#include "voxblox/utils/timing.h"

namespace voxblox {

class MeshIntegrator {
 public:
  MeshIntegrator(Layer<TsdfVoxel>* tsdf_layer, MeshLayer* mesh_layer)
      : tsdf_layer_(tsdf_layer), mesh_layer_(mesh_layer) {
    DCHECK_NOTNULL(tsdf_layer_);
    DCHECK_NOTNULL(mesh_layer_);

    voxel_size_ = tsdf_layer_->voxel_size();
    block_size_ = tsdf_layer_->block_size();
    voxels_per_side_ = tsdf_layer_->voxels_per_side();

    voxel_size_inv_ = 1.0 / voxel_size_;
    block_size_inv_ = 1.0 / block_size_;
    voxels_per_side_inv_ = 1.0 / voxels_per_side_;

    cube_index_offsets_ << 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0,
        0, 0, 1, 1, 1, 1;
  }

  // Generates mesh for the entire tsdf layer from scratch.
  void generateWholeMesh() { mesh_layer_->clear(); }

  void generateMeshForUpdatedBlocks() {
    // Only update parts of the mesh for blocks that have updated.
  }

  void updateMeshForBlock(const BlockIndex& block_index) {
    Mesh::Ptr mesh = mesh_layer_->allocateMeshPtrByIndex(block_index);
    // This block should already exist, otherwise it makes no sense to update
    // the mesh for it. ;)
    Block<TsdfVoxel>::ConstPtr block =
        tsdf_layer_->getBlockPtrByIndex(block_index);

    size_t vps = block->voxels_per_side();
    VertexIndex next_mesh_index = 0;

    VoxelIndex voxel_index;
    for (voxel_index.x() = 1; voxel_index.x() < vps - 1; ++voxel_index.x()) {
      for (voxel_index.y() = 1; voxel_index.y() < vps - 1; ++voxel_index.y()) {
        for (voxel_index.z() = 1; voxel_index.z() < vps - 1;
             ++voxel_index.z()) {
          Point coords = block->computeCoordinatesFromVoxelIndex(voxel_index);
          extractMeshInsideBlock(block, voxel_index, coords, &next_mesh_index,
                                 mesh.get());
        }
      }
    }
  }

  void extractMeshInsideBlock(const Block<TsdfVoxel>::ConstPtr& block,
                              const VoxelIndex& index, const Point& coords,
                              VertexIndex* next_mesh_index, Mesh* mesh) {
    DCHECK_NOTNULL(mesh);
    Eigen::Matrix<FloatingPoint, 3, 8> cube_coord_offsets =
        cube_index_offsets_.cast<FloatingPoint>() * voxel_size_;
    Eigen::Matrix<FloatingPoint, 3, 8> corner_coords;
    Eigen::Matrix<FloatingPoint, 8, 1> corner_sdf;
    bool all_neighbors_observed = true;

    for (int i = 0; i < 8; ++i) {
      VoxelIndex corner_index = index + cube_index_offsets_.col(i);
      const TsdfVoxel& voxel = block->getVoxelByVoxelIndex(corner_index);

      // Do not extract a mesh here if one of the corner is unobserved and
      // outside the truncation region.
      if (voxel.weight <= 1e-6) {
        all_neighbors_observed = false;
        break;
      }
      corner_coords.col(i) = coords + cube_coord_offsets.col(i);
      corner_sdf(i) = voxel.distance;
    }

    if (all_neighbors_observed) {
      MarchingCubes::meshCube(corner_coords, corner_sdf, next_mesh_index, mesh);
    }
  }

 protected:
  Layer<TsdfVoxel>* tsdf_layer_;
  MeshLayer* mesh_layer_;

  // Cached map config.
  FloatingPoint voxel_size_;
  size_t voxels_per_side_;
  FloatingPoint block_size_;

  // Derived types.
  FloatingPoint voxel_size_inv_;
  FloatingPoint voxels_per_side_inv_;
  FloatingPoint block_size_inv_;

  // Cached index map.
  Eigen::Matrix<int, 3, 8> cube_index_offsets_;
};

}  // namespace voxblox

#endif  // VOXBLOX_MESH_MESH_INTEGRATOR_H_

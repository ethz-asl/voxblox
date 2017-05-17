// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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
#include "voxblox/interpolator/interpolator.h"

namespace voxblox {

class MeshIntegrator {
 public:
  struct Config {
    bool use_color = true;
    bool compute_normals = true;
    float min_weight = 1e-4;
  };

  MeshIntegrator(const Config& config, Layer<TsdfVoxel>* tsdf_layer,
                 MeshLayer* mesh_layer)
      : config_(config),
        tsdf_layer_(CHECK_NOTNULL(tsdf_layer)),
        mesh_layer_(CHECK_NOTNULL(mesh_layer)) {
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
  void generateWholeMesh() {
    mesh_layer_->clear();
    // Get all of the blocks in the TSDF layer, and mesh each one.
    BlockIndexList all_tsdf_blocks;
    tsdf_layer_->getAllAllocatedBlocks(&all_tsdf_blocks);

    for (const BlockIndex& block_index : all_tsdf_blocks) {
      updateMeshForBlock(block_index);
    }
  }

  void generateMeshForUpdatedBlocks(bool clear_updated_flag) {
    // Only update parts of the mesh for blocks that have updated.
    // clear_updated_flag decides whether to reset 'updated' after updating the
    // mesh.
    BlockIndexList all_tsdf_blocks;
    tsdf_layer_->getAllAllocatedBlocks(&all_tsdf_blocks);

    for (const BlockIndex& block_index : all_tsdf_blocks) {
      Block<TsdfVoxel>::Ptr block =
          tsdf_layer_->getBlockPtrByIndex(block_index);
      if (block->updated()) {
        updateMeshForBlock(block_index);
        if (clear_updated_flag) {
          block->updated() = false;
        }
      }
    }
  }

  void extractBlockMesh(Block<TsdfVoxel>::ConstPtr block, Mesh::Ptr mesh) {
    size_t vps = block->voxels_per_side();
    VertexIndex next_mesh_index = 0;

    VoxelIndex voxel_index;
    for (voxel_index.x() = 0; voxel_index.x() < vps - 1; ++voxel_index.x()) {
      for (voxel_index.y() = 0; voxel_index.y() < vps - 1; ++voxel_index.y()) {
        for (voxel_index.z() = 0; voxel_index.z() < vps - 1;
             ++voxel_index.z()) {
          Point coords = block->computeCoordinatesFromVoxelIndex(voxel_index);
          extractMeshInsideBlock(*block, voxel_index, coords, &next_mesh_index,
                                 mesh.get());
        }
      }
    }

    // Max X plane (takes care of max-Y corner as well).
    voxel_index.x() = vps - 1;
    for (voxel_index.z() = 0; voxel_index.z() < vps - 1; voxel_index.z()++) {
      for (voxel_index.y() = 0; voxel_index.y() < vps; voxel_index.y()++) {
        Point coords = block->computeCoordinatesFromVoxelIndex(voxel_index);
        extractMeshOnBorder(*block, voxel_index, coords, &next_mesh_index,
                            mesh.get());
      }
    }

    // Max Y plane.
    voxel_index.y() = vps - 1;
    for (voxel_index.z() = 0; voxel_index.z() < vps - 1; voxel_index.z()++) {
      for (voxel_index.x() = 0; voxel_index.x() < vps - 1; voxel_index.x()++) {
        Point coords = block->computeCoordinatesFromVoxelIndex(voxel_index);
        extractMeshOnBorder(*block, voxel_index, coords, &next_mesh_index,
                            mesh.get());
      }
    }

    // Max Z plane (also takes care of corners).
    voxel_index.z() = vps - 1;
    for (voxel_index.y() = 0; voxel_index.y() < vps; voxel_index.y()++) {
      for (voxel_index.x() = 0; voxel_index.x() < vps; voxel_index.x()++) {
        Point coords = block->computeCoordinatesFromVoxelIndex(voxel_index);
        extractMeshOnBorder(*block, voxel_index, coords, &next_mesh_index,
                            mesh.get());
      }
    }
  }

  virtual void updateMeshForBlock(const BlockIndex& block_index) {
    Mesh::Ptr mesh = mesh_layer_->allocateMeshPtrByIndex(block_index);
    mesh->clear();
    // This block should already exist, otherwise it makes no sense to update
    // the mesh for it. ;)
    Block<TsdfVoxel>::ConstPtr block =
        tsdf_layer_->getBlockPtrByIndex(block_index);

    if (!block) {
      LOG(ERROR) << "Trying to mesh a non-existent block at index: "
                 << block_index.transpose();
      return;
    }

    extractBlockMesh(block, mesh);

    // Update colors if needed.
    if (config_.use_color) {
      updateMeshColor(*block, mesh.get());
    }

    if (config_.compute_normals) {
      computeMeshNormals(*block, mesh.get());
    }
  }

  void extractMeshInsideBlock(const Block<TsdfVoxel>& block,
                              const VoxelIndex& index, const Point& coords,
                              VertexIndex* next_mesh_index, Mesh* mesh) {
    DCHECK_NOTNULL(next_mesh_index);
    DCHECK_NOTNULL(mesh);
    Eigen::Matrix<FloatingPoint, 3, 8> cube_coord_offsets =
        cube_index_offsets_.cast<FloatingPoint>() * voxel_size_;
    Eigen::Matrix<FloatingPoint, 3, 8> corner_coords;
    Eigen::Matrix<FloatingPoint, 8, 1> corner_sdf;
    bool all_neighbors_observed = true;

    for (unsigned int i = 0; i < 8; ++i) {
      VoxelIndex corner_index = index + cube_index_offsets_.col(i);
      const TsdfVoxel& voxel = block.getVoxelByVoxelIndex(corner_index);

      // Do not extract a mesh here if one of the corner is unobserved and
      // outside the truncation region.
      // TODO(helenol): comment above from open_chisel, but no actual checks
      // on distance are ever made. Definitely we should skip doing checks of
      // voxels that are too far from the surface...
      if (voxel.weight <= config_.min_weight) {
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

  void extractMeshOnBorder(const Block<TsdfVoxel>& block,
                           const VoxelIndex& index, const Point& coords,
                           VertexIndex* next_mesh_index, Mesh* mesh) {
    DCHECK_NOTNULL(mesh);
    Eigen::Matrix<FloatingPoint, 3, 8> cube_coord_offsets =
        cube_index_offsets_.cast<FloatingPoint>() * voxel_size_;
    Eigen::Matrix<FloatingPoint, 3, 8> corner_coords;
    Eigen::Matrix<FloatingPoint, 8, 1> corner_sdf;
    bool all_neighbors_observed = true;
    corner_coords.setZero();
    corner_sdf.setZero();

    for (unsigned int i = 0; i < 8; ++i) {
      VoxelIndex corner_index = index + cube_index_offsets_.col(i);

      if (block.isValidVoxelIndex(corner_index)) {
        const TsdfVoxel& voxel = block.getVoxelByVoxelIndex(corner_index);

        if (voxel.weight <= config_.min_weight) {
          all_neighbors_observed = false;
          break;
        }
        corner_coords.col(i) = coords + cube_coord_offsets.col(i);
        corner_sdf(i) = voxel.distance;
      } else {
        // We have to access a different block.
        BlockIndex block_offset = BlockIndex::Zero();

        for (unsigned int j = 0u; j < 3u; j++) {
          if (corner_index(j) < 0) {
            block_offset(j) = -1;
            corner_index(j) = corner_index(j) + voxels_per_side_;
          } else if (corner_index(j) >= voxels_per_side_) {
            block_offset(j) = 1;
            corner_index(j) = corner_index(j) - voxels_per_side_;
          }
        }

        BlockIndex neighbor_index = block.block_index() + block_offset;

        if (tsdf_layer_->hasBlock(neighbor_index)) {
          const Block<TsdfVoxel>& neighbor_block =
              tsdf_layer_->getBlockByIndex(neighbor_index);

          CHECK(neighbor_block.isValidVoxelIndex(corner_index));
          const TsdfVoxel& voxel =
              neighbor_block.getVoxelByVoxelIndex(corner_index);

          if (voxel.weight <= config_.min_weight) {
            all_neighbors_observed = false;
            break;
          }
          corner_coords.col(i) = coords + cube_coord_offsets.col(i);
          corner_sdf(i) = voxel.distance;
        } else {
          all_neighbors_observed = false;
          break;
        }
      }
    }

    if (all_neighbors_observed) {
      MarchingCubes::meshCube(corner_coords, corner_sdf, next_mesh_index, mesh);
    }
  }

  void updateMeshColor(const Block<TsdfVoxel>& block, Mesh* mesh) {
    CHECK_NOTNULL(mesh);

    mesh->colors.clear();
    mesh->colors.resize(mesh->indices.size());

    // Use nearest-neighbor search.
    for (size_t i = 0; i < mesh->vertices.size(); i++) {
      const Point& vertex = mesh->vertices[i];
      VoxelIndex voxel_index = block.computeVoxelIndexFromCoordinates(vertex);
      if (block.isValidVoxelIndex(voxel_index)) {
        const TsdfVoxel& voxel = block.getVoxelByVoxelIndex(voxel_index);

        if (voxel.weight >= config_.min_weight) {
          mesh->colors[i] = voxel.color;
        }
      } else {
        const Block<TsdfVoxel>::ConstPtr neighbor_block =
            tsdf_layer_->getBlockPtrByCoordinates(vertex);
        const TsdfVoxel& voxel = neighbor_block->getVoxelByCoordinates(vertex);
        if (voxel.weight >= config_.min_weight) {
          mesh->colors[i] = voxel.color;
        }
      }
    }
  }

  void computeMeshNormals(const Block<TsdfVoxel>& block, Mesh* mesh) {
    mesh->normals.clear();
    mesh->normals.resize(mesh->indices.size(), Point::Zero());

    Interpolator<TsdfVoxel> interpolator(tsdf_layer_);

    Point grad;
    for (size_t i = 0; i < mesh->vertices.size(); i++) {
      const Point& pos = mesh->vertices[i];
      // Otherwise the norm stays 0.
      if (interpolator.getGradient(pos, &grad, true)) {
        mesh->normals[i] = grad.normalized();
      } else if (interpolator.getGradient(pos, &grad, false)) {
        mesh->normals[i] = grad.normalized();
      }
    }
  }

 protected:
  Config config_;

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

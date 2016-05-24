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

#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/mesh/marching_cubes.h"
#include "voxblox/mesh/mesh_layer.h"
#include "voxblox/utils/timing.h"

namespace voxblox {

class MeshIntegrator {
 public:
  struct Config {
    bool use_color = true;
    bool compute_normals = true;
  };

  static constexpr FloatingPoint kMinWeight = 2.0;

  MeshIntegrator(Layer<TsdfVoxel>* tsdf_layer, MeshLayer* mesh_layer,
                 const Config& config)
      : tsdf_layer_(tsdf_layer), mesh_layer_(mesh_layer), config_(config) {
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

  void updateMeshForBlock(const BlockIndex& block_index) {
    Mesh::Ptr mesh = mesh_layer_->allocateMeshPtrByIndex(block_index);
    mesh->clear();
    // This block should already exist, otherwise it makes no sense to update
    // the mesh for it. ;)
    Block<TsdfVoxel>::ConstPtr block =
        tsdf_layer_->getBlockPtrByIndex(block_index);

    size_t vps = block->voxels_per_side();
    VertexIndex next_mesh_index = 0;

    VoxelIndex voxel_index;
    for (voxel_index.x() = 0; voxel_index.x() < vps - 1; ++voxel_index.x()) {
      for (voxel_index.y() = 0; voxel_index.y() < vps - 1; ++voxel_index.y()) {
        for (voxel_index.z() = 0; voxel_index.z() < vps - 1;
             ++voxel_index.z()) {
          Point coords = block->computeCoordinatesFromVoxelIndex(voxel_index);
          extractMeshInsideBlock(block, voxel_index, coords, &next_mesh_index,
                                 mesh.get());
        }
      }
    }

    // Max X plane (takes care of max-Y corner as well).
    voxel_index.x() = vps - 1;
    for (voxel_index.z() = 0; voxel_index.z() < vps - 1; voxel_index.z()++) {
      for (voxel_index.y() = 0; voxel_index.y() < vps; voxel_index.y()++) {
        Point coords = block->computeCoordinatesFromVoxelIndex(voxel_index);
        extractMeshOnBorder(block, voxel_index, coords, &next_mesh_index,
                            mesh.get());
      }
    }

    // Max Y plane.
    voxel_index.y() = vps - 1;
    for (voxel_index.z() = 0; voxel_index.z() < vps - 1; voxel_index.z()++) {
      for (voxel_index.x() = 0; voxel_index.x() < vps - 1; voxel_index.x()++) {
        Point coords = block->computeCoordinatesFromVoxelIndex(voxel_index);
        extractMeshOnBorder(block, voxel_index, coords, &next_mesh_index,
                            mesh.get());
      }
    }

    // Max Z plane (also takes care of corners).
    voxel_index.z() = vps - 1;
    for (voxel_index.y() = 0; voxel_index.y() < vps; voxel_index.y()++) {
      for (voxel_index.x() = 0; voxel_index.x() < vps; voxel_index.x()++) {
        Point coords = block->computeCoordinatesFromVoxelIndex(voxel_index);
        extractMeshOnBorder(block, voxel_index, coords, &next_mesh_index,
                            mesh.get());
      }
    }

    // Update colors if needed.
    if (config_.use_color) {
      updateMeshColor(block, mesh.get());
    }

    if (config_.compute_normals) {
      computeMeshNormals(block, mesh.get());
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
      // TODO(helenol): comment above from open_chisel, but no actual checks
      // on distance are ever made. Definitely we should skip doing checks of
      // voxels that are too far from the surface...
      if (voxel.weight <= kMinWeight) {
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

  void extractMeshOnBorder(const Block<TsdfVoxel>::ConstPtr& block,
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

      if (block->isValidVoxelIndex(corner_index)) {
        const TsdfVoxel& voxel = block->getVoxelByVoxelIndex(corner_index);

        if (voxel.weight <= kMinWeight) {
          all_neighbors_observed = false;
          break;
        }
        corner_coords.col(i) = coords + cube_coord_offsets.col(i);
        corner_sdf(i) = voxel.distance;
      } else {
        // We have to access a different block.
        BlockIndex block_offset = Eigen::Vector3i::Zero();

        for (int j = 0; j < 3; j++) {
          if (corner_index(j) < 0) {
            block_offset(j) = -1;
            corner_index(j) = voxels_per_side_ - 1;
          } else if (corner_index(j) >= voxels_per_side_) {
            block_offset(j) = 1;
            corner_index(j) = 0;
          }
        }

        BlockIndex neighbor_index = block->block_index() + block_offset;

        if (tsdf_layer_->hasBlock(neighbor_index)) {
          Block<TsdfVoxel>::ConstPtr neighbor_block =
              tsdf_layer_->getBlockPtrByIndex(neighbor_index);

          const TsdfVoxel& voxel =
              neighbor_block->getVoxelByVoxelIndex(corner_index);

          if (voxel.weight <= kMinWeight) {
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

  void updateMeshColor(const Block<TsdfVoxel>::ConstPtr& block, Mesh* mesh) {
    mesh->colors.clear();
    mesh->colors.resize(mesh->indices.size());

    // Use nearest-neighbor search.
    for (size_t i = 0; i < mesh->vertices.size(); i++) {
      const Point& vertex = mesh->vertices[i];
      VoxelIndex voxel_index = block->computeVoxelIndexFromCoordinates(vertex);
      Point voxel_center = block->computeCoordinatesFromVoxelIndex(voxel_index);

      // Should be within half a voxel of the voxel center in all dimensions, or
      // it belongs in the other one.
      Point dist_from_center = vertex - voxel_center;
      for (unsigned int j = 0; j < 3; ++j) {
        if (std::abs(dist_from_center(j)) > voxel_size_ / 2.0) {
          voxel_index(j) += signum(dist_from_center(j));
        }
      }

      if (block->isValidVoxelIndex(voxel_index)) {
        mesh->colors[i] = block->getVoxelByVoxelIndex(voxel_index).color;
      } else {
        // Get the nearest block.
        const Block<TsdfVoxel>::ConstPtr neighbor_block =
            tsdf_layer_->getBlockPtrByCoordinates(vertex);
        if (neighbor_block) {
          mesh->colors[i] = neighbor_block->getVoxelByCoordinates(vertex).color;
        }
      }
    }
  }

  void computeMeshNormals(const Block<TsdfVoxel>::ConstPtr& block, Mesh* mesh) {
    mesh->normals.clear();
    mesh->normals.resize(mesh->indices.size(), Point::Zero());

    Point grad;
    FloatingPoint dist;
    for (size_t i = 0; i < mesh->vertices.size(); i++) {
      const Point& pos = mesh->vertices[i];
      // Otherwise the norm stays 0.
      if (getDistanceAndGradient(pos, &dist, &grad)) {
        mesh->normals[i] = grad.normalized();
      }
    }
  }

  // TODO(helenol): this has no interpolation within the voxel.
  // At least add an option in the future...
  bool getDistanceAndGradient(const Point& pos, FloatingPoint* distance,
                              Point* grad) {
    CHECK_NOTNULL(distance);
    CHECK_NOTNULL(grad);
    // If we're lucky, everything is within one block.
    const Block<TsdfVoxel>::ConstPtr block =
        tsdf_layer_->getBlockPtrByCoordinates(pos);
    if (!block) {
      return false;
    }
    VoxelIndex voxel_index = block->computeVoxelIndexFromCoordinates(pos);
    const TsdfVoxel& voxel = block->getVoxelByVoxelIndex(voxel_index);
    *distance = static_cast<FloatingPoint>(voxel.distance);
    if (voxel.weight < kMinWeight) {
      return false;
    }

    FloatingPoint interpolated_distance =
        static_cast<FloatingPoint>(voxel.distance);
    Point voxel_center = block->computeCoordinatesFromVoxelIndex(voxel_index);
    Point weight = (pos - voxel_center) / voxel_size_;

    // Now get the gradient.
    *grad = Point::Zero();
    Point offset = Point::Zero();
    // Iterate over all 3 D, and over negative and positive signs in central
    // difference.
    for (unsigned int i = 0; i < 3; ++i) {
      for (int sign = -1; sign <= 1; sign += 2) {
        offset = Point::Zero();
        offset(i) = sign * voxel_size_;
        voxel_index = block->computeVoxelIndexFromCoordinates(pos + offset);
        if (block->isValidVoxelIndex(voxel_index)) {
          const TsdfVoxel& pos_vox = block->getVoxelByVoxelIndex(voxel_index);
          (*grad)(i) += sign * static_cast<FloatingPoint>(pos_vox.distance);
          if (pos_vox.weight < kMinWeight) {
            return false;
          }
        } else {
          const Block<TsdfVoxel>::ConstPtr neighbor_block =
              tsdf_layer_->getBlockPtrByCoordinates(pos + offset);
          if (!neighbor_block) {
            return false;
          }
          const TsdfVoxel& pos_vox =
              neighbor_block->getVoxelByCoordinates(pos + offset);
          (*grad)(i) += sign * static_cast<FloatingPoint>(pos_vox.distance);
          if (pos_vox.weight < kMinWeight) {
            return false;
          }
        }
      }
    }
    // Scale by correct size.
    // This is central difference, so it's 2x voxel size between
    // measurements.
    *grad /= (2 * voxel_size_);
    return true;
  }

 protected:
  Layer<TsdfVoxel>* tsdf_layer_;
  MeshLayer* mesh_layer_;

  Config config_;

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

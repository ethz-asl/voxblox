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
#include <thread>
#include <vector>

#include <glog/logging.h>
#include <Eigen/Core>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/interpolator/interpolator.h"
#include "voxblox/mesh/marching_cubes.h"
#include "voxblox/mesh/mesh_layer.h"
#include "voxblox/utils/timing.h"

namespace voxblox {

template <typename VoxelType>
class MeshIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool use_color = true;
    float min_weight = 1e-4;
    size_t integrator_threads = std::thread::hardware_concurrency();
  };

  MeshIntegrator(const Config& config, Layer<VoxelType>* tsdf_layer,
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

    if (config_.integrator_threads == 0) {
      LOG(WARNING) << "Automatic core count failed, defaulting to 1 threads";
      config_.integrator_threads = 1;
    }
  }

  // Generates mesh for the tsdf layer.
  void generateMesh(bool only_mesh_updated_blocks, bool clear_updated_flag) {
    BlockIndexList all_tsdf_blocks;
    if (only_mesh_updated_blocks) {
      tsdf_layer_->getAllUpdatedBlocks(&all_tsdf_blocks);
    } else {
      tsdf_layer_->getAllAllocatedBlocks(&all_tsdf_blocks);
    }

    // Allocate all the mesh memory
    for (const BlockIndex& block_index : all_tsdf_blocks) {
      mesh_layer_->allocateMeshPtrByIndex(block_index);
    }

    ThreadSafeIndex index_getter(all_tsdf_blocks.size());

    AlignedVector<std::thread> integration_threads;
    for (size_t i = 0; i < config_.integrator_threads; ++i) {
      integration_threads.emplace_back(
          &MeshIntegrator::generateMeshBlocksFunction, this, all_tsdf_blocks,
          clear_updated_flag, &index_getter);
    }

    for (std::thread& thread : integration_threads) {
      thread.join();
    }
  }

  void generateMeshBlocksFunction(const BlockIndexList& all_tsdf_blocks,
                                  bool clear_updated_flag,
                                  ThreadSafeIndex* index_getter) {
    DCHECK(index_getter != nullptr);

    size_t list_idx;
    while (index_getter->getNextIndex(&list_idx)) {
      const BlockIndex& block_idx = all_tsdf_blocks[list_idx];
      typename Block<VoxelType>::Ptr block =
          tsdf_layer_->getBlockPtrByIndex(block_idx);

      updateMeshForBlock(block_idx);
      if (clear_updated_flag) {
        block->updated() = false;
      }
    }
  }

  void extractBlockMesh(typename Block<VoxelType>::ConstPtr block,
                        Mesh::Ptr mesh) {
    DCHECK(block != nullptr);
    DCHECK(mesh != nullptr);

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
    Mesh::Ptr mesh = mesh_layer_->getMeshPtrByIndex(block_index);
    mesh->clear();
    // This block should already exist, otherwise it makes no sense to update
    // the mesh for it. ;)
    typename Block<VoxelType>::ConstPtr block =
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

    mesh->updated = true;
  }

  void extractMeshInsideBlock(const Block<VoxelType>& block,
                              const VoxelIndex& index, const Point& coords,
                              VertexIndex* next_mesh_index, Mesh* mesh) {
    DCHECK(next_mesh_index != nullptr);
    DCHECK(mesh != nullptr);

    Eigen::Matrix<FloatingPoint, 3, 8> cube_coord_offsets =
        cube_index_offsets_.cast<FloatingPoint>() * voxel_size_;
    Eigen::Matrix<FloatingPoint, 3, 8> corner_coords;
    Eigen::Matrix<FloatingPoint, 8, 1> corner_sdf;
    bool all_neighbors_observed = true;

    for (unsigned int i = 0; i < 8; ++i) {
      VoxelIndex corner_index = index + cube_index_offsets_.col(i);
      const VoxelType& voxel = block.getVoxelByVoxelIndex(corner_index);

      // Do not extract a mesh here if one of the corner is unobserved.
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

  void extractMeshOnBorder(const Block<VoxelType>& block,
                           const VoxelIndex& index, const Point& coords,
                           VertexIndex* next_mesh_index, Mesh* mesh) {
    DCHECK(mesh != nullptr);

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
        const VoxelType& voxel = block.getVoxelByVoxelIndex(corner_index);

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
          const Block<VoxelType>& neighbor_block =
              tsdf_layer_->getBlockByIndex(neighbor_index);

          CHECK(neighbor_block.isValidVoxelIndex(corner_index));
          const VoxelType& voxel =
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

  void updateMeshColor(const Block<VoxelType>& block, Mesh* mesh) {
    DCHECK(mesh != nullptr);

    mesh->colors.clear();
    mesh->colors.resize(mesh->indices.size());

    // Use nearest-neighbor search.
    for (size_t i = 0; i < mesh->vertices.size(); i++) {
      const Point& vertex = mesh->vertices[i];
      VoxelIndex voxel_index = block.computeVoxelIndexFromCoordinates(vertex);
      if (block.isValidVoxelIndex(voxel_index)) {
        const VoxelType& voxel = block.getVoxelByVoxelIndex(voxel_index);

        if (voxel.weight >= config_.min_weight) {
          mesh->colors[i] = voxel.color;
        }
      } else {
        const typename Block<VoxelType>::ConstPtr neighbor_block =
            tsdf_layer_->getBlockPtrByCoordinates(vertex);
        const VoxelType& voxel = neighbor_block->getVoxelByCoordinates(vertex);
        if (voxel.weight >= config_.min_weight) {
          mesh->colors[i] = voxel.color;
        }
      }
    }
  }

 protected:
  Config config_;

  Layer<VoxelType>* tsdf_layer_;
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

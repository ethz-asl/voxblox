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
#include <list>
#include <memory>
#include <string>
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
#include "voxblox/utils/meshing_utils.h"
#include "voxblox/utils/timing.h"

namespace voxblox {

struct MeshIntegratorConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool use_color = true;
  float min_weight = 1e-4;

  size_t integrator_threads = std::thread::hardware_concurrency();

  inline std::string print() const {
    std::stringstream ss;
    // clang-format off
    ss << "================== Mesh Integrator Config ====================\n";
    ss << " - use_color:                 " << use_color << "\n";
    ss << " - min_weight:                " << min_weight << "\n";
    ss << " - integrator_threads:        " << integrator_threads << "\n";
    ss << "==============================================================\n";
    // clang-format on
    return ss.str();
  }
};

/**
 * Integrates a TSDF layer to incrementally update a mesh layer using marching
 * cubes.
 */
template <typename VoxelType>
class MeshIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void initFromSdfLayer(const Layer<VoxelType>& sdf_layer) {
    voxel_size_ = sdf_layer.voxel_size();
    block_size_ = sdf_layer.block_size();
    voxels_per_side_ = sdf_layer.voxels_per_side();

    voxel_size_inv_ = 1.0 / voxel_size_;
    block_size_inv_ = 1.0 / block_size_;
    voxels_per_side_inv_ = 1.0 / voxels_per_side_;
  }

  /**
   * Use this constructor in case you would like to modify the layer during mesh
   * extraction, i.e. modify the updated flag.
   */
  MeshIntegrator(const MeshIntegratorConfig& config,
                 Layer<VoxelType>* sdf_layer, MeshLayer* mesh_layer)
      : config_(config),
        sdf_layer_mutable_(CHECK_NOTNULL(sdf_layer)),
        sdf_layer_const_(CHECK_NOTNULL(sdf_layer)),
        mesh_layer_(CHECK_NOTNULL(mesh_layer)) {
    initFromSdfLayer(*sdf_layer);

    cube_index_offsets_ << 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0,
        0, 0, 1, 1, 1, 1;

    if (config_.integrator_threads == 0) {
      LOG(WARNING) << "Automatic core count failed, defaulting to 1 threads";
      config_.integrator_threads = 1;
    }
  }

  /**
   * This constructor will not allow you to modify the layer, i.e. clear the
   * updated flag.
   */
  MeshIntegrator(const MeshIntegratorConfig& config,
                 const Layer<VoxelType>& sdf_layer, MeshLayer* mesh_layer)
      : config_(config),
        sdf_layer_mutable_(nullptr),
        sdf_layer_const_(&sdf_layer),
        mesh_layer_(CHECK_NOTNULL(mesh_layer)) {
    initFromSdfLayer(sdf_layer);

    // clang-format off
    cube_index_offsets_ << 0, 1, 1, 0, 0, 1, 1, 0,
                           0, 0, 1, 1, 0, 0, 1, 1,
                           0, 0, 0, 0, 1, 1, 1, 1;
    // clang-format on

    if (config_.integrator_threads == 0) {
      LOG(WARNING) << "Automatic core count failed, defaulting to 1 threads";
      config_.integrator_threads = 1;
    }
  }

  /// Generates mesh from the tsdf layer.
  void generateMesh(bool only_mesh_updated_blocks, bool clear_updated_flag) {
    CHECK(!clear_updated_flag || (sdf_layer_mutable_ != nullptr))
        << "If you would like to modify the updated flag in the blocks, please "
        << "use the constructor that provides a non-const link to the sdf "
        << "layer!";
    BlockIndexList all_tsdf_blocks;
    if (only_mesh_updated_blocks) {
      sdf_layer_const_->getAllUpdatedBlocks(Update::kMesh, &all_tsdf_blocks);
    } else {
      sdf_layer_const_->getAllAllocatedBlocks(&all_tsdf_blocks);
    }

    // Allocate all the mesh memory
    for (const BlockIndex& block_index : all_tsdf_blocks) {
      mesh_layer_->allocateMeshPtrByIndex(block_index);
    }

    std::unique_ptr<ThreadSafeIndex> index_getter(
        new MixedThreadSafeIndex(all_tsdf_blocks.size()));

    std::list<std::thread> integration_threads;
    for (size_t i = 0; i < config_.integrator_threads; ++i) {
      integration_threads.emplace_back(
          &MeshIntegrator::generateMeshBlocksFunction, this, all_tsdf_blocks,
          clear_updated_flag, index_getter.get());
    }

    for (std::thread& thread : integration_threads) {
      thread.join();
    }
  }

  void generateMeshBlocksFunction(const BlockIndexList& all_tsdf_blocks,
                                  bool clear_updated_flag,
                                  ThreadSafeIndex* index_getter) {
    DCHECK(index_getter != nullptr);
    CHECK(!clear_updated_flag || (sdf_layer_mutable_ != nullptr))
        << "If you would like to modify the updated flag in the blocks, please "
        << "use the constructor that provides a non-const link to the sdf "
        << "layer!";

    size_t list_idx;
    while (index_getter->getNextIndex(&list_idx)) {
      const BlockIndex& block_idx = all_tsdf_blocks[list_idx];
      updateMeshForBlock(block_idx);
      if (clear_updated_flag) {
        typename Block<VoxelType>::Ptr block =
            sdf_layer_mutable_->getBlockPtrByIndex(block_idx);
        block->updated().reset(Update::kMesh);
      }
    }
  }

  void extractBlockMesh(typename Block<VoxelType>::ConstPtr block,
                        Mesh::Ptr mesh) {
    DCHECK(block != nullptr);
    DCHECK(mesh != nullptr);

    IndexElement vps = block->voxels_per_side();
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

    // Max X plane
    // takes care of edge (x_max, y_max, z),
    // takes care of edge (x_max, y, z_max).
    voxel_index.x() = vps - 1;
    for (voxel_index.z() = 0; voxel_index.z() < vps; voxel_index.z()++) {
      for (voxel_index.y() = 0; voxel_index.y() < vps; voxel_index.y()++) {
        Point coords = block->computeCoordinatesFromVoxelIndex(voxel_index);
        extractMeshOnBorder(*block, voxel_index, coords, &next_mesh_index,
                            mesh.get());
      }
    }

    // Max Y plane.
    // takes care of edge (x, y_max, z_max),
    // without corner (x_max, y_max, z_max).
    voxel_index.y() = vps - 1;
    for (voxel_index.z() = 0; voxel_index.z() < vps; voxel_index.z()++) {
      for (voxel_index.x() = 0; voxel_index.x() < vps - 1; voxel_index.x()++) {
        Point coords = block->computeCoordinatesFromVoxelIndex(voxel_index);
        extractMeshOnBorder(*block, voxel_index, coords, &next_mesh_index,
                            mesh.get());
      }
    }

    // Max Z plane.
    voxel_index.z() = vps - 1;
    for (voxel_index.y() = 0; voxel_index.y() < vps - 1; voxel_index.y()++) {
      for (voxel_index.x() = 0; voxel_index.x() < vps - 1; voxel_index.x()++) {
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
        sdf_layer_const_->getBlockPtrByIndex(block_index);

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

      if (!utils::getSdfIfValid(voxel, config_.min_weight, &(corner_sdf(i)))) {
        all_neighbors_observed = false;
        break;
      }

      corner_coords.col(i) = coords + cube_coord_offsets.col(i);
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

        if (!utils::getSdfIfValid(voxel, config_.min_weight,
                                  &(corner_sdf(i)))) {
          all_neighbors_observed = false;
          break;
        }

        corner_coords.col(i) = coords + cube_coord_offsets.col(i);
      } else {
        // We have to access a different block.
        BlockIndex block_offset = BlockIndex::Zero();

        for (unsigned int j = 0u; j < 3u; j++) {
          if (corner_index(j) < 0) {
            block_offset(j) = -1;
            corner_index(j) = corner_index(j) + voxels_per_side_;
          } else if (corner_index(j) >=
                     static_cast<IndexElement>(voxels_per_side_)) {
            block_offset(j) = 1;
            corner_index(j) = corner_index(j) - voxels_per_side_;
          }
        }

        BlockIndex neighbor_index = block.block_index() + block_offset;

        if (sdf_layer_const_->hasBlock(neighbor_index)) {
          const Block<VoxelType>& neighbor_block =
              sdf_layer_const_->getBlockByIndex(neighbor_index);

          CHECK(neighbor_block.isValidVoxelIndex(corner_index));
          const VoxelType& voxel =
              neighbor_block.getVoxelByVoxelIndex(corner_index);

          if (!utils::getSdfIfValid(voxel, config_.min_weight,
                                    &(corner_sdf(i)))) {
            all_neighbors_observed = false;
            break;
          }

          corner_coords.col(i) = coords + cube_coord_offsets.col(i);
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
        utils::getColorIfValid(voxel, config_.min_weight, &(mesh->colors[i]));
      } else {
        const typename Block<VoxelType>::ConstPtr neighbor_block =
            sdf_layer_const_->getBlockPtrByCoordinates(vertex);
        const VoxelType& voxel = neighbor_block->getVoxelByCoordinates(vertex);
        utils::getColorIfValid(voxel, config_.min_weight, &(mesh->colors[i]));
      }
    }
  }

 protected:
  MeshIntegratorConfig config_;

  /**
   * Having both a const and a mutable pointer to the layer allows this
   * integrator to work both with a const layer (in case you don't want to clear
   * the updated flag) and mutable layer (in case you do want to clear the
   * updated flag).
   */
  Layer<VoxelType>* sdf_layer_mutable_;
  const Layer<VoxelType>* sdf_layer_const_;

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

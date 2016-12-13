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

#ifndef VOXBLOX_MESH_MESH_LABEL_INTEGRATOR_H_
#define VOXBLOX_MESH_MESH_LABEL_INTEGRATOR_H_

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>

#include "voxblox/core/color.h"
#include "voxblox/mesh/mesh_integrator.h"

namespace voxblox {

class MeshLabelIntegrator : public MeshIntegrator {
 public:
  MeshLabelIntegrator(const Config& config,
                      Layer<TsdfVoxel>* tsdf_layer,
                      Layer<LabelVoxel>* label_layer,
                      MeshLayer* mesh_layer)
      : MeshIntegrator(config, tsdf_layer, mesh_layer),
        label_layer_(label_layer) {
    DCHECK_NOTNULL(label_layer_);
  }

  virtual void updateMeshForBlock(const BlockIndex& block_index) {
    Mesh::Ptr mesh = mesh_layer_->allocateMeshPtrByIndex(block_index);
    mesh->clear();
    // This block should already exist, otherwise it makes no sense to update
    // the mesh for it. ;)
    Block<TsdfVoxel>::ConstPtr tsdf_block =
        tsdf_layer_->getBlockPtrByIndex(block_index);
    // TODO(grinvalm) Is it safe to assume equal allocation and
    // correspondence of block between the two layers?
    Block<LabelVoxel>::ConstPtr label_block =
            label_layer_->getBlockPtrByIndex(block_index);

    if (!tsdf_block) {
      LOG(ERROR) << "Trying to mesh a non-existent block at index: "
                 << block_index.transpose();
      return;
    }

    size_t vps = tsdf_block->voxels_per_side();
    VertexIndex next_mesh_index = 0;

    VoxelIndex voxel_index;
    for (voxel_index.x() = 0; voxel_index.x() < vps - 1; ++voxel_index.x()) {
      for (voxel_index.y() = 0; voxel_index.y() < vps - 1; ++voxel_index.y()) {
        for (voxel_index.z() = 0; voxel_index.z() < vps - 1;
             ++voxel_index.z()) {
          Point coords =
              tsdf_block->computeCoordinatesFromVoxelIndex(voxel_index);
          extractMeshInsideBlock(*tsdf_block, voxel_index, coords,
                                 &next_mesh_index, mesh.get());
        }
      }
    }

    // Max X plane (takes care of max-Y corner as well).
    voxel_index.x() = vps - 1;
    for (voxel_index.z() = 0; voxel_index.z() < vps - 1; voxel_index.z()++) {
      for (voxel_index.y() = 0; voxel_index.y() < vps; voxel_index.y()++) {
        Point coords =
            tsdf_block->computeCoordinatesFromVoxelIndex(voxel_index);
        extractMeshOnBorder(*tsdf_block, voxel_index, coords,
                            &next_mesh_index, mesh.get());
      }
    }

    // Max Y plane.
    voxel_index.y() = vps - 1;
    for (voxel_index.z() = 0; voxel_index.z() < vps - 1; voxel_index.z()++) {
      for (voxel_index.x() = 0; voxel_index.x() < vps - 1; voxel_index.x()++) {
        Point coords =
            tsdf_block->computeCoordinatesFromVoxelIndex(voxel_index);
        extractMeshOnBorder(*tsdf_block, voxel_index, coords,
                            &next_mesh_index, mesh.get());
      }
    }

    // Max Z plane (also takes care of corners).
    voxel_index.z() = vps - 1;
    for (voxel_index.y() = 0; voxel_index.y() < vps; voxel_index.y()++) {
      for (voxel_index.x() = 0; voxel_index.x() < vps; voxel_index.x()++) {
        Point coords = tsdf_block->computeCoordinatesFromVoxelIndex(voxel_index);
        extractMeshOnBorder(*tsdf_block, voxel_index, coords,
                            &next_mesh_index, mesh.get());
      }
    }

    // Update colors if needed.
    if (config_.use_color) {
      updateMeshColor(*label_block, mesh.get());
    }

    if (config_.compute_normals) {
      computeMeshNormals(*tsdf_block, mesh.get());
    }
  }

  Color getColorFromLabel(const Label& label) {
    // TODO(grinvalm) use a color palette that
    // sequentially gives distant colors.
    return rainbowColorMap(label/1000.0);
  }


  void updateMeshColor(const Block<LabelVoxel>& label_block, Mesh* mesh) {
    mesh->colors.clear();
    mesh->colors.resize(mesh->indices.size());

    // Use nearest-neighbor search.
    for (size_t i = 0u; i < mesh->vertices.size(); i++) {
      const Point& vertex = mesh->vertices[i];
      VoxelIndex voxel_index =
          label_block.computeVoxelIndexFromCoordinates(vertex);
      Point voxel_center =
          label_block.computeCoordinatesFromVoxelIndex(voxel_index);

      // Should be within half a voxel of the voxel center in all dimensions, or
      // it belongs in the other one.
      Point dist_from_center = vertex - voxel_center;
      for (unsigned int j = 0; j < 3; ++j) {
        if (std::abs(dist_from_center(j)) > voxel_size_ / 2.0) {
          voxel_index(j) += signum(dist_from_center(j));
        }
      }

      if (label_block.isValidVoxelIndex(voxel_index)) {
        mesh->colors[i] = getColorFromLabel(
            label_block.getVoxelByVoxelIndex(voxel_index).label);
      } else {
        // Get the nearest block.
        const Block<LabelVoxel>::ConstPtr neighbor_block =
            label_layer_->getBlockPtrByCoordinates(vertex);
        if (neighbor_block) {
          mesh->colors[i] = getColorFromLabel(
              neighbor_block->getVoxelByCoordinates(vertex).label);
        }
      }
    }
  }

 protected:
  Layer<LabelVoxel>* label_layer_;
};

}  // namespace voxblox

#endif  // VOXBLOX_MESH_MESH_LABEL_INTEGRATOR_H_

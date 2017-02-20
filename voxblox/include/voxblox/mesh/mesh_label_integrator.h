#ifndef VOXBLOX_MESH_MESH_LABEL_INTEGRATOR_H_
#define VOXBLOX_MESH_MESH_LABEL_INTEGRATOR_H_

#include <algorithm>
#include <cmath>
#include <map>
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
        label_layer_(CHECK_NOTNULL(label_layer)) {}

  virtual void updateMeshForBlock(const BlockIndex& block_index) {
    Mesh::Ptr mesh = mesh_layer_->allocateMeshPtrByIndex(block_index);
    mesh->clear();
    // This block should already exist, otherwise it makes no sense to update
    // the mesh for it. ;)
    Block<TsdfVoxel>::ConstPtr tsdf_block =
        tsdf_layer_->getBlockPtrByIndex(block_index);
    Block<LabelVoxel>::ConstPtr label_block =
            label_layer_->getBlockPtrByIndex(block_index);

    if (!tsdf_block && !label_block) {
      LOG(ERROR) << "Trying to mesh a non-existent block at index: "
                 << block_index.transpose();
      return;
    } else if (!(tsdf_block && label_block)) {
      LOG(FATAL) << "Block allocation differs between the two layers.";
    }

    extractBlockMesh(tsdf_block, mesh);

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
    Color color;
    auto label_color_map_it = label_color_map_.find(label);

    if (label_color_map_it != label_color_map_.end()) {
      color = label_color_map_it->second;
    } else {
      color = randomColor();
      label_color_map_.insert(std::pair<Label, Color>(label, color));
    }
    return color;
  }


  void updateMeshColor(const Block<LabelVoxel>& label_block, Mesh* mesh) {
    CHECK_NOTNULL(mesh);

    mesh->colors.clear();
    mesh->colors.resize(mesh->indices.size());

    // Use nearest-neighbor search.
    for (size_t i = 0u; i < mesh->vertices.size(); i++) {
      const Point& vertex = mesh->vertices[i];
      VoxelIndex voxel_index =
          label_block.computeVoxelIndexFromCoordinates(vertex);
      const Point voxel_center =
          label_block.computeCoordinatesFromVoxelIndex(voxel_index);

      // Should be within half a voxel of the voxel center in all dimensions, or
      // it belongs in the other one.
      const Point dist_from_center = vertex - voxel_center;
      for (unsigned int j = 0u; j < 3u; ++j) {
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

  std::map<Label, Color> label_color_map_;
};

}  // namespace voxblox

#endif  // VOXBLOX_MESH_MESH_LABEL_INTEGRATOR_H_

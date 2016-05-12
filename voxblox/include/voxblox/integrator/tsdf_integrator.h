#ifndef VOXBLOX_INTEGRATOR_RAY_INTEGRATOR_H_
#define VOXBLOX_INTEGRATOR_RAY_INTEGRATOR_H_

#include <algorithm>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/utils/timing.h"

namespace voxblox {

class TsdfIntegrator {
 public:
  struct Config {
    float default_truncation_distance = 0.1;
    float max_weight = 100.0;
    bool voxel_carving_enabled = true;
  };

  TsdfIntegrator(Layer<TsdfVoxel>* layer, const Config& config)
      : layer_(layer), config_(config) {
    DCHECK(layer_);

    voxel_size_ = layer_->voxel_size();
    block_size_ = layer_->block_size();
    voxels_per_side_ = layer_->voxels_per_side();

    voxel_size_inv_ = 1.0 / voxel_size_;
    block_size_inv_ = 1.0 / block_size_;
    voxels_per_side_inv_ = 1.0 / voxels_per_side_;
  }

  inline void updateTsdfVoxel(const Point& origin, const Point& point_C,
                              const Point& point_G, const Point& voxel_center,
                              const Color& color,
                              const float truncation_distance,
                              TsdfVoxel* tsdf_voxel) {
    Eigen::Vector3d voxel_direction = point_G - voxel_center;
    Eigen::Vector3d ray_direction = point_G - origin;

    float sdf = static_cast<float>(voxel_direction.norm());
    // Figure out if it's in front of the plane or behind.
    if (voxel_direction.dot(ray_direction) < 0) {
      sdf = -sdf;
    }

    const float weight = 1.0f;
    const float new_weight = tsdf_voxel->weight + weight;

    tsdf_voxel->color = Color::blendTwoColors(
        tsdf_voxel->color, tsdf_voxel->weight, color, weight);
    const float new_sdf =
        (sdf * weight + tsdf_voxel->distance * tsdf_voxel->weight) / new_weight;

    tsdf_voxel->distance = (new_sdf > 0.0)
                               ? std::min(truncation_distance, new_sdf)
                               : std::max(-truncation_distance, new_sdf);
    tsdf_voxel->weight = std::min(config_.max_weight, new_weight);
  }

  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C, const Colors& colors) {
    DCHECK_EQ(points_C.size(), colors.size());
    timing::Timer integrate_timer("integrate");

    const Point& origin = T_G_C.getPosition();

    for (size_t pt_idx = 0; pt_idx < points_C.size(); ++pt_idx) {
      const Point& point_C = points_C[pt_idx];
      const Point point_G = T_G_C * point_C;
      const Color& color = colors[pt_idx];

      HierarchicalIndexMap hierarchical_idx_map;
      FloatingPoint truncation_distance = config_.default_truncation_distance;

      getHierarchicalIndexAlongRay(
          origin, point_G, voxels_per_side_, voxel_size_, truncation_distance,
          config_.voxel_carving_enabled, &hierarchical_idx_map);

      timing::Timer update_voxels_timer("integrate/update_voxels");
      for (const HierarchicalIndex& hierarchical_idx : hierarchical_idx_map) {
        Block<TsdfVoxel>::Ptr block =
            layer_->allocateBlockPtrByIndex(hierarchical_idx.first);
        block->updated() = true;
        DCHECK(block);
        for (const VoxelIndex& local_voxel_idx : hierarchical_idx.second) {
          const Point voxel_center_G =
              block->computeCoordinatesFromVoxelIndex(local_voxel_idx);
          TsdfVoxel& tsdf_voxel = block->getVoxelByVoxelIndex(local_voxel_idx);

          updateTsdfVoxel(origin, point_C, point_G, voxel_center_G, color,
                          truncation_distance, &tsdf_voxel);
        }
      }
      update_voxels_timer.Stop();
    }
    integrate_timer.Stop();
  }

 protected:
  Layer<TsdfVoxel>* layer_;

  Config config_;

  // Cached map config.
  FloatingPoint voxel_size_;
  size_t voxels_per_side_;
  FloatingPoint block_size_;

  // Derived types.
  FloatingPoint voxel_size_inv_;
  FloatingPoint voxels_per_side_inv_;
  FloatingPoint block_size_inv_;
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_RAY_INTEGRATOR_H_

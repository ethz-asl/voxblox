#ifndef VOXBLOX_INTEGRATOR_RAY_INTEGRATOR_H_
#define VOXBLOX_INTEGRATOR_RAY_INTEGRATOR_H_

#include <algorithm>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/integrator_utils.h"

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
    const float sdf = static_cast<float>((point_G - voxel_center).norm());
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

    const Point& origin = T_G_C.getPosition();
    const Point& origin_scaled = origin * voxel_size_inv_;

    for (size_t pt_idx = 0; pt_idx < points_C.size(); ++pt_idx) {
      const Point& point_C = points_C[pt_idx];
      const Point& point_G = T_G_C * point_C;
      const Color& color = colors[pt_idx];

      const Ray unit_ray = (point_G - origin).normalized();

      const FloatingPoint truncation_distance =
          config_.default_truncation_distance;

      const Point ray_end = point_G + unit_ray * truncation_distance;
      const Point ray_start = (config_.voxel_carving_enabled)
                                  ? origin
                                  : (point_G - unit_ray * truncation_distance);

      const Point& point_G_scaled = point_G * voxel_size_inv_;

      IndexVector global_voxel_index;
      castRay(origin_scaled, point_G_scaled, &global_voxel_index);

      HierarchicalIndexMap hierarchical_idx_map;
      for (const AnyIndex& global_voxel_idx : global_voxel_index) {
        BlockIndex block_idx = floorVectorAndDowncast(
            global_voxel_idx.cast<FloatingPoint>() * voxels_per_side_inv_);

        VoxelIndex local_voxel_idx(global_voxel_idx.x() % voxels_per_side_,
                                   global_voxel_idx.y() % voxels_per_side_,
                                   global_voxel_idx.z() % voxels_per_side_);

        if (local_voxel_idx.x() < 0) {
          local_voxel_idx.x() += voxels_per_side_;
        }
        if (local_voxel_idx.y() < 0) {
          local_voxel_idx.y() += voxels_per_side_;
        }
        if (local_voxel_idx.z() < 0) {
          local_voxel_idx.z() += voxels_per_side_;
        }

        hierarchical_idx_map[block_idx].push_back(local_voxel_idx);
      }

      for (const HierarchicalIndex& hierarchical_idx : hierarchical_idx_map) {
        Block<TsdfVoxel>::Ptr block =
            layer_->allocateBlockPtrByIndex(hierarchical_idx.first);
        DCHECK(block);
        for (const VoxelIndex& local_voxel_idx : hierarchical_idx.second) {
          const Point voxel_center_G =
              block->computeCoordinatesFromVoxelIndex(local_voxel_idx);
          TsdfVoxel& tsdf_voxel = block->getVoxelByVoxelIndex(local_voxel_idx);

          updateTsdfVoxel(origin, point_C, point_G, voxel_center_G, color,
                          truncation_distance, &tsdf_voxel);
        }
      }
    }
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

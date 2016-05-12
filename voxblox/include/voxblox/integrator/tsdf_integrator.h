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

  float getVoxelWeight(const Point& point_C, const Point& point_G,
                       const Point& voxel_center) const {
    return 1.0f;
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

    const float weight = getVoxelWeight(point_C, point_G, voxel_center);
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

  void integratePointCloudMerged(const Transformation& T_G_C,
                                    const Pointcloud& points_C,
                                    const Colors& colors) {
    DCHECK_EQ(points_C.size(), colors.size());
    timing::Timer integrate_timer("integrate");

    const Point& origin = T_G_C.getPosition();

    // Pre-compute a list of unique voxels to end on.
    // Create a hashmap: VOXEL INDEX -> index in original cloud.
    BlockHashMapType<std::vector<size_t>>::type voxel_map;
    for (size_t pt_idx = 0; pt_idx < points_C.size(); ++pt_idx) {
      const Point& point_C = points_C[pt_idx];
      const Point point_G = T_G_C * point_C;

      // Figure out what the end voxel is here.
      VoxelIndex voxel_index = floorVectorAndDowncast(
          point_G.cast<FloatingPoint>() * voxel_size_inv_);
      voxel_map[voxel_index].push_back(pt_idx);
    }

    LOG(INFO) << "Went from " << points_C.size() << " points to "
              << voxel_map.size() << " raycasts.";

    FloatingPoint truncation_distance = config_.default_truncation_distance;
    for (const BlockHashMapType<std::vector<size_t>>::type::value_type& kv :
         voxel_map) {
      if (kv.second.empty()) {
        continue;
      }
      // Key actually doesn't matter at all.
      Point mean_point_C = Point::Zero();
      Color mean_color;
      float current_weight = 0.0;

      for (size_t pt_idx : kv.second) {
        const Point& point_C = points_C[pt_idx];
        const Color& color = colors[pt_idx];

        // TODO(helenol): proper weights, proper merging.
        float point_weight = getVoxelWeight(point_C, point_C, point_C);
        mean_point_C =
            (mean_point_C * current_weight + point_C * point_weight) /
            (current_weight + point_weight);
        mean_color = Color::blendTwoColors(mean_color, current_weight, color,
                                           point_weight);
      }

      const Point point_G = T_G_C * mean_point_C;

      HierarchicalIndexMap hierarchical_idx_map;
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

          updateTsdfVoxel(origin, mean_point_C, point_G, voxel_center_G,
                          mean_color, truncation_distance, &tsdf_voxel);
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

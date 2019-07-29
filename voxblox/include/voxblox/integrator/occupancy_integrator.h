#ifndef VOXBLOX_INTEGRATOR_OCCUPANCY_INTEGRATOR_H_
#define VOXBLOX_INTEGRATOR_OCCUPANCY_INTEGRATOR_H_

#include <algorithm>
#include <vector>

#include <glog/logging.h>
#include <Eigen/Core>

#include "voxblox/core/block_hash.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/utils/timing.h"

namespace voxblox {
/**
 * Integrates a pointcloud into an occupancy layer by raycasting the points and
 * updating all the voxels the rays pass through.
 */
class OccupancyIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    float probability_hit = 0.65f;
    float probability_miss = 0.4f;
    float threshold_min = 0.12f;
    float threshold_max = 0.97f;
    float threshold_occupancy = 0.7f;
    FloatingPoint min_ray_length_m = 0.1;
    FloatingPoint max_ray_length_m = 5.0;
  };

  OccupancyIntegrator(const Config& config, Layer<OccupancyVoxel>* layer)
      : config_(config), layer_(layer) {
    DCHECK(layer_ != NULL);
    DCHECK_GT(layer_->voxel_size(), 0.0);
    DCHECK_GT(layer_->block_size(), 0.0);
    DCHECK_GT(layer_->voxels_per_side(), 0u);

    voxel_size_ = layer_->voxel_size();
    block_size_ = layer_->block_size();
    voxels_per_side_ = layer_->voxels_per_side();

    voxel_size_inv_ = 1.0 / voxel_size_;
    block_size_inv_ = 1.0 / block_size_;
    voxels_per_side_inv_ = 1.0 / voxels_per_side_;

    // Cache rest of the probability settings.
    prob_hit_log_ = logOddsFromProbability(config_.probability_hit);
    prob_miss_log_ = logOddsFromProbability(config_.probability_miss);
    clamp_min_log_ = logOddsFromProbability(config_.threshold_min);
    clamp_max_log_ = logOddsFromProbability(config_.threshold_max);
    min_occupancy_log_ = logOddsFromProbability(config_.threshold_occupancy);
  }

  inline void updateOccupancyVoxel(bool occupied, OccupancyVoxel* occ_voxel) {
    DCHECK(occ_voxel != NULL);
    // Set voxel to observed.
    occ_voxel->observed = true;
    // Skip update if necessary.
    float log_odds_update = occupied ? prob_hit_log_ : prob_miss_log_;
    if ((log_odds_update >= 0 &&
         occ_voxel->probability_log >= clamp_max_log_) ||
        (log_odds_update <= 0 &&
         occ_voxel->probability_log <= clamp_min_log_)) {
      return;
    }
    // Otherwise update and reclamp.
    occ_voxel->probability_log = std::min(
        std::max(occ_voxel->probability_log + log_odds_update, clamp_min_log_),
        clamp_max_log_);
  }

  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C) {
    timing::Timer integrate_timer("integrate_occ");

    const Point& origin = T_G_C.getPosition();

    LongIndexSet free_cells;
    LongIndexSet occupied_cells;

    const Point start_scaled = origin * voxel_size_inv_;
    Point end_scaled = Point::Zero();

    for (size_t pt_idx = 0; pt_idx < points_C.size(); ++pt_idx) {
      const Point& point_C = points_C[pt_idx];
      const Point point_G = T_G_C * point_C;
      const Ray unit_ray = (point_G - origin).normalized();

      timing::Timer cast_ray_timer("integrate_occ/cast_ray");

      AlignedVector<GlobalIndex> global_voxel_indices;
      FloatingPoint ray_distance = (point_G - origin).norm();
      if (ray_distance < config_.min_ray_length_m) {
        continue;
      } else if (ray_distance > config_.max_ray_length_m) {
        // Simply clear up until the max ray distance in this case.
        end_scaled =
            (origin + config_.max_ray_length_m * unit_ray) * voxel_size_inv_;

        if (free_cells.find(getGridIndexFromPoint<GlobalIndex>(end_scaled)) ==
            free_cells.end()) {
          castRay(start_scaled, end_scaled, &global_voxel_indices);
          free_cells.insert(global_voxel_indices.begin(),
                            global_voxel_indices.end());
        }
      } else {
        end_scaled = point_G * voxel_size_inv_;
        if (occupied_cells.find(getGridIndexFromPoint<GlobalIndex>(
                end_scaled)) == occupied_cells.end()) {
          castRay(start_scaled, end_scaled, &global_voxel_indices);

          if (global_voxel_indices.size() > 2) {
            free_cells.insert(global_voxel_indices.begin(),
                              global_voxel_indices.end() - 1);
            occupied_cells.insert(global_voxel_indices.back());
          }
        }
      }
      cast_ray_timer.Stop();
    }

    timing::Timer update_voxels_timer("integrate_occ/update_occupancy");

    // Clean up the lists: remove any occupied cells from free cells.
    for (const GlobalIndex& global_index : occupied_cells) {
      LongIndexSet::iterator cell_it = free_cells.find(global_index);
      if (cell_it != free_cells.end()) {
        free_cells.erase(cell_it);
      }
    }
    // Then actually update the occupancy voxels.
    BlockIndex last_block_idx = BlockIndex::Zero();
    Block<OccupancyVoxel>::Ptr block;

    bool occupied = false;
    for (const GlobalIndex& global_voxel_idx : free_cells) {
      BlockIndex block_idx = getBlockIndexFromGlobalVoxelIndex(
          global_voxel_idx, voxels_per_side_inv_);
      VoxelIndex local_voxel_idx =
          getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);

      if (!block || block_idx != last_block_idx) {
        block = layer_->allocateBlockPtrByIndex(block_idx);
        block->updated().set();
        last_block_idx = block_idx;
      }

      OccupancyVoxel& occ_voxel = block->getVoxelByVoxelIndex(local_voxel_idx);
      updateOccupancyVoxel(occupied, &occ_voxel);
    }
    block.reset();

    occupied = true;
    for (const GlobalIndex& global_voxel_idx : occupied_cells) {
      BlockIndex block_idx = getBlockIndexFromGlobalVoxelIndex(
          global_voxel_idx, voxels_per_side_inv_);
      VoxelIndex local_voxel_idx =
          getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);

      if (!block || block_idx != last_block_idx) {
        block = layer_->allocateBlockPtrByIndex(block_idx);
        block->updated().set();
        last_block_idx = block_idx;
      }

      OccupancyVoxel& occ_voxel = block->getVoxelByVoxelIndex(local_voxel_idx);
      updateOccupancyVoxel(occupied, &occ_voxel);
    }

    update_voxels_timer.Stop();
    integrate_timer.Stop();
  }

 protected:
  Config config_;

  Layer<OccupancyVoxel>* layer_;

  // Cached map config.
  FloatingPoint voxel_size_;
  size_t voxels_per_side_;
  FloatingPoint block_size_;

  float prob_hit_log_;
  float prob_miss_log_;
  float clamp_min_log_;
  float clamp_max_log_;
  float min_occupancy_log_;

  // Derived types.
  FloatingPoint voxel_size_inv_;
  FloatingPoint voxels_per_side_inv_;
  FloatingPoint block_size_inv_;
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_OCCUPANCY_INTEGRATOR_H_

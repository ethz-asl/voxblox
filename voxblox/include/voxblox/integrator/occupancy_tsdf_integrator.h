#ifndef VOXBLOX_INTEGRATOR_OCCUPANCY_TSDF_INTEGRATOR_H_
#define VOXBLOX_INTEGRATOR_OCCUPANCY_TSDF_INTEGRATOR_H_

#include <algorithm>
#include <cmath>
#include <vector>

#include <glog/logging.h>
#include <Eigen/Core>

#include "voxblox/core/block_hash.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/utils/timing.h"

namespace voxblox {

// Generate occupancy map from TSDF map by a simple truncation
class OccTsdfIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool allow_clear = true;

    FloatingPoint min_weight = 1e-4;

    // The threshold of TSDF distance is occ_voxel_size_ratio * voxel size
    FloatingPoint occ_voxel_size_ratio = 0.865;  // Sqrt(3)/2
  };

  OccTsdfIntegrator(const Config& config, Layer<TsdfVoxel>* tsdf_layer,
                    Layer<OccupancyVoxel>* occ_layer)
      : config_(config), tsdf_layer_(tsdf_layer), occ_layer_(occ_layer) {
    CHECK_NOTNULL(tsdf_layer_);
    CHECK_NOTNULL(occ_layer_);

    voxels_per_side_ = tsdf_layer_->voxels_per_side();
    voxel_size_ = tsdf_layer_->voxel_size();
  }

  void updateFromTsdfLayer(bool clear_updated_flag, bool in_batch) {
    BlockIndexList updated_tsdf_blocks;
    if (in_batch)
      tsdf_layer_->getAllAllocatedBlocks(&updated_tsdf_blocks);
    else
      tsdf_layer_->getAllUpdatedBlocks(Update::kEsdf, &updated_tsdf_blocks);
    const bool kIncremental = true;
    updateFromTsdfBlocks(updated_tsdf_blocks, kIncremental);

    if (clear_updated_flag) {
      for (const BlockIndex& block_index : updated_tsdf_blocks) {
        if (tsdf_layer_->hasBlock(block_index)) {
          tsdf_layer_->getBlockByIndex(block_index)
              .setUpdated(Update::kEsdf, false);
        }
      }
    }
  }

  void updateFromTsdfBlocks(const BlockIndexList& tsdf_blocks,
                            bool kIncremental) {
    CHECK_EQ(tsdf_layer_->voxels_per_side(), occ_layer_->voxels_per_side());
    CHECK_EQ(tsdf_layer_->voxel_size(), occ_layer_->voxel_size());

    timing::Timer allocate_timer("update_occ/allocate_vox");

    clearList();

    // Go through all blocks in TSDF map (that are recently updated)
    // and copy their values for relevant voxels.
    if (kIncremental) {
      for (const BlockIndex& block_index : tsdf_blocks) {
        Block<TsdfVoxel>::ConstPtr tsdf_block =
            tsdf_layer_->getBlockPtrByIndex(block_index);
        if (!tsdf_block) continue;

        // Allocate the same block in the occupancy layer.
        // Block indices are the same across all layers.
        Block<OccupancyVoxel>::Ptr occ_block =
            occ_layer_->allocateBlockPtrByIndex(block_index);
        occ_block->setUpdatedAll();

        const size_t num_voxels_per_block = occ_block->num_voxels();
        for (size_t lin_index = 0u; lin_index < num_voxels_per_block;
             ++lin_index) {
          const TsdfVoxel& tsdf_voxel =
              tsdf_block->getVoxelByLinearIndex(lin_index);
          if (tsdf_voxel.weight >= config_.min_weight) {
            OccupancyVoxel& occ_voxel =
                occ_block->getVoxelByLinearIndex(lin_index);
            occ_voxel.observed = true;
            occ_voxel.behind = (tsdf_voxel.distance < 0.0);
            bool original_occ_state = occ_voxel.occupied;
            occ_voxel.occupied = (std::abs(tsdf_voxel.distance) <=
                                  config_.occ_voxel_size_ratio * voxel_size_);
            if (occ_voxel.occupied)
              occ_voxel.probability_log = 1.0;  // only for visualization
            else
              occ_voxel.probability_log = 0.0;

            VoxelIndex voxel_index =
                occ_block->computeVoxelIndexFromLinearIndex(lin_index);
            GlobalIndex global_index =
                getGlobalVoxelIndexFromBlockAndVoxelIndex(
                    block_index, voxel_index, occ_layer_->voxels_per_side());

            if (original_occ_state && (!occ_voxel.occupied))
              delete_list_.push_back(global_index);
            else if ((!original_occ_state) && occ_voxel.occupied)
              insert_list_.push_back(global_index);
          }
        }
      }
    }
    allocate_timer.Stop();
  }

  GlobalIndexList getInsertList() { return insert_list_; }

  GlobalIndexList getDeleteList() { return delete_list_; }

  inline void clearList() {
    GlobalIndexList().swap(insert_list_);
    GlobalIndexList().swap(delete_list_);
  }

 protected:
  Config config_;

  Layer<TsdfVoxel>* tsdf_layer_;
  Layer<OccupancyVoxel>* occ_layer_;

  // Cached map config.
  FloatingPoint voxel_size_;
  size_t voxels_per_side_;
  FloatingPoint block_size_;

  // Derived types.
  FloatingPoint voxel_size_inv_;
  FloatingPoint voxels_per_side_inv_;
  FloatingPoint block_size_inv_;

  // Used for FIESTA ESDF mapping
  GlobalIndexList insert_list_;
  GlobalIndexList delete_list_;
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_OCCUPANCY_TSDF_INTEGRATOR_H_

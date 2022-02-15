#ifndef VOXBLOX_INTEGRATOR_ESDF_OCC_EDT_INTEGRATOR_H_
#define VOXBLOX_INTEGRATOR_ESDF_OCC_EDT_INTEGRATOR_H_

#include <algorithm>
#include <queue>
#include <utility>
#include <vector>

#include <glog/logging.h>
#include <Eigen/Core>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/utils/bucket_queue.h"
#include "voxblox/utils/neighbor_tools.h"
#include "voxblox/utils/timing.h"

namespace voxblox {

/**
 * Builds an ESDF layer out of a given occupancy layer.
 */
class EsdfOccEdtIntegrator {  
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool verbose = false;

    /**
     * Maximum distance to calculate the actual distance to.
     * Any values above this will be set to default_distance_m.
     */
    FloatingPoint max_distance_m = 2.0;

    // Default distance set for unknown values and values > max_distance_m.
    FloatingPoint default_distance_m = 2.0;

    // Trunction distance behind surface, unit: m
    FloatingPoint max_behind_surface_m = 1.0f;

    // Number of buckets for the bucketed priority queue.
    int num_buckets = 20;

    // Number of the neighbor voxels (select from 6, 18, 24 and 26)
    // TODO: double-check if 24 is the best choice
    int num_neighbor = 24;

    // Turn on the patch code (Algorithm 3 in FIESTA) or not
    bool patch_on = true;
    bool early_break = false;

    // Local map boundary size (unit: voxel)
    GlobalIndex range_boundary_offset = GlobalIndex(100, 100, 50);
  };

  EsdfOccEdtIntegrator(const Config& config,
                          Layer<OccupancyVoxel>* occ_layer,
                          Layer<EsdfVoxel>* esdf_layer);

  void updateFromOccLayer(bool clear_updated_flag);

  void updateFromOccBlocks(const BlockIndexList& occ_blocks);

  void setLocalRange();

  void getUpdateRange();

  void resetFixed();

  void updateESDF();

  void processRaise(EsdfVoxel* cur_vox);

  void processLower(EsdfVoxel* cur_vox);

  void deleteFromList(EsdfVoxel* occ_vox, EsdfVoxel* cur_vox);

  void insertIntoList(EsdfVoxel* occ_vox, EsdfVoxel* cur_vox);

  inline float dist(GlobalIndex vox_idx_a, GlobalIndex vox_idx_b);

  inline int distSquare(GlobalIndex vox_idx_a, GlobalIndex vox_idx_b);
                                            
  inline bool voxInRange(GlobalIndex vox_idx);

  void loadInsertList(const GlobalIndexList& insert_list);

  void loadDeleteList(const GlobalIndexList& delete_list);

  void assignError(GlobalIndex vox_idx, float esdf_error);

  inline void clear() {
    GlobalIndexList().swap(insert_list_);
    GlobalIndexList().swap(delete_list_);
    update_queue_.clear();
    updated_voxel_.clear();
  }

  /// Update some specific settings.
  float getEsdfMaxDistance() const { return config_.max_distance_m; }
  void setEsdfMaxDistance(float max_distance) {
    config_.max_distance_m = max_distance;
    if (config_.default_distance_m < max_distance) {
      config_.default_distance_m = max_distance;
    }
  }

 protected:
  Config config_;

  Layer<OccupancyVoxel>* occ_layer_;
  Layer<EsdfVoxel>* esdf_layer_;

  // Data structure used for FIESTA
  GlobalIndexList insert_list_;
  GlobalIndexList delete_list_;
  BucketQueue<GlobalIndex> update_queue_;
  LongIndexSet updated_voxel_;  // TODO

  size_t esdf_voxels_per_side_;
  FloatingPoint esdf_voxel_size_;

  // Update (inseted and deleted occupied voxels) range, unit: voxel
  GlobalIndex update_range_min_;
  GlobalIndex update_range_max_;

  // Local map range (update range + boundary size), unit: voxel
  GlobalIndex range_min_;
  GlobalIndex range_max_;

  // for recording and logging
  int total_updated_count_ = 0;

  int sum_occ_changed_ = 0;
  int sum_raise_ = 0;
  int sum_lower_ = 0;
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_ESDF_OCC_EDT_INTEGRATOR_H_

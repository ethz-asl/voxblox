#ifndef VOXBLOX_INTEGRATOR_ESDF_VOXFIELD_INTEGRATOR_H_
#define VOXBLOX_INTEGRATOR_ESDF_VOXFIELD_INTEGRATOR_H_

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
#include "voxblox/utils/neighbor_tools_ex.h"
#include "voxblox/utils/timing.h"

namespace voxblox {

/**
 * Builds an ESDF layer out of a given TSDF layer efficiently.
 */
class EsdfVoxfieldIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // output log or not
    bool verbose = false;

    /**
     * Maximum distance to calculate the actual distance to.
     * Any values above this will be set to default_distance_m.
     */
    FloatingPoint max_distance_m = 10.0;

    // Default distance set for unknown values and values > max_distance_m.
    FloatingPoint default_distance_m = 10.0;

    // Trunction distance behind surface, unit: m
    FloatingPoint max_behind_surface_m = 1.0f;

    // Fixed band distance threshold, unit: m
    FloatingPoint band_distance_m = 1.0f; 

    // The threshold of TSDF distance is occ_voxel_size_ratio * voxel size
    FloatingPoint occ_voxel_size_ratio = 0.865;  // Sqrt(3) / 2

    // Minimum weight to consider a TSDF value seen at.
    float min_weight = 1e-6;

    // Default voxel unit distance square (deprecated)
    // int default_dist_square = 50 * 50; 

    // Number of buckets for the bucketed priority queue.
    int num_buckets = 20;

    // Number of the neighbor voxels (select from 6, 18, 24 and 26)
    // TODO: double-check if 24 is the best choice (according to FIESTA paper,
    // it's the best)
    int num_neighbor = 24;

    // Turn on the patch code (Algorithm 3 in FIESTA) or not
    bool patch_on = true;
    bool early_break = true;
    
    // Finer ESDF with the consideration of the inner voxel distance from the voxel center to the actual surface
    // Gradient TSDF is needed for the calculation
    bool finer_esdf_on = false;

    // use a fixed band for esdf to directly copy the tsdf value
    bool fixed_band_esdf_on = false;

    float gradient_sign =
        1.0f;  // sign (direction) of the gradient, towards or opposite to the
               // surface, select from 1.0 or -1.0

    bool allocate_tsdf_in_range = false;

    // Local map boundary size (unit: voxel)
    GlobalIndex range_boundary_offset = GlobalIndex(10, 10, 5);
  };

  EsdfVoxfieldIntegrator(const Config& config, Layer<TsdfVoxel>* tsdf_layer,
                         Layer<EsdfVoxel>* esdf_layer);

  void updateFromTsdfLayer(bool clear_updated_flag);

  void updateFromTsdfBlocks(const BlockIndexList& tsdf_blocks);

  void setLocalRange();

  void getUpdateRange();

  void resetFixed();

  void updateESDF();

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

  // Convenience functions. Determine if the voxel is in the fixed band
  inline bool isFixed(FloatingPoint dist_m) const {
    return std::abs(dist_m) < config_.band_distance_m;
  }

  inline bool isOccupied(FloatingPoint dist_m) const {
    return std::abs(dist_m) <= config_.occ_voxel_size_ratio * esdf_voxel_size_;                             
  }

  inline bool isOccupied(FloatingPoint dist_m, Ray gradient) const {
    if (gradient.norm() > kFloatEpsilon) {
      Ray dist_on_axis = dist_m * gradient;
      bool is_occupied = true;
      for (int i = 0; i < 3; i++) {
        if (std::abs(dist_on_axis(i)) > 0.5 * esdf_voxel_size_)
          is_occupied = false;
      }
      return is_occupied;
    } else {
      return isOccupied(dist_m);
    }
  }

  inline bool isObserved(FloatingPoint weight) const {
    return weight >= config_.min_weight;
  }

 protected:
  Config config_;

  Layer<TsdfVoxel>* tsdf_layer_;
  Layer<EsdfVoxel>* esdf_layer_;

  // Data structure used for FIESTA
  GlobalIndexList insert_list_;
  GlobalIndexList delete_list_;
  BucketQueue<GlobalIndex> update_queue_;
  LongIndexSet updated_voxel_;  

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
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_ESDF_VOXFIELD_INTEGRATOR_H_

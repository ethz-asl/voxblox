#ifndef VOXBLOX_INTEGRATOR_ESDF_OCC_INTEGRATOR_H_
#define VOXBLOX_INTEGRATOR_ESDF_OCC_INTEGRATOR_H_

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
#include "voxblox/utils/timing.h"

namespace voxblox {

/**
 * Builds an ESDF layer out of a given occupancy layer.
 */
class EsdfOccIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Maximum distance to calculate the actual distance to.
     * Any values above this will be set to default_distance_m.
     */
    FloatingPoint max_distance_m = 2.0;
    /// Default distance set for unknown values and values > max_distance_m.
    FloatingPoint default_distance_m = 2.0;
    /// Number of buckets for the bucketed priority queue.
    int num_buckets = 20;
  };

  EsdfOccIntegrator(const Config& config, Layer<OccupancyVoxel>* occ_layer,
                    Layer<EsdfVoxel>* esdf_layer);

  /**
   * Fixed is overloaded as occupied in this case.
   * Only batch operations are currently supported for the occupancy map.
   */
  void updateFromOccLayerBatch();
  void updateFromOccBlocks(const BlockIndexList& occ_blocks);

  void processOpenSet();

  /**
   * Uses 26-connectivity and quasi-Euclidean distances.
   * Directions is the direction that the neighbor voxel lives in. If you
   * need the direction FROM the neighbor voxel TO the current voxel, take
   * negative of the given direction.
   */
  void getNeighborsAndDistances(
      const BlockIndex& block_index, const VoxelIndex& voxel_index,
      AlignedVector<VoxelKey>* neighbors, AlignedVector<float>* distances,
      AlignedVector<Eigen::Vector3i>* directions) const;
  void getNeighbor(const BlockIndex& block_index, const VoxelIndex& voxel_index,
                   const Eigen::Vector3i& direction,
                   BlockIndex* neighbor_block_index,
                   VoxelIndex* neighbor_voxel_index) const;

 protected:
  Config config_;

  Layer<OccupancyVoxel>* occ_layer_;
  Layer<EsdfVoxel>* esdf_layer_;

  /**
   * Open Queue for incremental updates. Contains global voxel indices
   * for the ESDF layer.
   */
  BucketQueue<VoxelKey> open_;

  /** Raise set for updates; these are values that used to be in the fixed
   * frontier and now have a higher value, or their children which need to have
   * their values invalidated.
   */
  AlignedQueue<VoxelKey> raise_;

  size_t esdf_voxels_per_side_;
  FloatingPoint esdf_voxel_size_;
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_ESDF_OCC_INTEGRATOR_H_

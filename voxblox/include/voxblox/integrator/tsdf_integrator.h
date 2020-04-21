#ifndef VOXBLOX_INTEGRATOR_TSDF_INTEGRATOR_H_
#define VOXBLOX_INTEGRATOR_TSDF_INTEGRATOR_H_

#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <glog/logging.h>
#include <Eigen/Core>

#include "voxblox/core/block_hash.h"
#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/utils/approx_hash_array.h"
#include "voxblox/utils/timing.h"

namespace voxblox {

enum class TsdfIntegratorType : int {
  kSimple = 1,
  kMerged = 2,
  kFast = 3,
};

static constexpr size_t kNumTsdfIntegratorTypes = 3u;

const std::array<std::string, kNumTsdfIntegratorTypes>
    kTsdfIntegratorTypeNames = {{/*kSimple*/ "simple",
                                 /*kMerged*/ "merged",
                                 /*kFast*/ "fast"}};

/**
 * Base class to the simple, merged and fast TSDF integrators. The integrator
 * takes in a pointcloud + pose and uses this information to update the TSDF
 * information in the given TSDF layer. Note most functions in this class state
 * if they are thread safe. Unless explicitly stated otherwise, this thread
 * safety is based on the assumption that any pointers passed to the functions
 * point to objects that are guaranteed to not be accessed by other threads.
 */
class TsdfIntegratorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<TsdfIntegratorBase> Ptr;

  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    float default_truncation_distance = 0.1;
    float max_weight = 10000.0;
    bool voxel_carving_enabled = true;
    FloatingPoint min_ray_length_m = 0.1;
    FloatingPoint max_ray_length_m = 5.0;
    bool use_const_weight = false;
    bool allow_clear = true;
    bool use_weight_dropoff = true;
    bool use_sparsity_compensation_factor = false;
    float sparsity_compensation_factor = 1.0f;

    size_t integrator_threads = std::thread::hardware_concurrency();

    /// Mode of the ThreadSafeIndex, determines the integration order of the
    /// rays. Options: "mixed", "sorted"
    std::string integration_order_mode = "mixed";

    /// merge integrator specific
    bool enable_anti_grazing = false;

    /// fast integrator specific
    float start_voxel_subsampling_factor = 2.0f;
    /// fast integrator specific
    int max_consecutive_ray_collisions = 2;
    /// fast integrator specific
    int clear_checks_every_n_frames = 1;
    /// fast integrator specific
    float max_integration_time_s = std::numeric_limits<float>::max();

    std::string print() const;
  };

  TsdfIntegratorBase(const Config& config, Layer<TsdfVoxel>* layer);

  /**
   * Integrates the given point infomation into the TSDF.
   * NOT thread safe.
   * @param freespace_points if true points will only be integrated up to the
   * truncation distance. Used when we are given a minimum distance to a point,
   * rather then exact distance. This is useful for clearing out free space.
   */
  virtual void integratePointCloud(const Transformation& T_G_C,
                                   const Pointcloud& points_C,
                                   const Colors& colors,
                                   const bool freespace_points = false) = 0;

  /// Returns a CONST ref of the config.
  const Config& getConfig() const { return config_; }

  void setLayer(Layer<TsdfVoxel>* layer);

 protected:
  /// Thread safe.
  inline bool isPointValid(const Point& point_C, const bool freespace_point,
                           bool* is_clearing) const {
    DCHECK(is_clearing != nullptr);
    const FloatingPoint ray_distance = point_C.norm();
    if (ray_distance < config_.min_ray_length_m) {
      return false;
    } else if (ray_distance > config_.max_ray_length_m) {
      if (config_.allow_clear || freespace_point) {
        *is_clearing = true;
        return true;
      } else {
        return false;
      }
    } else {
      *is_clearing = freespace_point;
      return true;
    }
  }

  /**
   * Will return a pointer to a voxel located at global_voxel_idx in the tsdf
   * layer. Thread safe.
   * Takes in the last_block_idx and last_block to prevent unneeded map lookups.
   * If this voxel belongs to a block that has not been allocated, a block in
   * temp_block_map_ is created/accessed and a voxel from this map is returned
   * instead. Unlike the layer, accessing temp_block_map_ is controlled via a
   * mutex allowing it to grow during integration.
   * These temporary blocks can be merged into the layer later by calling
   * updateLayerWithStoredBlocks
   */
  TsdfVoxel* allocateStorageAndGetVoxelPtr(const GlobalIndex& global_voxel_idx,
                                           Block<TsdfVoxel>::Ptr* last_block,
                                           BlockIndex* last_block_idx);

  /**
   * Merges temporarily stored blocks into the main layer. NOT thread safe, see
   * allocateStorageAndGetVoxelPtr for more details.
   */
  void updateLayerWithStoredBlocks();

  /// Updates tsdf_voxel, Thread safe.
  void updateTsdfVoxel(const Point& origin, const Point& point_G,
                       const GlobalIndex& global_voxel_index,
                       const Color& color, const float weight,
                       TsdfVoxel* tsdf_voxel);

  /// Calculates TSDF distance, Thread safe.
  float computeDistance(const Point& origin, const Point& point_G,
                        const Point& voxel_center) const;

  /// Thread safe.
  float getVoxelWeight(const Point& point_C) const;

  Config config_;

  Layer<TsdfVoxel>* layer_;

  // Cached map config.
  FloatingPoint voxel_size_;
  size_t voxels_per_side_;
  FloatingPoint block_size_;

  // Derived types.
  FloatingPoint voxel_size_inv_;
  FloatingPoint voxels_per_side_inv_;
  FloatingPoint block_size_inv_;

  std::mutex temp_block_mutex_;
  /**
   * Temporary block storage, used to hold blocks that need to be created while
   * integrating a new pointcloud
   */
  Layer<TsdfVoxel>::BlockHashMap temp_block_map_;

  /**
   * We need to prevent simultaneous access to the voxels in the map. We could
   * put a single mutex on the map or on the blocks, but as voxel updating is
   * the most expensive operation in integration and most voxels are close
   * together, both strategies would bottleneck the system. We could make a
   * mutex per voxel, but this is too ram heavy as one mutex = 40 bytes.
   * Because of this we create an array that is indexed by the first n bits of
   * the voxels hash. Assuming a uniform hash distribution, this means the
   * chance of two threads needing the same lock for unrelated voxels is
   * (num_threads / (2^n)). For 8 threads and 12 bits this gives 0.2%.
   */
  ApproxHashArray<12, std::mutex, GlobalIndex, LongIndexHash> mutexes_;
};

/// Creates a TSDF integrator of the desired type.
class TsdfIntegratorFactory {
 public:
  static TsdfIntegratorBase::Ptr create(
      const std::string& integrator_type_name,
      const TsdfIntegratorBase::Config& config, Layer<TsdfVoxel>* layer);
  static TsdfIntegratorBase::Ptr create(
      const TsdfIntegratorType integrator_type,
      const TsdfIntegratorBase::Config& config, Layer<TsdfVoxel>* layer);
};

/**
 * Basic TSDF integrator. Every point is raycast through all the voxels, which
 * are updated individually. An exact but very slow approach.
 */
class SimpleTsdfIntegrator : public TsdfIntegratorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SimpleTsdfIntegrator(const Config& config, Layer<TsdfVoxel>* layer)
      : TsdfIntegratorBase(config, layer) {}

  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C, const Colors& colors,
                           const bool freespace_points = false);

  void integrateFunction(const Transformation& T_G_C,
                         const Pointcloud& points_C, const Colors& colors,
                         const bool freespace_points,
                         ThreadSafeIndex* index_getter);
};

/**
 * Uses ray bundling to improve integration speed, points which lie in the same
 * voxel are "merged" into a single point. Raycasting and updating then proceeds
 * as normal. Fast for large voxels, with minimal loss of information.
 */
class MergedTsdfIntegrator : public TsdfIntegratorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MergedTsdfIntegrator(const Config& config, Layer<TsdfVoxel>* layer)
      : TsdfIntegratorBase(config, layer) {}

  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C, const Colors& colors,
                           const bool freespace_points = false);

 protected:
  void bundleRays(const Transformation& T_G_C, const Pointcloud& points_C,
                  const bool freespace_points, ThreadSafeIndex* index_getter,
                  LongIndexHashMapType<AlignedVector<size_t>>::type* voxel_map,
                  LongIndexHashMapType<AlignedVector<size_t>>::type* clear_map);

  void integrateVoxel(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
      const std::pair<GlobalIndex, AlignedVector<size_t>>& kv,
      const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map);

  void integrateVoxels(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
      const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map,
      const LongIndexHashMapType<AlignedVector<size_t>>::type& clear_map,
      size_t thread_idx);

  void integrateRays(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
      const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map,
      const LongIndexHashMapType<AlignedVector<size_t>>::type& clear_map);
};

/**
 * An integrator that prioritizes speed over everything else. Rays are cast from
 * the pointcloud to the sensor origin. If a ray intersects
 * max_consecutive_ray_collisions voxels in a row that have already been updated
 * by other rays from the same cloud, it is terminated early. This results in a
 * large reduction in the number of freespace updates and greatly improves
 * runtime while ensuring all voxels receive at least a minimum number of
 * updates. Speed is further enhanced through limiting the number of rays cast
 * from each voxel as set by start_voxel_subsampling_factor and use of the
 * ApproxHashSet. Up to an order of magnitude faster then the other integrators
 * for small voxels.
 */
class FastTsdfIntegrator : public TsdfIntegratorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FastTsdfIntegrator(const Config& config, Layer<TsdfVoxel>* layer)
      : TsdfIntegratorBase(config, layer) {}

  void integrateFunction(const Transformation& T_G_C,
                         const Pointcloud& points_C, const Colors& colors,
                         const bool freespace_points,
                         ThreadSafeIndex* index_getter);

  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C, const Colors& colors,
                           const bool freespace_points = false);

 private:
  /**
   * Two approximate sets are used below. The limitations of these sets are
   * outlined in approx_hash_array.h, but in brief they are thread safe and very
   * fast, but have a small chance of returning false positives and false
   * negatives. As rejecting a ray or integrating an uninformative ray are not
   * very harmful operations this trade-off works well in this integrator.
   */

  /**
   * uses 2^20 bytes (8 megabytes) of ram per tester
   * A testers false negative rate is inversely proportional to its size
   */
  static constexpr size_t masked_bits_ = 20;
  /**
   * only needs to zero the above 8mb of memory once every 10,000 scans
   * (uses an additional 80,000 bytes)
   */
  static constexpr size_t full_reset_threshold_ = 10000;

  /**
   * Voxel start locations are added to this set before ray casting. The ray
   * casting only occurs if no ray has been cast from this location for this
   * scan.
   */
  ApproxHashSet<masked_bits_, full_reset_threshold_, GlobalIndex, LongIndexHash>
      start_voxel_approx_set_;

  /**
   * This set records which voxels a scans rays have passed through. If a ray
   * moves through max_consecutive_ray_collisions voxels in a row that have
   * already been seen this scan, it is deemed to be adding no new information
   * and the casting stops.
   */
  ApproxHashSet<masked_bits_, full_reset_threshold_, GlobalIndex, LongIndexHash>
      voxel_observed_approx_set_;

  /// Used in terminating the integration early if it exceeds a time limit.
  std::chrono::time_point<std::chrono::steady_clock> integration_start_time_;
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_TSDF_INTEGRATOR_H_

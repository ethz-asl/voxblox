#ifndef VOXBLOX_INTEGRATOR_TSDF_INTEGRATOR_H_
#define VOXBLOX_INTEGRATOR_TSDF_INTEGRATOR_H_

#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <limits>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>
#include <vector>

#include <glog/logging.h>
#include <Eigen/Core>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/utils/approx_hash_array.h"
#include "voxblox/utils/timing.h"

#include "voxblox/core/block_hash.h"

namespace voxblox {

// Note most functions state if they are thread safe. Unless explicitly stated
// otherwise, this thread safety is based on the assumption that any pointers
// passed to the functions point to objects that are guaranteed to not be
// accessed by other threads.
class TsdfIntegratorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef AnyIndexHashMapType<TsdfVoxel>::type VoxelMap;

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

    // merge integrator specific
    bool enable_anti_grazing = false;

    // fast integrator specific
    float start_voxel_subsampling_factor = 2.0f;
    int max_consecutive_ray_collisions = 2;
  };

  TsdfIntegratorBase(const Config& config, Layer<TsdfVoxel>* layer);

  // NOT thread safe.
  virtual void integratePointCloud(const Transformation& T_G_C,
                                   const Pointcloud& points_C,
                                   const Colors& colors) = 0;

  // Returns a CONST ref of the config.
  const Config& getConfig() const { return config_; }

 protected:
  // Thread safe.
  inline bool isPointValid(const Point& point_C, bool* is_clearing) const;

  // Will return a pointer to a voxel located at global_voxel_idx in the tsdf
  // layer. Thread safe.
  // Takes in the last_block_idx and last_block to prevent unneeded map lookups.
  // If the block this voxel would be in has not been allocated, a voxel in
  // temp_voxel_storage is allocated and returned instead.
  // This can be merged into the layer later by calling
  // updateLayerWithStoredVoxels(temp_voxel_storage)
  inline TsdfVoxel* findOrTempAllocateVoxelPtr(
      const VoxelIndex& global_voxel_idx, Block<TsdfVoxel>::Ptr* last_block,
      BlockIndex* last_block_idx, VoxelMap* temp_voxel_storage) const;

  // NOT thread safe
  inline void updateLayerWithStoredVoxels(const VoxelMap& temp_voxel_storage);

  // Updates tsdf_voxel. Thread safe.
  inline void updateTsdfVoxel(const Point& origin, const Point& point_G,
                              const Point& voxel_center, const Color& color,
                              const float weight, TsdfVoxel* tsdf_voxel);

  // Thread safe.
  inline float computeDistance(const Point& origin, const Point& point_G,
                               const Point& voxel_center) const;

  // Thread safe.
  inline float getVoxelWeight(const Point& point_C) const;

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

  // We need to prevent simultaneous access to the voxels in the map. We could
  // put a single mutex on the map or on the blocks, but as voxel updating is
  // the most expensive operation in integration and most voxels are close
  // together, both strategies would bottleneck the system. We could make a
  // mutex per voxel, but this is too ram heavy as one mutex = 40 bytes.
  // Because of this we create an array that is indexed by the first n bits of
  // the voxels hash. Assuming a uniform hash distribution, this means the
  // chance of two threads needing the same lock for unrelated voxels is
  // (num_threads / (2^n)). For 8 threads and 12 bits this gives 0.2%.
  ApproxHashArray<12, std::mutex> mutexes_;
};

class SimpleTsdfIntegrator : public TsdfIntegratorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SimpleTsdfIntegrator(const Config& config, Layer<TsdfVoxel>* layer)
      : TsdfIntegratorBase(config, layer) {}

  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C, const Colors& colors);

  void integrateFunction(const Transformation& T_G_C,
                         const Pointcloud& points_C, const Colors& colors,
                         ThreadSafeIndex* index_getter,
                         VoxelMap* temp_voxel_storage);
};

class MergedTsdfIntegrator : public TsdfIntegratorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MergedTsdfIntegrator(const Config& config, Layer<TsdfVoxel>* layer)
      : TsdfIntegratorBase(config, layer) {}

  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C, const Colors& colors);

 private:
  inline void bundleRays(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, ThreadSafeIndex* index_getter,
      AnyIndexHashMapType<std::vector<size_t>>::type* voxel_map,
      AnyIndexHashMapType<std::vector<size_t>>::type* clear_map);

  void integrateVoxel(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
      const std::pair<AnyIndex, std::vector<size_t>>& kv,
      const AnyIndexHashMapType<std::vector<size_t>>::type& voxel_map,
      VoxelMap* temp_voxel_storage);

  void integrateVoxels(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
      const AnyIndexHashMapType<std::vector<size_t>>::type& voxel_map,
      const AnyIndexHashMapType<std::vector<size_t>>::type& clear_map,
      size_t thread_idx, VoxelMap* temp_voxel_storage);

  void integrateRays(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
      const AnyIndexHashMapType<std::vector<size_t>>::type& voxel_map,
      const AnyIndexHashMapType<std::vector<size_t>>::type& clear_map);
};

class FastTsdfIntegrator : public TsdfIntegratorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FastTsdfIntegrator(const Config& config, Layer<TsdfVoxel>* layer)
      : TsdfIntegratorBase(config, layer) {}

  void integrateFunction(const Transformation& T_G_C,
                         const Pointcloud& points_C, const Colors& colors,
                         ThreadSafeIndex* index_getter,
                         VoxelMap* temp_voxel_storage);

  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C, const Colors& colors);

 private:
  // Two approximate sets are used below. The limitations of these sets are
  // outlined in approx_hash_array.h, but in brief they are thread safe and very
  // fast, but have a small chance of returning false positives and false
  // negatives. As rejecting a ray or integrating an uninformative ray are not
  // very harmful operations this trade-off works well in this integrator.

  // uses 2^20 bytes (8 megabytes) of ram per tester
  // A testers false negative rate is inversely proportional to its size
  static constexpr size_t masked_bits_ = 20;
  // only needs to zero the above 8mb of memory once every 10,000 scans
  // (uses an additional 80,000 bytes)
  static constexpr size_t full_reset_threshold = 10000;

  // Voxel start locations are added to this set before ray casting. The ray
  // casting only occurs if no ray has been cast from this location for this
  // scan.
  ApproxHashSet<masked_bits_, full_reset_threshold> start_voxel_approx_set_;
  // This set records which voxels a scans rays have passed through. If a ray
  // moves through max_consecutive_ray_collisions voxels in a row that have
  // already been seen this scan, it is deemed to be adding no new information
  // and the casting stops.
  ApproxHashSet<masked_bits_, full_reset_threshold> voxel_observed_approx_set_;
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_TSDF_INTEGRATOR_H_

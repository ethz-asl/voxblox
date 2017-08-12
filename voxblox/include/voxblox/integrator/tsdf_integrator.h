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
// otherwise, this thread saftey is based on the assumption that any pointers
// passed to the functions point to objects that are guaranteed to not be
// accessed by other threads.
class TsdfIntegratorBase {
 public:
  typedef BlockHashMapType<TsdfVoxel>::type VoxelMap;

  struct Config {
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
    // only implemented in the merge integrator
    bool enable_anti_grazing = false;
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
  // layer. Thread safe
  // If the block this voxel would be in has not been allocated, a voxel in
  // temp_voxel_storage is allocated and returned instead.
  // This can be merged into the layer later by calling
  // updateLayerWithStoredVoxels(temp_voxel_storage)
  inline TsdfVoxel* findOrTempAllocateVoxelPtr(
      const VoxelIndex& global_voxel_idx, VoxelMap* temp_voxel_storage) const;

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

  ApproxHashArray<12, std::mutex> mutexes_;  // 4096 locks
};

class SimpleTsdfIntegrator : public TsdfIntegratorBase {
 public:
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
  MergedTsdfIntegrator(const Config& config, Layer<TsdfVoxel>* layer)
      : TsdfIntegratorBase(config, layer) {}

  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C, const Colors& colors);

 private:
  inline void bundleRays(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, ThreadSafeIndex* index_getter,
      BlockHashMapType<std::vector<size_t>>::type* voxel_map,
      BlockHashMapType<std::vector<size_t>>::type* clear_map);

  void integrateVoxel(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
      const std::pair<AnyIndex, std::vector<size_t>>& kv,
      const BlockHashMapType<std::vector<size_t>>::type& voxel_map,
      VoxelMap* temp_voxel_storage);

  void integrateVoxels(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
      const BlockHashMapType<std::vector<size_t>>::type& voxel_map,
      const BlockHashMapType<std::vector<size_t>>::type& clear_map, size_t tid,
      VoxelMap* temp_voxel_storage);

  void integrateRays(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
      const BlockHashMapType<std::vector<size_t>>::type& voxel_map,
      const BlockHashMapType<std::vector<size_t>>::type& clear_map);
};

class FastTsdfIntegrator : public TsdfIntegratorBase {
 public:
  FastTsdfIntegrator(const Config& config, Layer<TsdfVoxel>* layer)
      : TsdfIntegratorBase(config, layer) {}

  void integrateFunction(const Transformation& T_G_C,
                         const Pointcloud& points_C, const Colors& colors,
                         ThreadSafeIndex* index_getter,
                         VoxelMap* temp_voxel_storage);

  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C, const Colors& colors);

 private:
  static constexpr FloatingPoint voxel_sub_sample_ = 2.0f;
  static constexpr size_t max_consecutive_ray_collisions_ = 2;
  static constexpr size_t masked_bits_ = 20;  // 8 mb of ram per tester
  static constexpr size_t full_reset_threshold = 10000;
  ApproxHashSet<masked_bits_, full_reset_threshold> approx_start_tester_;
  ApproxHashSet<masked_bits_, full_reset_threshold> approx_ray_tester_;
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_TSDF_INTEGRATOR_H_

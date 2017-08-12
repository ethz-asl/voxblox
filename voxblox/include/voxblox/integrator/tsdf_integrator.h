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

  TsdfIntegratorBase(const Config& config, Layer<TsdfVoxel>* layer)
      : config_(config), layer_(layer) {
    DCHECK(layer_);

    voxel_size_ = layer_->voxel_size();
    block_size_ = layer_->block_size();
    voxels_per_side_ = layer_->voxels_per_side();

    voxel_size_inv_ = 1.0 / voxel_size_;
    block_size_inv_ = 1.0 / block_size_;
    voxels_per_side_inv_ = 1.0 / voxels_per_side_;

    if (config_.integrator_threads == 0) {
      LOG(WARNING) << "Automatic core count failed, defaulting to 1 threads";
      config_.integrator_threads = 1;
    }
  }

  virtual void integratePointCloud(const Transformation& T_G_C,
                                   const Pointcloud& points_C,
                                   const Colors& colors) = 0;

  // Returns a CONST ref of the config.
  const Config& getConfig() const { return config_; }

 protected:
  bool isPointValid(const Point& point_C, bool* is_clearing) {
    const FloatingPoint ray_distance = point_C.norm();
    if (ray_distance < config_.min_ray_length_m) {
      return false;
    } else if (ray_distance > config_.max_ray_length_m) {
      if (config_.allow_clear) {
        *is_clearing = true;
        return true;
      } else {
        return false;
      }
    } else {
      *is_clearing = false;
      return true;
    }
  }

  // Can be used to pre-allocate all the blocks needed by the integrator. This
  // allows thread-safe lockless block map access
  void allocateBlocks(const Transformation& T_G_C, const Pointcloud& points_C,
                      const Colors& colors) {
    ThreadSafeIndex index_getter(points_C.size(), config_.integrator_threads);
    size_t point_idx;
    while (index_getter.getNextIndex(&point_idx)) {
      const Point& point_C = points_C[point_idx];
      bool is_clearing;
      if (!isPointValid(point_C, &is_clearing)) {
        continue;
      }

      const Point origin = T_G_C.getPosition();
      const Point point_G = T_G_C * point_C;

      RayCaster ray_caster(origin, point_G, is_clearing,
                           config_.voxel_carving_enabled,
                           config_.max_ray_length_m, layer_->block_size_inv(),
                           config_.default_truncation_distance);

      BlockIndex block_idx;
      while (ray_caster.nextRayIndex(&block_idx)) {
        Block<TsdfVoxel>::Ptr block =
            layer_->allocateBlockPtrByIndex(block_idx);
        block->updated() = true;
      }
    }
  }

  inline TsdfVoxel* getVoxel(const VoxelIndex& global_voxel_idx,
                             Block<TsdfVoxel>::Ptr* block,
                             BlockIndex* last_block_idx,
                             VoxelMap* temp_voxel_storage) {
    BlockIndex block_idx = getBlockIndexFromGlobalVoxelIndex(
        global_voxel_idx, voxels_per_side_inv_);
    VoxelIndex local_voxel_idx =
        getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);

    if (block_idx != *last_block_idx) {
      *block = layer_->getBlockPtrByIndex(block_idx);
      *last_block_idx = block_idx;
      if (*block != nullptr) {
        (*block)->updated() = true;
      }
    }

    // If no block at this location currently exists, we allocate a temporary
    // voxel that will be merged into the map later
    if (*block == nullptr) {
      return &((*temp_voxel_storage)[global_voxel_idx]);
    } else {
      return &((*block)->getVoxelByVoxelIndex(local_voxel_idx));
    }
  }

  void updateLayerWithStoredVoxels(const VoxelMap& temp_voxel_storage) {
    BlockIndex last_block_idx;
    Block<TsdfVoxel>::Ptr block = nullptr;
    for (const std::pair<const VoxelIndex, TsdfVoxel>& temp_voxel :
         temp_voxel_storage) {
      BlockIndex block_idx = getBlockIndexFromGlobalVoxelIndex(
          temp_voxel.first, voxels_per_side_inv_);
      VoxelIndex local_voxel_idx =
          getLocalFromGlobalVoxelIndex(temp_voxel.first, voxels_per_side_);

      if ((block_idx != last_block_idx) || (block == nullptr)) {
        block = layer_->allocateBlockPtrByIndex(block_idx);
        block->updated() = true;
      }

      TsdfVoxel& base_voxel = block->getVoxelByVoxelIndex(local_voxel_idx);

      const float new_weight = base_voxel.weight + temp_voxel.second.weight;

      // it is possible that both voxels have weights very close to zero
      if (new_weight < kFloatEpsilon) {
        continue;
      }
      base_voxel.color = Color::blendTwoColors(
          base_voxel.color, base_voxel.weight, temp_voxel.second.color,
          temp_voxel.second.weight);
      base_voxel.distance =
          (base_voxel.distance * base_voxel.weight +
           temp_voxel.second.distance * temp_voxel.second.weight) /
          new_weight;

      base_voxel.weight = std::min(config_.max_weight, new_weight);
    }
  }

  // updates tsdf_voxel. thread safe
  inline void updateTsdfVoxel(const Point& origin, const Point& point_G,
                              const Point& voxel_center, const Color& color,
                              const float truncation_distance,
                              const float weight, TsdfVoxel* tsdf_voxel) {
    // Figure out whether the voxel is behind or in front of the surface.
    // To do this, project the voxel_center onto the ray from origin to point G.
    // Then check if the the magnitude of the vector is smaller or greater than
    // the original distance...
    Point v_voxel_origin = voxel_center - origin;
    Point v_point_origin = point_G - origin;

    FloatingPoint dist_G = v_point_origin.norm();
    // projection of a (v_voxel_origin) onto b (v_point_origin)
    FloatingPoint dist_G_V = v_voxel_origin.dot(v_point_origin) / dist_G;

    float sdf = static_cast<float>(dist_G - dist_G_V);

    float updated_weight = weight;
    // Compute updated weight in case we use weight dropoff. It's easier here
    // that in getVoxelWeight as here we have the actual SDF for the voxel
    // already computed.
    const FloatingPoint dropoff_epsilon = voxel_size_;
    if (config_.use_weight_dropoff && sdf < -dropoff_epsilon) {
      updated_weight = weight * (truncation_distance + sdf) /
                       (truncation_distance - dropoff_epsilon);
      updated_weight = std::max(updated_weight, 0.0f);
    }

    // Compute the updated weight in case we compensate for sparsity. By
    // multiplicating the weight of occupied areas (|sdf| < truncation distance)
    // by a factor, we prevent to easily fade out these areas with the free
    // space parts of other rays which pass through the corresponding voxels.
    // This can be useful for creating a TSDF map from sparse sensor data (e.g.
    // visual features from a SLAM system). By default, this option is disabled.
    if (config_.use_sparsity_compensation_factor) {
      if (std::abs(sdf) < config_.default_truncation_distance) {
        updated_weight *= config_.sparsity_compensation_factor;
      }
    }

    // Grab and lock the mutex responsible for this voxel
    std::lock_guard<std::mutex> lock(
        mutexes_.get(getGridIndexFromPoint(point_G, voxel_size_inv_)));

    const float new_weight = tsdf_voxel->weight + updated_weight;
    tsdf_voxel->color = Color::blendTwoColors(
        tsdf_voxel->color, tsdf_voxel->weight, color, updated_weight);
    const float new_sdf =
        (sdf * updated_weight + tsdf_voxel->distance * tsdf_voxel->weight) /
        new_weight;

    tsdf_voxel->distance = (new_sdf > 0.0)
                               ? std::min(truncation_distance, new_sdf)
                               : std::max(-truncation_distance, new_sdf);
    tsdf_voxel->weight = std::min(config_.max_weight, new_weight);
  }

  inline float computeDistance(const Point& origin, const Point& point_G,
                               const Point& voxel_center) {
    Point v_voxel_origin = voxel_center - origin;
    Point v_point_origin = point_G - origin;

    FloatingPoint dist_G = v_point_origin.norm();
    // projection of a (v_voxel_origin) onto b (v_point_origin)
    FloatingPoint dist_G_V = v_voxel_origin.dot(v_point_origin) / dist_G;

    float sdf = static_cast<float>(dist_G - dist_G_V);
    return sdf;
  }

  inline float getVoxelWeight(const Point& point_C) const {
    if (config_.use_const_weight) {
      return 1.0f;
    }
    FloatingPoint dist_z = std::abs(point_C.z());
    if (dist_z > kEpsilon) {
      return 1.0f / (dist_z * dist_z);
    }
    return 0.0f;
  }

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
                           const Pointcloud& points_C, const Colors& colors) {
    std::vector<VoxelMap> temp_voxel_storage(config_.integrator_threads);

    timing::Timer integrate_timer("integrate");

    ThreadSafeIndex index_getter(points_C.size(), config_.integrator_threads);

    std::vector<std::thread> integration_threads;
    for (size_t i = 0; i < config_.integrator_threads; ++i) {
      integration_threads.emplace_back(&SimpleTsdfIntegrator::integrateFunction,
                                       this, T_G_C, points_C, colors,
                                       &index_getter, &(temp_voxel_storage[i]));
    }

    for (std::thread& thread : integration_threads) {
      thread.join();
    }
    integrate_timer.Stop();

    timing::Timer insertion_timer("inserting_missed_voxels");
    for (const VoxelMap& temp_voxels : temp_voxel_storage) {
      updateLayerWithStoredVoxels(temp_voxels);
    }
    insertion_timer.Stop();
  }

  void integrateFunction(const Transformation& T_G_C,
                         const Pointcloud& points_C, const Colors& colors,
                         ThreadSafeIndex* index_getter,
                         VoxelMap* temp_voxel_storage) {
    size_t point_idx;
    while (index_getter->getNextIndex(&point_idx)) {
      const Point& point_C = points_C[point_idx];
      const Color& color = colors[point_idx];
      bool is_clearing;
      if (!isPointValid(point_C, &is_clearing)) {
        continue;
      }

      const Point origin = T_G_C.getPosition();
      const Point point_G = T_G_C * point_C;

      RayCaster ray_caster(origin, point_G, is_clearing,
                           config_.voxel_carving_enabled,
                           config_.max_ray_length_m, voxel_size_inv_,
                           config_.default_truncation_distance);

      BlockIndex last_block_idx = BlockIndex::Zero();
      Block<TsdfVoxel>::Ptr block;
      VoxelIndex global_voxel_idx;
      while (ray_caster.nextRayIndex(&global_voxel_idx)) {
        TsdfVoxel* voxel = getVoxel(global_voxel_idx, &block, &last_block_idx,
                                    temp_voxel_storage);
        if (voxel != nullptr) {
          const float weight = getVoxelWeight(point_C);
          const Point voxel_center_G =
              getCenterPointFromGridIndex(global_voxel_idx, voxel_size_);

          updateTsdfVoxel(origin, point_G, voxel_center_G, color,
                          config_.default_truncation_distance, weight, voxel);
        }
      }
    }
  }
};

class MergedTsdfIntegrator : public TsdfIntegratorBase {
 public:
  MergedTsdfIntegrator(const Config& config, Layer<TsdfVoxel>* layer)
      : TsdfIntegratorBase(config, layer) {}

  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C, const Colors& colors) {
    timing::Timer integrate_timer("integrate");

    // Pre-compute a list of unique voxels to end on.
    // Create a hashmap: VOXEL INDEX -> index in original cloud.
    BlockHashMapType<std::vector<size_t>>::type voxel_map;
    // This is a hash map (same as above) to all the indices that need to be
    // cleared.
    BlockHashMapType<std::vector<size_t>>::type clear_map;

    ThreadSafeIndex index_getter(points_C.size(), config_.integrator_threads);

    bundleRays(T_G_C, points_C, colors, &index_getter, &voxel_map, &clear_map);

    integrateRays(T_G_C, points_C, colors, config_.enable_anti_grazing, false,
                  voxel_map, clear_map);

    timing::Timer clear_timer("integrate/clear");

    integrateRays(T_G_C, points_C, colors, config_.enable_anti_grazing, true,
                  voxel_map, clear_map);

    clear_timer.Stop();

    integrate_timer.Stop();
  }

 private:
  inline void bundleRays(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, ThreadSafeIndex* index_getter,
      BlockHashMapType<std::vector<size_t>>::type* voxel_map,
      BlockHashMapType<std::vector<size_t>>::type* clear_map) {
    size_t point_idx;
    while (index_getter->getNextIndex(&point_idx)) {
      const Point& point_C = points_C[point_idx];
      bool is_clearing;
      if (!isPointValid(point_C, &is_clearing)) {
        continue;
      }

      const Point point_G = T_G_C * point_C;

      VoxelIndex voxel_index = getGridIndexFromPoint(point_G, voxel_size_inv_);

      if (is_clearing) {
        (*clear_map)[voxel_index].push_back(point_idx);
      } else {
        (*voxel_map)[voxel_index].push_back(point_idx);
      }
    }

    LOG(INFO) << "Went from " << points_C.size() << " points to "
              << voxel_map->size() << " raycasts  and " << clear_map->size()
              << " clear rays.";
  }

  void integrateVoxel(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
      const std::pair<AnyIndex, std::vector<size_t>>& kv,
      const BlockHashMapType<std::vector<size_t>>::type& voxel_map,
      VoxelMap* temp_voxel_storage) {
    if (kv.second.empty()) {
      return;
    }

    const Point& origin = T_G_C.getPosition();
    Color merged_color;
    Point merged_point_C = Point::Zero();
    FloatingPoint merged_weight = 0.0;

    for (const size_t pt_idx : kv.second) {
      const Point& point_C = points_C[pt_idx];
      const Color& color = colors[pt_idx];

      float point_weight = getVoxelWeight(point_C);
      merged_point_C =
          (merged_point_C * merged_weight + point_C * point_weight) /
          (merged_weight + point_weight);
      merged_color = Color::blendTwoColors(merged_color, merged_weight, color,
                                           point_weight);
      merged_weight += point_weight;

      // only take first point when clearing
      if (clearing_ray) {
        break;
      }
    }

    const Point merged_point_G = T_G_C * merged_point_C;

    RayCaster ray_caster(origin, merged_point_G, clearing_ray,
                         config_.voxel_carving_enabled,
                         config_.max_ray_length_m, voxel_size_inv_,
                         config_.default_truncation_distance, false);

    BlockIndex last_block_idx = BlockIndex::Zero();
    Block<TsdfVoxel>::Ptr block;

    VoxelIndex global_voxel_idx;
    while (ray_caster.nextRayIndex(&global_voxel_idx)) {
      if (enable_anti_grazing) {
        // Check if this one is already the the block hash map for this
        // insertion. Skip this to avoid grazing.
        if ((clearing_ray || global_voxel_idx != kv.first) &&
            voxel_map.find(global_voxel_idx) != voxel_map.end()) {
          continue;
        }
      }

      TsdfVoxel* voxel = getVoxel(global_voxel_idx, &block, &last_block_idx,
                                  temp_voxel_storage);
      if (voxel != nullptr) {
        const Point voxel_center_G =
            getCenterPointFromGridIndex(global_voxel_idx, voxel_size_);

        updateTsdfVoxel(origin, merged_point_G, voxel_center_G, merged_color,
                        config_.default_truncation_distance, merged_weight,
                        voxel);
      }
    }
  }

  void integrateVoxels(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
      const BlockHashMapType<std::vector<size_t>>::type& voxel_map,
      const BlockHashMapType<std::vector<size_t>>::type& clear_map, size_t tid,
      VoxelMap* temp_voxel_storage) {
    BlockHashMapType<std::vector<size_t>>::type::const_iterator it;
    size_t map_size;
    if (clearing_ray) {
      it = clear_map.begin();
      map_size = clear_map.size();
    } else {
      it = voxel_map.begin();
      map_size = voxel_map.size();
    }

    for (size_t i = 0; i < map_size; ++i) {
      if (((i + tid + 1) % config_.integrator_threads) == 0) {
        integrateVoxel(T_G_C, points_C, colors, enable_anti_grazing,
                       clearing_ray, *it, voxel_map, temp_voxel_storage);
      }
      ++it;
    }
  }

  void integrateRays(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
      const BlockHashMapType<std::vector<size_t>>::type& voxel_map,
      const BlockHashMapType<std::vector<size_t>>::type& clear_map) {
    const Point& origin = T_G_C.getPosition();

    std::vector<VoxelMap> temp_voxel_storage(config_.integrator_threads);

    // if only 1 thread just do function call, otherwise spawn threads
    if (config_.integrator_threads == 1) {
      integrateVoxels(T_G_C, points_C, colors, enable_anti_grazing,
                      clearing_ray, voxel_map, clear_map, 0,
                      &(temp_voxel_storage[0]));
    } else {
      std::vector<std::thread> integration_threads;
      for (size_t i = 0; i < config_.integrator_threads; ++i) {
        integration_threads.emplace_back(
            &MergedTsdfIntegrator::integrateVoxels, this, T_G_C, points_C,
            colors, enable_anti_grazing, clearing_ray, voxel_map, clear_map, i,
            &(temp_voxel_storage[i]));
      }

      for (std::thread& thread : integration_threads) {
        thread.join();
      }
    }

    timing::Timer insertion_timer("inserting_missed_voxels");
    for (const VoxelMap& temp_voxels : temp_voxel_storage) {
      updateLayerWithStoredVoxels(temp_voxels);
    }
    insertion_timer.Stop();
  }
};

class FastTsdfIntegrator : public TsdfIntegratorBase {
 public:
  FastTsdfIntegrator(const Config& config, Layer<TsdfVoxel>* layer)
      : TsdfIntegratorBase(config, layer) {}

  void integrateFunction(const Transformation& T_G_C,
                         const Pointcloud& points_C, const Colors& colors,
                         ThreadSafeIndex* index_getter,
                         VoxelMap* temp_voxel_storage) {
    size_t point_idx;
    while (index_getter->getNextIndex(&point_idx)) {
      const Point& point_C = points_C[point_idx];
      const Color& color = colors[point_idx];
      bool is_clearing;
      if (!isPointValid(point_C, &is_clearing)) {
        continue;
      }

      const Point origin = T_G_C.getPosition();
      const Point point_G = T_G_C * point_C;
      // immediately check if the voxel the point is in has already been ray
      // traced (saves time setting up ray tracer for already used points)
      AnyIndex global_voxel_idx =
          getGridIndexFromPoint(point_G, voxel_sub_sample_ * voxel_size_inv_);
      if (!approx_start_tester_.replaceHash(global_voxel_idx)) {
        continue;
      }

      RayCaster ray_caster(origin, point_G, is_clearing,
                           config_.voxel_carving_enabled,
                           config_.max_ray_length_m, voxel_size_inv_,
                           config_.default_truncation_distance, false);

      BlockIndex last_block_idx = BlockIndex::Zero();
      Block<TsdfVoxel>::Ptr block;

      size_t consecutive_ray_collisions = 0;
      while (ray_caster.nextRayIndex(&global_voxel_idx)) {
        // if all a ray is doing is follow in the path of another, stop casting
        if (!approx_ray_tester_.replaceHash(global_voxel_idx)) {
          ++consecutive_ray_collisions;
        } else {
          consecutive_ray_collisions = 0;
        }
        if (consecutive_ray_collisions > max_consecutive_ray_collisions_) {
          break;
        }

        TsdfVoxel* voxel = getVoxel(global_voxel_idx, &block, &last_block_idx,
                                    temp_voxel_storage);
        if (voxel != nullptr) {
          const float weight = getVoxelWeight(point_C);
          const Point voxel_center_G =
              getCenterPointFromGridIndex(global_voxel_idx, voxel_size_);

          updateTsdfVoxel(origin, point_G, voxel_center_G, color,
                          config_.default_truncation_distance, weight, voxel);
        }
      }
    }
  }

  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C, const Colors& colors) {
    std::vector<VoxelMap> temp_voxel_storage(config_.integrator_threads);

    timing::Timer integrate_timer("integrate");

    approx_ray_tester_.resetApproxSet();
    approx_start_tester_.resetApproxSet();

    ThreadSafeIndex index_getter(points_C.size(), config_.integrator_threads);

    std::vector<std::thread> integration_threads;
    for (size_t i = 0; i < config_.integrator_threads; ++i) {
      integration_threads.emplace_back(&FastTsdfIntegrator::integrateFunction,
                                       this, T_G_C, points_C, colors,
                                       &index_getter, &(temp_voxel_storage[i]));
    }

    for (std::thread& thread : integration_threads) {
      thread.join();
    }

    integrate_timer.Stop();

    timing::Timer insertion_timer("inserting_missed_voxels");
    for (const VoxelMap& temp_voxels : temp_voxel_storage) {
      updateLayerWithStoredVoxels(temp_voxels);
    }
    insertion_timer.Stop();
  }

 private:
  static constexpr FloatingPoint voxel_sub_sample_ = 2.0f;
  static constexpr size_t max_consecutive_ray_collisions_ = 4;
  static constexpr size_t masked_bits_ = 20;  // 8 mb of ram per tester
  static constexpr size_t full_reset_threshold = 10000;
  ApproxHashSet<masked_bits_, full_reset_threshold> approx_start_tester_;
  ApproxHashSet<masked_bits_, full_reset_threshold> approx_ray_tester_;
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_TSDF_INTEGRATOR_H_

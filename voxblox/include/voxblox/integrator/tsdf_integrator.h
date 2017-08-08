#ifndef INTEGRATOR_TSDF_INTEGRATOR_H_
#define INTEGRATOR_TSDF_INTEGRATOR_H_

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
  // thread safe method of getting the next point in a point cloud, also does
  // the
  // global transform
  class PointGetter {
   public:
    PointGetter(const Transformation& T_G_C, const Pointcloud& points_C,
                const Colors& colors, const size_t number_of_threads)
        : T_G_C_(T_G_C),
          points_C_(points_C),
          colors_(colors),
          origin_(T_G_C.getPosition()),
          number_of_threads_(number_of_threads),
          atomic_idx_(0) {
      DCHECK_EQ(points_C_.size(), colors_.size());
    }

    bool getNextPoint(Point* point_C, Point* point_G, Color* color,
                      Point* origin) {
      if (points_C_.size() <= atomic_idx_.load()) {
        return false;
      }

      *origin = origin_;

      const size_t pt_idx = getMixedIdx(atomic_idx_.fetch_add(1));
      *point_C = points_C_[pt_idx];
      *color = colors_[pt_idx];
      *point_G = T_G_C_ * *point_C;

      return true;
    }

   private:
    // Mixes up the order rays are given in so that each thread is working with
    // a
    // point that is far away from the other threads current points.
    size_t getMixedIdx(size_t base_idx) {
      const size_t number_of_points = points_C_.size();
      const size_t number_of_groups = number_of_threads_;
      const size_t points_per_group = number_of_points / number_of_groups;

      size_t group_num = base_idx % number_of_groups;
      size_t position_in_group = base_idx / number_of_groups;

      return group_num * points_per_group + position_in_group;
    }

    const Transformation& T_G_C_;
    const Pointcloud& points_C_;
    const Colors& colors_;
    const Point origin_;
    const size_t number_of_threads_;
    std::atomic<size_t> atomic_idx_;
  };

  // Can be used to pre-allocate all the blocks needed by the integrator. This
  // allows thread-safe lockless block map access
  void allocateBlocks(const Transformation& T_G_C, const Pointcloud& points_C) {
    const Point& origin = T_G_C.getPosition();
    IndexSet index_set;

    for (size_t pt_idx = 0; pt_idx < points_C.size(); ++pt_idx) {
      const Point& point_C = points_C[pt_idx];
      const Point point_G = T_G_C * point_C;

      // immediately check if the voxel the point is in has already been ray
      // traced (saves time setting up ray tracer for already used points)
      AnyIndex block_idx =
          getGridIndexFromPoint(point_G, layer_->block_size_inv());
      if (index_set.count(block_idx)) {
        continue;
      }

      const FloatingPoint ray_distance = (point_G - origin).norm();
      if (ray_distance < config_.min_ray_length_m) {
        continue;
      } else if (ray_distance > config_.max_ray_length_m) {
        // TODO(helenol): clear until max ray length instead.
        continue;
      }

      const FloatingPoint truncation_distance =
          config_.default_truncation_distance;

      const Ray unit_ray = (point_G - origin).normalized();

      const Point ray_end = point_G + unit_ray * truncation_distance;
      const Point ray_start = config_.voxel_carving_enabled
                                  ? origin
                                  : (point_G - unit_ray * truncation_distance);

      RayCaster ray_caster(ray_end * layer_->block_size_inv(),
                           ray_start * layer_->block_size_inv());

      while (ray_caster.nextRayIndex(&block_idx)) {
        if (!index_set.insert(block_idx).second) {
          break;
        }
      }
    }

    // ray termination strategy will miss some blocks that must be allocated
    // however, all these missed blocks will have neighbors that were not missed
    // so we add all the neighbors as well
    for (const AnyIndex& block_idx : index_set) {
      for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
          for (int z = -1; z <= 1; ++z) {
            Block<TsdfVoxel>::Ptr block =
                layer_->allocateBlockPtrByIndex(block_idx + AnyIndex(x, y, z));
            block->updated() = true;
          }
        }
      }
    }
  }

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

  std::shared_ptr<PointGetter> point_getter_;
  ApproxHashArray<12, std::mutex> mutexes_;  // 4096 locks
};

class SimpleTsdfIntegrator : public TsdfIntegratorBase {
 public:
  SimpleTsdfIntegrator(const Config& config, Layer<TsdfVoxel>* layer)
      : TsdfIntegratorBase(config, layer) {}

  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C, const Colors& colors) {
    point_getter_ = std::make_shared<PointGetter>(T_G_C, points_C, colors,
                                                  config_.integrator_threads);

    timing::Timer block_timer("block_allocation");
    allocateBlocks(T_G_C, points_C);
    block_timer.Stop();

    timing::Timer integrate_timer("integrate");

    std::vector<std::thread> integration_threads;
    for (size_t i = 0; i < config_.integrator_threads; ++i) {
      integration_threads.emplace_back(&SimpleTsdfIntegrator::integrateFunction,
                                       this);
    }

    for (std::thread& thread : integration_threads) {
      thread.join();
    }

    integrate_timer.Stop();
  }

  void integrateFunction() {
    Point point_C, point_G, origin;
    Color color;
    while (point_getter_->getNextPoint(&point_C, &point_G, &color, &origin)) {
      FloatingPoint ray_distance = (point_G - origin).norm();
      if (ray_distance < config_.min_ray_length_m) {
        continue;
      } else if (ray_distance > config_.max_ray_length_m) {
        // TODO(helenol): clear until max ray length instead.
        continue;
      }

      FloatingPoint truncation_distance = config_.default_truncation_distance;

      const Ray unit_ray = (point_G - origin).normalized();

      const Point ray_end = point_G + unit_ray * truncation_distance;
      const Point ray_start = config_.voxel_carving_enabled
                                  ? origin
                                  : (point_G - unit_ray * truncation_distance);

      const Point start_scaled = ray_start * voxel_size_inv_;
      const Point end_scaled = ray_end * voxel_size_inv_;

      IndexVector global_voxel_indices;
      castRay(start_scaled, end_scaled, &global_voxel_indices);

      BlockIndex last_block_idx = BlockIndex::Zero();
      Block<TsdfVoxel>::Ptr block;

      for (const AnyIndex& global_voxel_idx : global_voxel_indices) {
        BlockIndex block_idx = getBlockIndexFromGlobalVoxelIndex(
            global_voxel_idx, voxels_per_side_inv_);
        VoxelIndex local_voxel_idx =
            getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);

        if (!block || block_idx != last_block_idx) {
          block = layer_->getBlockPtrByIndex(block_idx);

          if (block == nullptr) {
            ROS_WARN_STREAM(
                "Block ("
                << block_idx.transpose()
                << ") was not pre-allocated, this should never happen");
            continue;
          }
          last_block_idx = block_idx;
        }

        const Point voxel_center_G =
            block->computeCoordinatesFromVoxelIndex(local_voxel_idx);
        TsdfVoxel& tsdf_voxel = block->getVoxelByVoxelIndex(local_voxel_idx);

        const float weight = getVoxelWeight(point_C);

        updateTsdfVoxel(origin, point_G, voxel_center_G, color,
                        truncation_distance, weight, &tsdf_voxel);
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
    DCHECK_EQ(points_C.size(), colors.size());
    timing::Timer integrate_timer("integrate");

    // Pre-compute a list of unique voxels to end on.
    // Create a hashmap: VOXEL INDEX -> index in original cloud.
    BlockHashMapType<std::vector<size_t>>::type voxel_map;
    // This is a hash map (same as above) to all the indices that need to be
    // cleared.
    BlockHashMapType<std::vector<size_t>>::type clear_map;

    bundleRays(T_G_C, points_C, &voxel_map, &clear_map);

    integrateRays(T_G_C, points_C, colors, config_.enable_anti_grazing, false,
                  voxel_map, clear_map);

    timing::Timer clear_timer("integrate/clear");

    integrateRays(T_G_C, points_C, colors, config_.enable_anti_grazing, true,
                  voxel_map, clear_map);

    clear_timer.Stop();

    integrate_timer.Stop();
  }

 private:
  struct VoxelInfo {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TsdfVoxel voxel;

    BlockIndex block_idx;
    VoxelIndex local_voxel_idx;

    Point point_C;
    Point point_G;
  };

  inline void bundleRays(
      const Transformation& T_G_C, const Pointcloud& points_C,
      BlockHashMapType<std::vector<size_t>>::type* voxel_map,
      BlockHashMapType<std::vector<size_t>>::type* clear_map) {
    for (size_t pt_idx = 0; pt_idx < points_C.size(); ++pt_idx) {
      const Point& point_C = points_C[pt_idx];
      const Point point_G = T_G_C * point_C;

      FloatingPoint ray_distance = (point_C).norm();
      if (ray_distance < config_.min_ray_length_m) {
        continue;
      } else if (config_.allow_clear &&
                 ray_distance > config_.max_ray_length_m) {
        VoxelIndex voxel_index =
            getGridIndexFromPoint(point_G, voxel_size_inv_);
        (*clear_map)[voxel_index].push_back(pt_idx);
        continue;
      }

      // Figure out what the end voxel is here.
      VoxelIndex voxel_index = getGridIndexFromPoint(point_G, voxel_size_inv_);
      (*voxel_map)[voxel_index].push_back(pt_idx);
    }

    LOG(INFO) << "Went from " << points_C.size() << " points to "
              << voxel_map->size() << " raycasts  and " << clear_map->size()
              << " clear rays.";
  }

  void updateVoxel(const VoxelInfo& voxel_info, const Point& origin) {
    static BlockIndex last_block_idx = BlockIndex::Zero();
    static Block<TsdfVoxel>::Ptr block;

    if (!block || voxel_info.block_idx != last_block_idx) {
      block = layer_->allocateBlockPtrByIndex(voxel_info.block_idx);
      block->updated() = true;
      last_block_idx = voxel_info.block_idx;
    }

    const Point voxel_center_G =
        block->computeCoordinatesFromVoxelIndex(voxel_info.local_voxel_idx);
    TsdfVoxel& tsdf_voxel =
        block->getVoxelByVoxelIndex(voxel_info.local_voxel_idx);

    updateTsdfVoxel(origin, voxel_info.point_G, voxel_center_G,
                    voxel_info.voxel.color, config_.default_truncation_distance,
                    voxel_info.voxel.weight, &tsdf_voxel);
  }

  void integrateVoxel(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
      const std::pair<AnyIndex, std::vector<size_t>>& kv,
      const BlockHashMapType<std::vector<size_t>>::type& voxel_map,
      std::queue<VoxelInfo>* voxel_update_queue) {
    if (kv.second.empty()) {
      return;
    }

    const Point& origin = T_G_C.getPosition();
    const Point voxel_center_offset(0.5, 0.5, 0.5);

    // stores all the information needed to update a map voxel
    VoxelInfo voxel_info;
    voxel_info.point_C = Point::Zero();
    voxel_info.voxel.weight = 0.0;

    for (const size_t pt_idx : kv.second) {
      const Point& point_C = points_C[pt_idx];
      const Color& color = colors[pt_idx];

      float point_weight = getVoxelWeight(point_C);
      voxel_info.point_C = (voxel_info.point_C * voxel_info.voxel.weight +
                            point_C * point_weight) /
                           (voxel_info.voxel.weight + point_weight);
      voxel_info.voxel.color = Color::blendTwoColors(
          voxel_info.voxel.color, voxel_info.voxel.weight, color, point_weight);
      voxel_info.voxel.weight += point_weight;

      // only take first point when clearing
      if (clearing_ray) {
        break;
      }
    }

    voxel_info.point_G = T_G_C * voxel_info.point_C;
    const Ray unit_ray = (voxel_info.point_G - origin).normalized();

    Point ray_end, ray_start;
    if (clearing_ray) {
      ray_end = origin + unit_ray * config_.max_ray_length_m;
      ray_start = origin;
    } else {
      ray_end =
          voxel_info.point_G + unit_ray * config_.default_truncation_distance;
      ray_start = config_.voxel_carving_enabled
                      ? origin
                      : (voxel_info.point_G -
                         unit_ray * config_.default_truncation_distance);
    }

    const Point start_scaled = ray_start * voxel_size_inv_;
    const Point end_scaled = ray_end * voxel_size_inv_;

    IndexVector global_voxel_index;
    timing::Timer cast_ray_timer("integrate/cast_ray");
    castRay(start_scaled, end_scaled, &global_voxel_index);
    cast_ray_timer.Stop();

    timing::Timer update_voxels_timer("integrate/update_voxels");

    for (const AnyIndex& global_voxel_idx : global_voxel_index) {
      if (enable_anti_grazing) {
        // Check if this one is already the the block hash map for this
        // insertion. Skip this to avoid grazing.
        if ((clearing_ray || global_voxel_idx != kv.first) &&
            voxel_map.find(global_voxel_idx) != voxel_map.end()) {
          continue;
        }
      }

      voxel_info.block_idx = getGridIndexFromPoint(
          global_voxel_idx.cast<FloatingPoint>(), voxels_per_side_inv_);

      voxel_info.local_voxel_idx =
          getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);

      voxel_update_queue->push(voxel_info);
    }
    update_voxels_timer.Stop();
  }

  void integrateVoxels(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
      const BlockHashMapType<std::vector<size_t>>::type& voxel_map,
      const BlockHashMapType<std::vector<size_t>>::type& clear_map,
      std::queue<VoxelInfo>* voxel_update_queue, size_t tid) {
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
                       clearing_ray, *it, voxel_map, voxel_update_queue);
      }
      ++it;
    }
  }

  void integrateRays(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
      const BlockHashMapType<std::vector<size_t>>::type& voxel_map,
      const BlockHashMapType<std::vector<size_t>>::type& clear_map) {
    std::vector<std::queue<VoxelInfo>> voxel_update_queues(
        config_.integrator_threads);

    const Point& origin = T_G_C.getPosition();

    // if only 1 thread just do function call, otherwise spawn threads
    if (config_.integrator_threads == 1) {
      integrateVoxels(T_G_C, points_C, colors, enable_anti_grazing,
                      clearing_ray, voxel_map, clear_map,
                      &(voxel_update_queues[0]), 0);
    } else {
      std::vector<std::thread> integration_threads;
      for (size_t i = 0; i < config_.integrator_threads; ++i) {
        integration_threads.emplace_back(
            &MergedTsdfIntegrator::integrateVoxels, this, T_G_C, points_C,
            colors, enable_anti_grazing, clearing_ray, voxel_map, clear_map,
            &(voxel_update_queues[i]), i);
      }

      for (std::thread& thread : integration_threads) {
        thread.join();
      }
    }
    for (std::queue<VoxelInfo>& voxel_update_queue : voxel_update_queues) {
      while (!voxel_update_queue.empty()) {
        updateVoxel(voxel_update_queue.front(), origin);
        voxel_update_queue.pop();
      }
    }
  }
};

class FastTsdfIntegrator : public TsdfIntegratorBase {
 public:
  FastTsdfIntegrator(const Config& config, Layer<TsdfVoxel>* layer)
      : TsdfIntegratorBase(config, layer) {}

  void integrateFunction() {
    Point point_C, point_G, origin;
    Color color;
    while (point_getter_->getNextPoint(&point_C, &point_G, &color, &origin)) {
      // immediately check if the voxel the point is in has already been ray
      // traced (saves time setting up ray tracer for already used points)
      AnyIndex global_voxel_idx =
          getGridIndexFromPoint(point_G, voxel_sub_sample_ * voxel_size_inv_);
      if (!approx_start_tester_.replaceHash(global_voxel_idx)) {
        continue;
      }

      const FloatingPoint ray_distance = (point_G - origin).norm();
      if (ray_distance < config_.min_ray_length_m) {
        continue;
      } else if (ray_distance > config_.max_ray_length_m) {
        // TODO(helenol): clear until max ray length instead.
        continue;
      }

      const FloatingPoint truncation_distance =
          config_.default_truncation_distance;

      const Ray unit_ray = (point_G - origin).normalized();

      const Point ray_end = point_G + unit_ray * truncation_distance;
      const Point ray_start = config_.voxel_carving_enabled
                                  ? origin
                                  : (point_G - unit_ray * truncation_distance);

      RayCaster ray_caster(ray_end * voxel_size_inv_,
                           ray_start * voxel_size_inv_);

      BlockIndex last_block_idx = BlockIndex::Zero();
      Block<TsdfVoxel>::Ptr tsdf_block;

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

        BlockIndex block_idx = getBlockIndexFromGlobalVoxelIndex(
            global_voxel_idx, voxels_per_side_inv_);
        VoxelIndex local_voxel_idx =
            getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);

        if (!tsdf_block || block_idx != last_block_idx) {
          tsdf_block = layer_->getBlockPtrByIndex(block_idx);

          if (tsdf_block == nullptr) {
            ROS_WARN_STREAM(
                "Block ("
                << block_idx.transpose()
                << ") was not pre-allocated, this should never happen");
            continue;
          }
          tsdf_block->updated() = true;
          last_block_idx = block_idx;
        }

        const Point voxel_center_G =
            tsdf_block->computeCoordinatesFromVoxelIndex(local_voxel_idx);
        TsdfVoxel& tsdf_voxel =
            tsdf_block->getVoxelByVoxelIndex(local_voxel_idx);

        const float weight = getVoxelWeight(point_C);

        updateTsdfVoxel(origin, point_G, voxel_center_G, color,
                        truncation_distance, weight, &tsdf_voxel);
      }
    }
  }

  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C, const Colors& colors) {
    point_getter_ = std::make_shared<PointGetter>(T_G_C, points_C, colors,
                                                  config_.integrator_threads);

    timing::Timer block_timer("block_allocation");
    allocateBlocks(T_G_C, points_C);
    block_timer.Stop();

    timing::Timer integrate_timer("integrate");

    approx_ray_tester_.resetApproxSet();
    approx_start_tester_.resetApproxSet();

    std::vector<std::thread> integration_threads;
    for (size_t i = 0; i < config_.integrator_threads; ++i) {
      integration_threads.emplace_back(&FastTsdfIntegrator::integrateFunction,
                                       this);
    }

    for (std::thread& thread : integration_threads) {
      thread.join();
    }

    integrate_timer.Stop();
  }

 private:
  static constexpr FloatingPoint voxel_sub_sample_ = 4.0f;
  static constexpr size_t max_consecutive_ray_collisions_ = 8;
  static constexpr size_t masked_bits_ = 20;  // 8 mb of ram per tester
  static constexpr size_t full_reset_threshold = 10000;
  ApproxHashSet<masked_bits_, full_reset_threshold> approx_start_tester_;
  ApproxHashSet<masked_bits_, full_reset_threshold> approx_ray_tester_;
};

}  // namespace voxblox

#endif  // INTEGRATOR_TSDF_INTEGRATOR_H_

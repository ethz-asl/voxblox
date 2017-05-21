#ifndef VOXBLOX_FAST_INTEGRATOR_TSDF_INTEGRATOR_H_
#define VOXBLOX_FAST_INTEGRATOR_TSDF_INTEGRATOR_H_

#include <algorithm>
#include <vector>
#include <iostream>
#include <queue>
#include <thread>
#include <utility>

#include <Eigen/Core>
#include <glog/logging.h>

#include "voxblox_fast/core/layer.h"
#include "voxblox_fast/core/voxel.h"
#include "voxblox_fast/integrator/integrator_utils.h"
#include "voxblox_fast/utils/timing.h"

namespace voxblox_fast {

class TsdfIntegrator {
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
    size_t integrator_threads = std::thread::hardware_concurrency();
  };

  struct VoxelInfo {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TsdfVoxel voxel;

    BlockIndex block_idx;
    VoxelIndex local_voxel_idx;

    Point point_C;
    Point point_G;
  };

  TsdfIntegrator(const Config& config, Layer<TsdfVoxel>* layer)
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

  float getVoxelWeight(const Point& point_C, const Point& point_G,
                       const Point& origin, const Point& voxel_center) const {
    if (config_.use_const_weight) {
      return 1.0;
    }
    FloatingPoint dist_z = std::abs(point_C.z());
    if (dist_z > 1e-6) {
      return 1.0 / (dist_z * dist_z);
    }
    return 0.0;
  }

  inline void updateTsdfVoxel(const Point& origin, const Point& point_C,
                              const Point& point_G, const Point& voxel_center,
                              const Color& color,
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

  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C, const Colors& colors) {
    DCHECK_EQ(points_C.size(), colors.size());
    timing::Timer integrate_timer("integrate");

    const Point& origin = T_G_C.getPosition();

    for (size_t pt_idx = 0; pt_idx < points_C.size(); ++pt_idx) {
      const Point& point_C = points_C[pt_idx];
      const Point point_G = T_G_C * point_C;
      const Color& color = colors[pt_idx];

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
      timing::Timer cast_ray_timer("integrate/cast_ray");
      castRay(start_scaled, end_scaled, &global_voxel_indices);
      cast_ray_timer.Stop();

      timing::Timer update_voxels_timer("integrate/update_voxels");

      BlockIndex last_block_idx = BlockIndex::Zero();
      Block<TsdfVoxel>::Ptr block;

      for (const AnyIndex& global_voxel_idx : global_voxel_indices) {
        BlockIndex block_idx = getBlockIndexFromGlobalVoxelIndex(
            global_voxel_idx, voxels_per_side_inv_);
        VoxelIndex local_voxel_idx =
            getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);

        if (local_voxel_idx.x() < 0) {
          local_voxel_idx.x() += voxels_per_side_;
        }
        if (local_voxel_idx.y() < 0) {
          local_voxel_idx.y() += voxels_per_side_;
        }
        if (local_voxel_idx.z() < 0) {
          local_voxel_idx.z() += voxels_per_side_;
        }

        if (!block || block_idx != last_block_idx) {
          block = layer_->allocateBlockPtrByIndex(block_idx);
          block->updated() = true;
          last_block_idx = block_idx;
        }

        const Point voxel_center_G =
            block->computeCoordinatesFromVoxelIndex(local_voxel_idx);
        TsdfVoxel& tsdf_voxel = block->getVoxelByVoxelIndex(local_voxel_idx);

        const float weight =
            getVoxelWeight(point_C, point_G, origin, voxel_center_G);
        updateTsdfVoxel(origin, point_C, point_G, voxel_center_G, color,
                        truncation_distance, weight, &tsdf_voxel);
      }
      update_voxels_timer.Stop();
    }
    integrate_timer.Stop();
  }

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

    updateTsdfVoxel(origin, voxel_info.point_C, voxel_info.point_G,
                    voxel_center_G, voxel_info.voxel.color,
                    config_.default_truncation_distance,
                    voxel_info.voxel.weight, &tsdf_voxel);
  }

  void integrateVoxel(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, bool discard, bool clearing_ray,
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

      float point_weight = getVoxelWeight(
          point_C, T_G_C * point_C, origin,
          (kv.first.cast<FloatingPoint>() + voxel_center_offset) * voxel_size_);
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
      if (discard) {
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
      const Colors& colors, bool discard, bool clearing_ray,
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
        integrateVoxel(T_G_C, points_C, colors, discard, clearing_ray, *it,
                       voxel_map, voxel_update_queue);
      }
      ++it;
    }
  }

  void integrateRays(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, bool discard, bool clearing_ray,
      const BlockHashMapType<std::vector<size_t>>::type& voxel_map,
      const BlockHashMapType<std::vector<size_t>>::type& clear_map) {
    std::vector<std::queue<VoxelInfo>> voxel_update_queues(
        config_.integrator_threads);

    const Point& origin = T_G_C.getPosition();

    // if only 1 thread just do function call, otherwise spawn threads
    if (config_.integrator_threads == 1) {
      integrateVoxels(T_G_C, points_C, colors, discard, clearing_ray, voxel_map,
                      clear_map, &(voxel_update_queues[0]), 0);
    } else {
      std::vector<std::thread> integration_threads;
      for (size_t i = 0; i < config_.integrator_threads; ++i) {
        integration_threads.emplace_back(&TsdfIntegrator::integrateVoxels, this,
                                         T_G_C, points_C, colors, discard,
                                         clearing_ray, voxel_map, clear_map,
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

  void integratePointCloudMerged(const Transformation& T_G_C,
                                 const Pointcloud& points_C,
                                 const Colors& colors, bool discard) {
    DCHECK_EQ(points_C.size(), colors.size());
    timing::Timer integrate_timer("integrate");

    // Pre-compute a list of unique voxels to end on.
    // Create a hashmap: VOXEL INDEX -> index in original cloud.
    BlockHashMapType<std::vector<size_t>>::type voxel_map;
    // This is a hash map (same as above) to all the indices that need to be
    // cleared.
    BlockHashMapType<std::vector<size_t>>::type clear_map;

    bundleRays(T_G_C, points_C, &voxel_map, &clear_map);

    integrateRays(T_G_C, points_C, colors, discard, false, voxel_map,
                  clear_map);

    timing::Timer clear_timer("integrate/clear");

    integrateRays(T_G_C, points_C, colors, discard, true, voxel_map, clear_map);

    clear_timer.Stop();

    integrate_timer.Stop();
  }

  // Returns a CONST ref of the config.
  const Config& getConfig() const { return config_; }

 protected:
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
};

}  // namespace voxblox

#endif  // VOXBLOX_FAST_INTEGRATOR_TSDF_INTEGRATOR_H_

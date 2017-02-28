#ifndef VOXBLOX_INTEGRATOR_TSDF_INTEGRATOR_H_
#define VOXBLOX_INTEGRATOR_TSDF_INTEGRATOR_H_

#include <algorithm>
#include <vector>

#include <glog/logging.h>
#include <Eigen/Core>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/utils/timing.h"

namespace voxblox {

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
    Point voxel_direction = point_G - voxel_center;
    Point ray_direction = point_G - origin;

    float sdf = static_cast<float>(voxel_direction.norm());
    // Figure out if it's in front of the plane or behind.
    if (voxel_direction.dot(ray_direction) < 0) {
      sdf = -sdf;
    }

    const float new_weight = tsdf_voxel->weight + weight;
    tsdf_voxel->color = Color::blendTwoColors(
        tsdf_voxel->color, tsdf_voxel->weight, color, weight);
    const float new_sdf =
        (sdf * weight + tsdf_voxel->distance * tsdf_voxel->weight) / new_weight;

    tsdf_voxel->distance = (new_sdf > 0.0)
                               ? std::min(truncation_distance, new_sdf)
                               : std::max(-truncation_distance, new_sdf);
    tsdf_voxel->weight = std::min(config_.max_weight, new_weight);
  }

  inline float computeDistance(const Point& origin, const Point& point_G,
                               const Point& voxel_center) {
    Point voxel_direction = point_G - voxel_center;
    Point ray_direction = point_G - origin;

    float sdf = static_cast<float>(voxel_direction.norm());
    // Figure out if it's in front of the plane or behind.
    if (voxel_direction.dot(ray_direction) < 0) {
      sdf = -sdf;
    }
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

  void integrateVoxel(const Transformation& T_G_C, const Pointcloud& points_C,
                      const Colors& colors, bool discard, bool clearing_ray,
                      const std::pair<AnyIndex, std::vector<size_t>>& kv,
                      BlockHashMapType<std::vector<size_t>>::type& voxel_map) {
    if (kv.second.empty()) {
      return;
    }

    const Point& origin = T_G_C.getPosition();
    const Point voxel_center_offset(0.5, 0.5, 0.5);

    //stores all the information needed to update a map voxel
    VoxelInfo voxel_info;
    voxel_info.point_C = Point::Zero();
    voxel_info.voxel.weight = 0.0;

    for (const size_t pt_idx : kv.second) {
      const Point& point_C = points_C[pt_idx];
      const Color& color = colors[pt_idx];

      float point_weight = getVoxelWeight(
          point_C, T_G_C * point_C, origin,
          (kv.first.cast<FloatingPoint>() + voxel_center_offset) * voxel_size_);
      voxel_info.point_C = (voxel_info.point_C * voxel_info.weight + point_C * point_weight) /
                     (voxel_info.weight + point_weight);
      voxel_info.color =
          Color::blendTwoColors(voxel_info.color, voxel_info.weight, color, point_weight);
      voxel_info.weight += point_weight;

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
      ray_end = voxel_info.point_G + unit_ray * config_.default_truncation_distance;
      ray_start =
          config_.voxel_carving_enabled
              ? origin
              : (voxel_info.point_G - unit_ray * config_.default_truncation_distance);
    }

    const Point start_scaled = ray_start * voxel_size_inv_;
    const Point end_scaled = ray_end * voxel_size_inv_;

    IndexVector global_voxel_index;
    timing::Timer cast_ray_timer("integrate/cast_ray");
    castRay(start_scaled, end_scaled, &global_voxel_index);
    cast_ray_timer.Stop();

    timing::Timer update_voxels_timer("integrate/update_voxels");

    BlockIndex last_block_idx = BlockIndex::Zero();
    Block<TsdfVoxel>::Ptr block;

    for (const AnyIndex& global_voxel_idx : global_voxel_index) {
      if (discard) {
        // Check if this one is already the the block hash map for this
        // insertion. Skip this to avoid grazing.
        if ((clearing_ray || global_voxel_idx != kv.first) &&
            voxel_map.find(global_voxel_idx) != voxel_map.end()) {
          continue;
        }
      }

      BlockIndex block_idx = getGridIndexFromPoint(
          global_voxel_idx.cast<FloatingPoint>(), voxels_per_side_inv_);

      VoxelIndex local_voxel_idx =
          getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);

      if (!block || block_idx != last_block_idx) {
        block = layer_->allocateBlockPtrByIndex(block_idx);
        block->updated() = true;
        last_block_idx = block_idx;
      }

      const Point voxel_center_G =
          block->computeCoordinatesFromVoxelIndex(local_voxel_idx);
      TsdfVoxel& tsdf_voxel = block->getVoxelByVoxelIndex(local_voxel_idx);

      updateTsdfVoxel(origin, voxel_info.point_C, voxel_info.point_G, voxel_center_G, voxel_info.color,
                      config_.default_truncation_distance, voxel_info.weight,
                      &tsdf_voxel);

    }
    update_voxels_timer.Stop();
  }

  void updateVoxel(const VoxelInfo& voxel_info, const Point& origin){
    static BlockIndex last_block_idx = BlockIndex::Zero();
    static Block<TsdfVoxel>::Ptr block;

    if (!block || block_idx != last_block_idx) {
        block = layer_->allocateBlockPtrByIndex(block_idx);
        block->updated() = true;
        last_block_idx = block_idx;
      }

      const Point voxel_center_G =
          block->computeCoordinatesFromVoxelIndex(local_voxel_idx);
      TsdfVoxel& tsdf_voxel = block->getVoxelByVoxelIndex(local_voxel_idx);

      updateTsdfVoxel(origin, voxel_info.point_C, voxel_info.point_G, voxel_center_G, voxel_info.color,
                      config_.default_truncation_distance, voxel_info.weight,
                      &tsdf_voxel);
  }

  void integratePointCloudMerged(const Transformation& T_G_C,
                                 const Pointcloud& points_C,
                                 const Colors& colors, bool discard) {
    DCHECK_EQ(points_C.size(), colors.size());
    timing::Timer integrate_timer("integrate");

    const Point& origin = T_G_C.getPosition();

    // Pre-compute a list of unique voxels to end on.
    // Create a hashmap: VOXEL INDEX -> index in original cloud.
    BlockHashMapType<std::vector<size_t>>::type voxel_map;
    // This is a hash map (same as above) to all the indices that need to be
    // cleared.
    BlockHashMapType<std::vector<size_t>>::type clear_map;

    bundleRays(T_G_C, points_C, &voxel_map, &clear_map);

    const Point voxel_center_offset(0.5, 0.5, 0.5);

    for (const BlockHashMapType<std::vector<size_t>>::type::value_type& kv :
         voxel_map) {
      integrateVoxel(T_G_C, points_C, colors, discard, false, kv, voxel_map);
    }

    timing::Timer clear_timer("integrate/clear");
    BlockIndex last_block_idx = BlockIndex::Zero();
    Block<TsdfVoxel>::Ptr block;
    for (const BlockHashMapType<std::vector<size_t>>::type::value_type& kv :
         clear_map) {
      integrateVoxel(T_G_C, points_C, colors, discard, true, kv, voxel_map);
    }
    clear_timer.Stop();

    integrate_timer.Stop();
  }

  // Returns a CONST ref of the config.
  const Config& getConfig() const { return config_; }

 protected:

  struct VoxelInfo{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TsdfVoxel voxel;

    BlockIndex block_idx;
    VoxelIndex local_voxel_idx;

    Point point_C;
    Point point_G;
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
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_TSDF_INTEGRATOR_H_

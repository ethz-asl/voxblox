#ifndef VOXBLOX_INTEGRATOR_TSDF_INTEGRATOR_H_
#define VOXBLOX_INTEGRATOR_TSDF_INTEGRATOR_H_

#include <algorithm>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>

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

  // Temporary structure for containing all the info needed to do a TSDF update
  // for pre-filter update.
  struct TsdfUpdate {
    float distance;
    Color color;
    float weight;
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
    Eigen::Vector3d voxel_direction = point_G - voxel_center;
    Eigen::Vector3d ray_direction = point_G - origin;

    float sdf = static_cast<float>(voxel_direction.norm());
    // Figure out if it's in front of the plane or behind.
    if (voxel_direction.dot(ray_direction) < 0) {
      sdf = -sdf;
    }

    // This is for the linear drop-off in confidence behind the surface.
    float updated_weight = weight;
    /*if (!config_.use_const_weight && sdf < 0.0) {
      updated_weight =
          weight *
          std::max((truncation_distance + sdf) / truncation_distance, 0.0f);
    } */

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
    Eigen::Vector3d voxel_direction = point_G - voxel_center;
    Eigen::Vector3d ray_direction = point_G - origin;

    float sdf = static_cast<float>(voxel_direction.norm());
    // Figure out if it's in front of the plane or behind.
    if (voxel_direction.dot(ray_direction) < 0) {
      sdf = -sdf;
    }
    return sdf;
  }

  inline void updateVoxelFromVector(
      const std::vector<TsdfUpdate>& updates_const, TsdfVoxel* tsdf_voxel) {
    // Iterate over the vector of updates.
    // Copy over data...
    std::vector<TsdfUpdate> updates = updates_const;

    // Selection strategy: minimum.
    // TODO(helenol): look into other selection strategies.
    std::sort(updates.begin(), updates.end(),
              [](const TsdfUpdate& a, const TsdfUpdate& b) -> bool {
                return std::abs(a.distance) < std::abs(b.distance);
              });

    /* float weight = updates.front().weight;
    Color color = updates.front().color;
    float sdf = updates.front().distance; */
    float weight = 0.0;
    Color color;
    float sdf = 0.0;

    // Iterate over the entire vector.
    float truncation_distance = config_.default_truncation_distance;

    // bool occupied = false;
    size_t i = 0;
    for (const TsdfUpdate& update : updates) {
      if (i > 1) {
        break;
      }
      /* if (i > updates.size() / 2) {
        break;
      } */
      /* if (sdf < truncation_distance) {
        occupied = true;
      }
      if (occupied && sdf > truncation_distance) {
        break;
      } */
      ++i;
      sdf = (sdf * weight + update.distance * update.weight) /
            (weight + update.weight);
      color = Color::blendTwoColors(color, weight, update.color, update.weight);
      weight += update.weight;
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
    for (size_t pt_idx = 0; pt_idx < points_C.size(); ++pt_idx) {
      const Point& point_C = points_C[pt_idx];
      const Point point_G = T_G_C * point_C;

      FloatingPoint ray_distance = (point_C).norm();
      if (ray_distance < config_.min_ray_length_m) {
        continue;
      } else if (config_.allow_clear && ray_distance > config_.max_ray_length_m) {
        VoxelIndex voxel_index =
            getGridIndexFromPoint(point_G, voxel_size_inv_);
        clear_map[voxel_index].push_back(pt_idx);
        continue;
      }

      // Figure out what the end voxel is here.
      VoxelIndex voxel_index = getGridIndexFromPoint(point_G, voxel_size_inv_);
      voxel_map[voxel_index].push_back(pt_idx);
    }

    LOG(INFO) << "Went from " << points_C.size() << " points to "
              << voxel_map.size() << " raycasts  and " << clear_map.size()
              << " clear rays.";

    const Point voxel_center_offset(0.5, 0.5, 0.5);

    FloatingPoint truncation_distance = config_.default_truncation_distance;
    for (const BlockHashMapType<std::vector<size_t>>::type::value_type& kv :
         voxel_map) {
      if (kv.second.empty()) {
        continue;
      }
      // Key actually doesn't matter at all.
      Point mean_point_C = Point::Zero();
      Color mean_color;
      float total_weight = 0.0;

      for (size_t pt_idx : kv.second) {
        const Point& point_C = points_C[pt_idx];
        const Color& color = colors[pt_idx];

        float point_weight = getVoxelWeight(
            point_C, T_G_C * point_C, origin,
            (kv.first.cast<FloatingPoint>() + voxel_center_offset) *
                voxel_size_);
        mean_point_C = (mean_point_C * total_weight + point_C * point_weight) /
                       (total_weight + point_weight);
        mean_color = Color::blendTwoColors(mean_color, total_weight, color,
                                           point_weight);
        total_weight += point_weight;
      }

      const Point point_G = T_G_C * mean_point_C;
      const Ray unit_ray = (point_G - origin).normalized();
      const Point ray_end = point_G + unit_ray * truncation_distance;
      const Point ray_start = config_.voxel_carving_enabled
                                  ? origin
                                  : (point_G - unit_ray * truncation_distance);

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
          if (global_voxel_idx != kv.first &&
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

        updateTsdfVoxel(origin, mean_point_C, point_G, voxel_center_G,
                        mean_color, truncation_distance, total_weight,
                        &tsdf_voxel);
      }
      update_voxels_timer.Stop();
    }

    timing::Timer clear_timer("integrate/clear");
    BlockIndex last_block_idx = BlockIndex::Zero();
    Block<TsdfVoxel>::Ptr block;
    for (const BlockHashMapType<std::vector<size_t>>::type::value_type& kv :
         clear_map) {
      if (kv.second.empty()) {
        continue;
      }
      // Key actually doesn't matter at all.
      Point point_C = Point::Zero();
      Color color;
      float weight = 0.0;

      for (size_t pt_idx : kv.second) {
        // Just take first.
        point_C = points_C[pt_idx];
        color = colors[pt_idx];

        weight = getVoxelWeight(
            point_C, T_G_C * point_C, origin,
            (kv.first.cast<FloatingPoint>() + voxel_center_offset) *
                voxel_size_);
        break;
      }

      const Point point_G = T_G_C * point_C;
      const Ray unit_ray = (point_G - origin).normalized();
      const Point ray_end = origin + unit_ray * config_.max_ray_length_m;
      const Point ray_start = origin;

      const Point start_scaled = ray_start * voxel_size_inv_;
      const Point end_scaled = ray_end * voxel_size_inv_;

      IndexVector global_voxel_index;
      timing::Timer cast_ray_timer("integrate/cast_ray");
      castRay(start_scaled, end_scaled, &global_voxel_index);
      cast_ray_timer.Stop();

      for (const AnyIndex& global_voxel_idx : global_voxel_index) {
        if (discard) {
          // Check if this one is already the the block hash map for this
          // insertion. Skip this to avoid grazing.
          if (voxel_map.find(global_voxel_idx) != voxel_map.end()) {
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

        updateTsdfVoxel(origin, point_C, point_G, voxel_center_G, color,
                        truncation_distance, weight, &tsdf_voxel);
      }
    }
    clear_timer.Stop();

    integrate_timer.Stop();
  }

  // ICRA 2017 research...
  void integratePointCloudPrefilter(const Transformation& T_G_C,
                                    const Pointcloud& points_C,
                                    const Colors& colors) {
    DCHECK_EQ(points_C.size(), colors.size());
    timing::Timer integrate_timer("integrate");

    const Point& origin = T_G_C.getPosition();

    // Pre-compute a list of unique voxels to end on.
    // Create a hashmap: VOXEL INDEX -> index in original cloud.
    BlockHashMapType<std::vector<size_t>>::type voxel_map;
    // This is a hash map (same as above) to all the indices that need to be
    // cleared.
    BlockHashMapType<std::vector<size_t>>::type clear_map;
    for (size_t pt_idx = 0; pt_idx < points_C.size(); ++pt_idx) {
      const Point& point_C = points_C[pt_idx];
      const Point point_G = T_G_C * point_C;

      FloatingPoint ray_distance = (point_C).norm();
      if (ray_distance < config_.min_ray_length_m) {
        continue;
      } else if (ray_distance > config_.max_ray_length_m) {
        if (config_.voxel_carving_enabled) {
          VoxelIndex voxel_index =
              getGridIndexFromPoint(point_G, voxel_size_inv_);
          clear_map[voxel_index].push_back(pt_idx);
        }
        continue;
      }

      // Figure out what the end voxel is here.
      VoxelIndex voxel_index = getGridIndexFromPoint(point_G, voxel_size_inv_);
      voxel_map[voxel_index].push_back(pt_idx);
    }

    LOG(INFO) << "Went from " << points_C.size() << " points to "
              << voxel_map.size() << " raycasts and " << clear_map.size()
              << " clear rays.";

    const Point voxel_center_offset(0.5, 0.5, 0.5);

    // Now re-sort the voxels by the updates.
    // Type of the pair is {distance, color, weight}.
    BlockHashMapType<std::vector<TsdfUpdate>>::type voxel_update_map;

    timing::Timer prefilter_timer("integrate/prefilter");

    FloatingPoint truncation_distance = config_.default_truncation_distance;
    for (const BlockHashMapType<std::vector<size_t>>::type::value_type& kv :
         voxel_map) {
      if (kv.second.empty()) {
        continue;
      }
      // Key actually doesn't matter at all.
      Point mean_point_C = Point::Zero();
      Color mean_color;
      float total_weight = 0.0;

      for (size_t pt_idx : kv.second) {
        const Point& point_C = points_C[pt_idx];
        const Color& color = colors[pt_idx];

        float point_weight = getVoxelWeight(
            point_C, T_G_C * point_C, origin,
            (kv.first.cast<FloatingPoint>() + voxel_center_offset) *
                voxel_size_);
        mean_point_C = (mean_point_C * total_weight + point_C * point_weight) /
                       (total_weight + point_weight);
        mean_color = Color::blendTwoColors(mean_color, total_weight, color,
                                           point_weight);
        total_weight += point_weight;
      }

      const Point point_G = T_G_C * mean_point_C;
      const Ray unit_ray = (point_G - origin).normalized();
      const Point ray_end = point_G + unit_ray * truncation_distance;
      const Point ray_start = config_.voxel_carving_enabled
                                  ? origin
                                  : (point_G - unit_ray * truncation_distance);

      const Point start_scaled = ray_start * voxel_size_inv_;
      const Point end_scaled = ray_end * voxel_size_inv_;

      IndexVector global_voxel_index;
      timing::Timer cast_ray_timer("integrate/cast_ray");
      castRay(start_scaled, end_scaled, &global_voxel_index);
      cast_ray_timer.Stop();

      for (const AnyIndex& global_voxel_idx : global_voxel_index) {
        Point voxel_center_G =
            getOriginPointFromGridIndex(global_voxel_idx, voxel_size_);

        float sdf = computeDistance(origin, point_G, voxel_center_G);

        TsdfUpdate update;
        update.distance = sdf;
        update.weight = total_weight;
        // Behind the surface, should down-weigh.
        if (sdf < 0.0) {
          update.weight =
              total_weight *
              std::max((truncation_distance + sdf) / truncation_distance, 0.0);
        }
        /* std::cout << "Num points: " << kv.second.size()
                  << " weight: " << update.weight << " sdf: " << sdf
                  << " tr. dist.: " << truncation_distance << std::endl; */

        update.color = mean_color;

        voxel_update_map[global_voxel_idx].push_back(update);
      }
    }
    prefilter_timer.Stop();

    timing::Timer clear_timer("integrate/clear");
    for (const BlockHashMapType<std::vector<size_t>>::type::value_type& kv :
         clear_map) {
      if (kv.second.empty()) {
        continue;
      }
      // Key actually doesn't matter at all.
      Point point_C = Point::Zero();
      Color color;
      float weight = 0.0;

      for (size_t pt_idx : kv.second) {
        // Just take first.
        point_C = points_C[pt_idx];
        color = colors[pt_idx];

        weight = getVoxelWeight(
            point_C, T_G_C * point_C, origin,
            (kv.first.cast<FloatingPoint>() + voxel_center_offset) *
                voxel_size_);
        break;
      }

      const Point point_G = T_G_C * point_C;
      const Ray unit_ray = (point_G - origin).normalized();
      const Point ray_end = origin + unit_ray * config_.max_ray_length_m;
      const Point ray_start = origin;

      const Point start_scaled = ray_start * voxel_size_inv_;
      const Point end_scaled = ray_end * voxel_size_inv_;

      IndexVector global_voxel_index;
      timing::Timer cast_ray_timer("integrate/cast_ray");
      castRay(start_scaled, end_scaled, &global_voxel_index);
      cast_ray_timer.Stop();

      for (const AnyIndex& global_voxel_idx : global_voxel_index) {
        float sdf = truncation_distance;

        TsdfUpdate update;
        update.distance = sdf;
        update.weight = weight;
        update.color = color;

        voxel_update_map[global_voxel_idx].push_back(update);
      }
    }
    clear_timer.Stop();

    timing::Timer update_voxels_timer("integrate/update_voxels");
    BlockIndex last_block_idx = BlockIndex::Zero();
    Block<TsdfVoxel>::Ptr block;
    for (const BlockHashMapType<std::vector<TsdfUpdate>>::type::value_type& kv :
         voxel_update_map) {
      if (kv.second.empty()) {
        continue;
      }
      VoxelIndex global_voxel_idx = kv.first;
      BlockIndex block_idx = getGridIndexFromPoint(
          global_voxel_idx.cast<FloatingPoint>(), voxels_per_side_inv_);

      VoxelIndex local_voxel_idx =
          getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);

      if (!block || block_idx != last_block_idx) {
        block = layer_->allocateBlockPtrByIndex(block_idx);
        block->updated() = true;
        last_block_idx = block_idx;
      }
      TsdfVoxel& tsdf_voxel = block->getVoxelByVoxelIndex(local_voxel_idx);

      updateVoxelFromVector(kv.second, &tsdf_voxel);
    }

    update_voxels_timer.Stop();
    integrate_timer.Stop();
  }

  // ICRA 2017 research, part 210582350
  void integratePointCloudPrefilterFast(const Transformation& T_G_C,
                                        const Pointcloud& points_C,
                                        const Colors& colors) {
    DCHECK_EQ(points_C.size(), colors.size());
    timing::Timer integrate_timer("integrate");

    const Point& origin = T_G_C.getPosition();

    timing::Timer sort_timer("integrate/sort");

    // Pre-compute a list of unique voxels to end on.
    // Create a hashmap: VOXEL INDEX -> index in original cloud.
    BlockHashMapType<std::vector<size_t>>::type voxel_map;
    // This is a hash map (same as above) to all the indices that need to be
    // cleared.
    BlockHashMapType<std::vector<size_t>>::type clear_map;
    for (size_t pt_idx = 0; pt_idx < points_C.size(); ++pt_idx) {
      const Point& point_C = points_C[pt_idx];
      const Point point_G = T_G_C * point_C;

      FloatingPoint ray_distance = (point_C).norm();
      if (ray_distance < config_.min_ray_length_m) {
        continue;
      } else if (ray_distance > config_.max_ray_length_m) {
        if (config_.voxel_carving_enabled) {
          VoxelIndex voxel_index =
              getGridIndexFromPoint(point_G, voxel_size_inv_);
          clear_map[voxel_index].push_back(pt_idx);
        }
        continue;
      }

      // Figure out what the end voxel is here.
      VoxelIndex voxel_index = getGridIndexFromPoint(point_G, voxel_size_inv_);
      voxel_map[voxel_index].push_back(pt_idx);
    }
    sort_timer.Stop();

    LOG(INFO) << "Went from " << points_C.size() << " points to "
              << voxel_map.size() << " raycasts and " << clear_map.size()
              << " clear rays.";

    const Point voxel_center_offset(0.5, 0.5, 0.5);

    // Now re-sort the voxels by the updates.
    // Type of the pair is {distance, color, weight}.
    BlockHashMapType<std::vector<TsdfUpdate>>::type voxel_update_map;

    timing::Timer prefilter_timer("integrate/prefilter");

    FloatingPoint truncation_distance = config_.default_truncation_distance;
    for (const BlockHashMapType<std::vector<size_t>>::type::value_type& kv :
         voxel_map) {
      if (kv.second.empty()) {
        continue;
      }
      // Key actually doesn't matter at all.
      Point mean_point_C = Point::Zero();
      Color mean_color;
      float total_weight = 0.0;

      for (size_t pt_idx : kv.second) {
        const Point& point_C = points_C[pt_idx];
        const Color& color = colors[pt_idx];

        float point_weight = getVoxelWeight(
            point_C, T_G_C * point_C, origin,
            (kv.first.cast<FloatingPoint>() + voxel_center_offset) *
                voxel_size_);
        mean_point_C = (mean_point_C * total_weight + point_C * point_weight) /
                       (total_weight + point_weight);
        mean_color = Color::blendTwoColors(mean_color, total_weight, color,
                                           point_weight);
        total_weight += point_weight;
      }

      const Point point_G = T_G_C * mean_point_C;
      const Ray unit_ray = (point_G - origin).normalized();
      const Point ray_end = point_G + unit_ray * truncation_distance;
      const Point ray_start = config_.voxel_carving_enabled
                                  ? origin
                                  : (point_G - unit_ray * truncation_distance);

      const Point start_scaled = ray_start * voxel_size_inv_;
      const Point end_scaled = ray_end * voxel_size_inv_;

      IndexVector global_voxel_index;
      timing::Timer cast_ray_timer("integrate/cast_ray");
      castRay(start_scaled, end_scaled, &global_voxel_index);
      cast_ray_timer.Stop();

      for (const AnyIndex& global_voxel_idx : global_voxel_index) {
        Point voxel_center_G =
            getOriginPointFromGridIndex(global_voxel_idx, voxel_size_);

        float sdf = computeDistance(origin, point_G, voxel_center_G);

        TsdfUpdate update;
        update.distance = sdf;
        update.weight = getVoxelWeight(
            mean_point_C, T_G_C * mean_point_C, origin,
            (kv.first.cast<FloatingPoint>() + voxel_center_offset) *
                voxel_size_);

        // Behind the surface, should down-weigh.
        if (sdf < 0.0) {
          update.weight =
              update.weight *
              std::max((truncation_distance + sdf) / truncation_distance, 0.0);
        }
        /* std::cout << "Num points: " << kv.second.size()
                  << " weight: " << update.weight << " sdf: " << sdf
                  << " tr. dist.: " << truncation_distance << std::endl; */

        update.color = mean_color;

        voxel_update_map[global_voxel_idx].push_back(update);
      }
    }
    prefilter_timer.Stop();

    timing::Timer clear_timer("integrate/clear");
    for (const BlockHashMapType<std::vector<size_t>>::type::value_type& kv :
         clear_map) {
      if (kv.second.empty()) {
        continue;
      }
      // Key actually doesn't matter at all.
      Point point_C = Point::Zero();
      Color color;
      float weight = 0.0;

      for (size_t pt_idx : kv.second) {
        // Just take first.
        point_C = points_C[pt_idx];
        color = colors[pt_idx];

        weight = getVoxelWeight(
            point_C, T_G_C * point_C, origin,
            (kv.first.cast<FloatingPoint>() + voxel_center_offset) *
                voxel_size_);
        break;
      }

      const Point point_G = T_G_C * point_C;
      const Ray unit_ray = (point_G - origin).normalized();
      const Point ray_end = origin + unit_ray * config_.max_ray_length_m;
      const Point ray_start = origin;

      const Point start_scaled = ray_start * voxel_size_inv_;
      const Point end_scaled = ray_end * voxel_size_inv_;

      IndexVector global_voxel_index;
      timing::Timer cast_ray_timer("integrate/cast_ray");
      castRay(start_scaled, end_scaled, &global_voxel_index);
      cast_ray_timer.Stop();

      for (const AnyIndex& global_voxel_idx : global_voxel_index) {
        float sdf = truncation_distance;

        TsdfUpdate update;
        update.distance = sdf;
        update.weight = weight;
        update.color = color;

        voxel_update_map[global_voxel_idx].push_back(update);
      }
    }
    clear_timer.Stop();

    timing::Timer update_voxels_timer("integrate/update_voxels");
    BlockIndex last_block_idx = BlockIndex::Zero();
    Block<TsdfVoxel>::Ptr block;
    for (const BlockHashMapType<std::vector<TsdfUpdate>>::type::value_type& kv :
         voxel_update_map) {
      if (kv.second.empty()) {
        continue;
      }
      VoxelIndex global_voxel_idx = kv.first;
      BlockIndex block_idx = getGridIndexFromPoint(
          global_voxel_idx.cast<FloatingPoint>(), voxels_per_side_inv_);

      VoxelIndex local_voxel_idx =
          getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);

      if (!block || block_idx != last_block_idx) {
        block = layer_->allocateBlockPtrByIndex(block_idx);
        block->updated() = true;
        last_block_idx = block_idx;
      }
      TsdfVoxel& tsdf_voxel = block->getVoxelByVoxelIndex(local_voxel_idx);

      updateVoxelFromVector(kv.second, &tsdf_voxel);
    }

    update_voxels_timer.Stop();
    integrate_timer.Stop();
  }

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

#endif  // VOXBLOX_INTEGRATOR_TSDF_INTEGRATOR_H_

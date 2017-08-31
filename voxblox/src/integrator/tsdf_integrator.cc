#include "voxblox/integrator/tsdf_integrator.h"

namespace voxblox {

// Note many functions state if they are thread safe. Unless explicitly stated
// otherwise, this thread safety is based on the assumption that any pointers
// passed to the functions point to objects that are guaranteed to not be
// accessed by other threads.

TsdfIntegratorBase::TsdfIntegratorBase(const Config& config,
                                       Layer<TsdfVoxel>* layer)
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

// Thread safe.
inline bool TsdfIntegratorBase::isPointValid(const Point& point_C,
                                             bool* is_clearing) const {
  DCHECK(is_clearing != nullptr);
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

// Will return a pointer to a voxel located at global_voxel_idx in the tsdf
// layer. Thread safe.
// Takes in the last_block_idx and last_block to prevent unneeded map lookups.
// If the block this voxel would be in has not been allocated, a block in
// temp_block_map_ is created/accessed and a voxel from this map is returned
// instead. Unlike the layer, accessing temp_block_map_ is controlled via a
// mutex allowing it to grow during integration.
// These temporary blocks can be merged into the layer later by calling
// updateLayerWithStoredBlocks()
inline TsdfVoxel* TsdfIntegratorBase::allocateStorageAndGetVoxelPtr(
    const VoxelIndex& global_voxel_idx, Block<TsdfVoxel>::Ptr* last_block,
    BlockIndex* last_block_idx) {
  DCHECK(last_block != nullptr);
  DCHECK(last_block_idx != nullptr);

  const BlockIndex block_idx =
      getBlockIndexFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_inv_);

  if (block_idx != *last_block_idx) {
    *last_block = layer_->getBlockPtrByIndex(block_idx);
    *last_block_idx = block_idx;
  }

  // If no block at this location currently exists, we allocate a temporary
  // voxel that will be merged into the map later
  if (*last_block == nullptr) {
    // To allow temp_block_map_ to grow we can only let one thread in at once
    std::lock_guard<std::mutex> lock(temp_block_mutex_);

    typename Layer<TsdfVoxel>::BlockHashMap::iterator it =
        temp_block_map_.find(block_idx);
    if (it != temp_block_map_.end()) {
      *last_block = it->second;
    } else {
      auto insert_status = temp_block_map_.emplace(
          block_idx, std::make_shared<Block<TsdfVoxel>>(
                         voxels_per_side_, voxel_size_,
                         getOriginPointFromGridIndex(block_idx, block_size_)));

      DCHECK(insert_status.second) << "Block already exists when allocating at "
                                   << block_idx.transpose();

      *last_block = insert_status.first->second;
    }
  }

  (*last_block)->updated() = true;

  const VoxelIndex local_voxel_idx =
      getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);

  return &((*last_block)->getVoxelByVoxelIndex(local_voxel_idx));
}

// NOT thread safe
inline void TsdfIntegratorBase::updateLayerWithStoredBlocks() {
  BlockIndex last_block_idx;
  Block<TsdfVoxel>::Ptr block = nullptr;

  for (const std::pair<const BlockIndex, Block<TsdfVoxel>::Ptr>&
           temp_block_pair : temp_block_map_) {
    layer_->insertBlock(temp_block_pair);
  }

  temp_block_map_.clear();
}

// Updates tsdf_voxel. Thread safe.
inline void TsdfIntegratorBase::updateTsdfVoxel(
    const Point& origin, const Point& point_G, const Point& voxel_center,
    const Color& color, const float weight, TsdfVoxel* tsdf_voxel) {
  DCHECK(tsdf_voxel != nullptr);

  const float sdf = computeDistance(origin, point_G, voxel_center);

  float updated_weight = weight;
  // Compute updated weight in case we use weight dropoff. It's easier here
  // that in getVoxelWeight as here we have the actual SDF for the voxel
  // already computed.
  const FloatingPoint dropoff_epsilon = voxel_size_;
  if (config_.use_weight_dropoff && sdf < -dropoff_epsilon) {
    updated_weight = weight * (config_.default_truncation_distance + sdf) /
                     (config_.default_truncation_distance - dropoff_epsilon);
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

  // Lookup the mutex that is responsible for this voxel and lock it
  std::lock_guard<std::mutex> lock(
      mutexes_.get(getGridIndexFromPoint(point_G, voxel_size_inv_)));

  const float new_weight = tsdf_voxel->weight + updated_weight;

  // it is possible to have weights very close to zero, due to the limited
  // precision of floating points dividing by this small value can cause nans
  if (new_weight < kFloatEpsilon) {
    return;
  }

  const float new_sdf =
      (sdf * updated_weight + tsdf_voxel->distance * tsdf_voxel->weight) /
      new_weight;

  // color blending is expensive only do it close to the surface
  if (std::abs(sdf) < config_.default_truncation_distance) {
    tsdf_voxel->color = Color::blendTwoColors(
        tsdf_voxel->color, tsdf_voxel->weight, color, updated_weight);
  }

  tsdf_voxel->distance =
      (new_sdf > 0.0) ? std::min(config_.default_truncation_distance, new_sdf)
                      : std::max(-config_.default_truncation_distance, new_sdf);
  tsdf_voxel->weight = std::min(config_.max_weight, new_weight);
}

// Thread safe.
// Figure out whether the voxel is behind or in front of the surface.
// To do this, project the voxel_center onto the ray from origin to point G.
// Then check if the the magnitude of the vector is smaller or greater than
// the original distance...
inline float TsdfIntegratorBase::computeDistance(
    const Point& origin, const Point& point_G,
    const Point& voxel_center) const {
  const Point v_voxel_origin = voxel_center - origin;
  const Point v_point_origin = point_G - origin;

  const FloatingPoint dist_G = v_point_origin.norm();
  // projection of a (v_voxel_origin) onto b (v_point_origin)
  const FloatingPoint dist_G_V = v_voxel_origin.dot(v_point_origin) / dist_G;

  const float sdf = static_cast<float>(dist_G - dist_G_V);
  return sdf;
}

// Thread safe.
inline float TsdfIntegratorBase::getVoxelWeight(const Point& point_C) const {
  if (config_.use_const_weight) {
    return 1.0f;
  }
  const FloatingPoint dist_z = std::abs(point_C.z());
  if (dist_z > kEpsilon) {
    return 1.0f / (dist_z * dist_z);
  }
  return 0.0f;
}

void SimpleTsdfIntegrator::integratePointCloud(const Transformation& T_G_C,
                                               const Pointcloud& points_C,
                                               const Colors& colors) {
  timing::Timer integrate_timer("integrate");

  ThreadSafeIndex index_getter(points_C.size(), config_.integrator_threads);

  AlignedVector<std::thread> integration_threads;
  for (size_t i = 0; i < config_.integrator_threads; ++i) {
    integration_threads.emplace_back(&SimpleTsdfIntegrator::integrateFunction,
                                     this, T_G_C, points_C, colors,
                                     &index_getter);
  }

  for (std::thread& thread : integration_threads) {
    thread.join();
  }
  integrate_timer.Stop();

  timing::Timer insertion_timer("inserting_missed_blocks");
  updateLayerWithStoredBlocks();
  insertion_timer.Stop();
}

void SimpleTsdfIntegrator::integrateFunction(const Transformation& T_G_C,
                                             const Pointcloud& points_C,
                                             const Colors& colors,
                                             ThreadSafeIndex* index_getter) {
  DCHECK(index_getter != nullptr);

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

    Block<TsdfVoxel>::Ptr block = nullptr;
    BlockIndex block_idx;
    VoxelIndex global_voxel_idx;
    while (ray_caster.nextRayIndex(&global_voxel_idx)) {
      TsdfVoxel* voxel =
          allocateStorageAndGetVoxelPtr(global_voxel_idx, &block, &block_idx);

      const float weight = getVoxelWeight(point_C);
      const Point voxel_center_G =
          getCenterPointFromGridIndex(global_voxel_idx, voxel_size_);

      updateTsdfVoxel(origin, point_G, voxel_center_G, color, weight, voxel);
    }
  }
}

void MergedTsdfIntegrator::integratePointCloud(const Transformation& T_G_C,
                                               const Pointcloud& points_C,
                                               const Colors& colors) {
  timing::Timer integrate_timer("integrate");

  // Pre-compute a list of unique voxels to end on.
  // Create a hashmap: VOXEL INDEX -> index in original cloud.
  AnyIndexHashMapType<AlignedVector<size_t>>::type voxel_map;
  // This is a hash map (same as above) to all the indices that need to be
  // cleared.
  AnyIndexHashMapType<AlignedVector<size_t>>::type clear_map;

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

inline void MergedTsdfIntegrator::bundleRays(
    const Transformation& T_G_C, const Pointcloud& points_C,
    const Colors& colors, ThreadSafeIndex* index_getter,
    AnyIndexHashMapType<AlignedVector<size_t>>::type* voxel_map,
    AnyIndexHashMapType<AlignedVector<size_t>>::type* clear_map) {
  DCHECK(voxel_map != nullptr);
  DCHECK(clear_map != nullptr);

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

void MergedTsdfIntegrator::integrateVoxel(
    const Transformation& T_G_C, const Pointcloud& points_C,
    const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
    const std::pair<AnyIndex, AlignedVector<size_t>>& kv,
    const AnyIndexHashMapType<AlignedVector<size_t>>::type& voxel_map) {
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

    const float point_weight = getVoxelWeight(point_C);
    merged_point_C = (merged_point_C * merged_weight + point_C * point_weight) /
                     (merged_weight + point_weight);
    merged_color =
        Color::blendTwoColors(merged_color, merged_weight, color, point_weight);
    merged_weight += point_weight;

    // only take first point when clearing
    if (clearing_ray) {
      break;
    }
  }

  const Point merged_point_G = T_G_C * merged_point_C;

  RayCaster ray_caster(origin, merged_point_G, clearing_ray,
                       config_.voxel_carving_enabled, config_.max_ray_length_m,
                       voxel_size_inv_, config_.default_truncation_distance);

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

    Block<TsdfVoxel>::Ptr block = nullptr;
    BlockIndex block_idx;
    TsdfVoxel* voxel =
        allocateStorageAndGetVoxelPtr(global_voxel_idx, &block, &block_idx);
    const Point voxel_center_G =
        getCenterPointFromGridIndex(global_voxel_idx, voxel_size_);

    updateTsdfVoxel(origin, merged_point_G, voxel_center_G, merged_color,
                    merged_weight, voxel);
  }
}

void MergedTsdfIntegrator::integrateVoxels(
    const Transformation& T_G_C, const Pointcloud& points_C,
    const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
    const AnyIndexHashMapType<AlignedVector<size_t>>::type& voxel_map,
    const AnyIndexHashMapType<AlignedVector<size_t>>::type& clear_map,
    size_t thread_idx) {
  AnyIndexHashMapType<AlignedVector<size_t>>::type::const_iterator it;
  size_t map_size;
  if (clearing_ray) {
    it = clear_map.begin();
    map_size = clear_map.size();
  } else {
    it = voxel_map.begin();
    map_size = voxel_map.size();
  }

  for (size_t i = 0; i < map_size; ++i) {
    if (((i + thread_idx + 1) % config_.integrator_threads) == 0) {
      integrateVoxel(T_G_C, points_C, colors, enable_anti_grazing, clearing_ray,
                     *it, voxel_map);
    }
    ++it;
  }
}

void MergedTsdfIntegrator::integrateRays(
    const Transformation& T_G_C, const Pointcloud& points_C,
    const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
    const AnyIndexHashMapType<AlignedVector<size_t>>::type& voxel_map,
    const AnyIndexHashMapType<AlignedVector<size_t>>::type& clear_map) {
  const Point& origin = T_G_C.getPosition();

  // if only 1 thread just do function call, otherwise spawn threads
  if (config_.integrator_threads == 1) {
    constexpr size_t thread_idx = 0;
    integrateVoxels(T_G_C, points_C, colors, enable_anti_grazing, clearing_ray,
                    voxel_map, clear_map, thread_idx);
  } else {
    AlignedVector<std::thread> integration_threads;
    for (size_t i = 0; i < config_.integrator_threads; ++i) {
      integration_threads.emplace_back(
          &MergedTsdfIntegrator::integrateVoxels, this, T_G_C, points_C, colors,
          enable_anti_grazing, clearing_ray, voxel_map, clear_map, i);
    }

    for (std::thread& thread : integration_threads) {
      thread.join();
    }
  }

  timing::Timer insertion_timer("inserting_missed_blocks");
  updateLayerWithStoredBlocks();

  insertion_timer.Stop();
}

void FastTsdfIntegrator::integrateFunction(const Transformation& T_G_C,
                                           const Pointcloud& points_C,
                                           const Colors& colors,
                                           ThreadSafeIndex* index_getter) {
  DCHECK(index_getter != nullptr);

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
    // Checks to see if another ray in this scan has already started 'close' to
    // this location. If it has then we skip ray casting this point. We measure
    // if a start location is 'close' to another points by inserting the point
    // into a set of voxels. This voxel set has a resolution
    // start_voxel_subsampling_factor times higher then the voxel size.
    AnyIndex global_voxel_idx = getGridIndexFromPoint(
        point_G, config_.start_voxel_subsampling_factor * voxel_size_inv_);
    if (!start_voxel_approx_set_.replaceHash(global_voxel_idx)) {
      continue;
    }

    constexpr bool cast_from_origin = false;
    RayCaster ray_caster(origin, point_G, is_clearing,
                         config_.voxel_carving_enabled,
                         config_.max_ray_length_m, voxel_size_inv_,
                         config_.default_truncation_distance, cast_from_origin);

    size_t consecutive_ray_collisions = 0;

    Block<TsdfVoxel>::Ptr block = nullptr;
    BlockIndex block_idx;
    while (ray_caster.nextRayIndex(&global_voxel_idx)) {
      // Check if the current voxel has been seen by any ray cast this scan. If
      // it has increment the consecutive_ray_collisions counter, otherwise
      // reset it. If the counter reaches a threshold we stop casting as the ray
      // is deemed to be contributing too little new information.
      if (!voxel_observed_approx_set_.replaceHash(global_voxel_idx)) {
        ++consecutive_ray_collisions;
      } else {
        consecutive_ray_collisions = 0;
      }
      if (consecutive_ray_collisions > config_.max_consecutive_ray_collisions) {
        break;
      }

      TsdfVoxel* voxel =
          allocateStorageAndGetVoxelPtr(global_voxel_idx, &block, &block_idx);

      const float weight = getVoxelWeight(point_C);
      const Point voxel_center_G =
          getCenterPointFromGridIndex(global_voxel_idx, voxel_size_);

      updateTsdfVoxel(origin, point_G, voxel_center_G, color, weight, voxel);
    }
  }
}

void FastTsdfIntegrator::integratePointCloud(const Transformation& T_G_C,
                                             const Pointcloud& points_C,
                                             const Colors& colors) {
  timing::Timer integrate_timer("integrate");

  start_voxel_approx_set_.resetApproxSet();
  voxel_observed_approx_set_.resetApproxSet();

  ThreadSafeIndex index_getter(points_C.size(), config_.integrator_threads);

  AlignedVector<std::thread> integration_threads;
  for (size_t i = 0; i < config_.integrator_threads; ++i) {
    integration_threads.emplace_back(&FastTsdfIntegrator::integrateFunction,
                                     this, T_G_C, points_C, colors,
                                     &index_getter);
  }

  for (std::thread& thread : integration_threads) {
    thread.join();
  }

  integrate_timer.Stop();

  timing::Timer insertion_timer("inserting_missed_blocks");
  updateLayerWithStoredBlocks();
  insertion_timer.Stop();
}

}  // namespace voxblox

#ifndef VOXBLOX_FAST_INTEGRATOR_INTEGRATOR_UTILS_H_
#define VOXBLOX_FAST_INTEGRATOR_INTEGRATOR_UTILS_H_

#include <algorithm>
#include <vector>

#include <glog/logging.h>
#include <Eigen/Core>

#include "voxblox_fast/core/common.h"
#include "voxblox_fast/utils/timing.h"

namespace voxblox_fast {

// This function assumes PRE-SCALED coordinates, where one unit = one voxel
// size. The indices are also returned in this scales coordinate system, which
// should map to Local/Voxel indices.
inline void castRay(
    const Point& start_scaled, const Point& end_scaled,
    std::vector<AnyIndex, Eigen::aligned_allocator<AnyIndex> >* indices) {
  CHECK_NOTNULL(indices);

  constexpr FloatingPoint kTolerance = 1e-6;

  const AnyIndex start_index = getGridIndexFromPoint(start_scaled);
  const AnyIndex end_index = getGridIndexFromPoint(end_scaled);
  if (start_index == end_index) {
    return;
  }


  const Ray ray_scaled = end_scaled - start_scaled;

  const AnyIndex ray_step_signs(signum(ray_scaled.x()), signum(ray_scaled.y()),
                          signum(ray_scaled.z()));

  const AnyIndex corrected_step(std::max(0, ray_step_signs.x()),
                          std::max(0, ray_step_signs.y()),
                          std::max(0, ray_step_signs.z()));

  const Point start_scaled_shifted = start_scaled - start_index.cast<FloatingPoint>();

  const Ray distance_to_boundaries(corrected_step.cast<FloatingPoint>() -
                             start_scaled_shifted);

  Ray t_to_next_boundary((std::abs(ray_scaled.x()) < kTolerance)
                             ? 2.0
                             : distance_to_boundaries.x() / ray_scaled.x(),
                         (std::abs(ray_scaled.y()) < kTolerance)
                             ? 2.0
                             : distance_to_boundaries.y() / ray_scaled.y(),
                         (std::abs(ray_scaled.z()) < kTolerance)
                             ? 2.0
                             : distance_to_boundaries.z() / ray_scaled.z());

  // Distance to cross one grid cell along the ray in t.
  // Same as absolute inverse value of delta_coord.
  const Ray t_step_size =
      ray_step_signs.cast<FloatingPoint>().cwiseQuotient(ray_scaled);

  AnyIndex curr_index = start_index;
  indices->push_back(curr_index);

  while (curr_index != end_index) {
    int t_min_idx;
    t_to_next_boundary.minCoeff(&t_min_idx);
    DCHECK_LT(t_min_idx, 3);
    DCHECK_GE(t_min_idx, 0);

    curr_index[t_min_idx] += ray_step_signs[t_min_idx];
    t_to_next_boundary[t_min_idx] += t_step_size[t_min_idx];

    indices->push_back(curr_index);
  }
}


// Takes start and end in WORLD COORDINATES, does all pre-scaling and
// sorting into hierarhical index.
inline void getHierarchicalIndexAlongRay(
    const Point& start, const Point& end, size_t voxels_per_side,
    FloatingPoint voxel_size, FloatingPoint truncation_distance,
    bool voxel_carving_enabled, HierarchicalIndexMap* hierarchical_idx_map) {
  hierarchical_idx_map->clear();

  FloatingPoint voxels_per_side_inv = 1.0 / voxels_per_side;
  FloatingPoint voxel_size_inv = 1.0 / voxel_size;

  const Ray unit_ray = (end - start).normalized();

  const Point ray_end = end + unit_ray * truncation_distance;
  const Point ray_start =
      voxel_carving_enabled ? start : (end - unit_ray * truncation_distance);

  const Point start_scaled = ray_start * voxel_size_inv;
  const Point end_scaled = ray_end * voxel_size_inv;

  IndexVector global_voxel_index;
  timing::Timer cast_ray_timer("integrate/cast_ray");
  castRay(start_scaled, end_scaled, &global_voxel_index);
  cast_ray_timer.Stop();

  timing::Timer create_index_timer("integrate/create_hi_index");
  for (const AnyIndex& global_voxel_idx : global_voxel_index) {
    BlockIndex block_idx = getBlockIndexFromGlobalVoxelIndex(
        global_voxel_idx, voxels_per_side_inv);
    VoxelIndex local_voxel_idx =
        getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side);

    if (local_voxel_idx.x() < 0) {
      local_voxel_idx.x() += voxels_per_side;
    }
    if (local_voxel_idx.y() < 0) {
      local_voxel_idx.y() += voxels_per_side;
    }
    if (local_voxel_idx.z() < 0) {
      local_voxel_idx.z() += voxels_per_side;
    }

    (*hierarchical_idx_map)[block_idx].push_back(local_voxel_idx);
  }
  create_index_timer.Stop();
}

}  // namespace voxblox

#endif  // VOXBLOX_FAST_INTEGRATOR_INTEGRATOR_UTILS_H_

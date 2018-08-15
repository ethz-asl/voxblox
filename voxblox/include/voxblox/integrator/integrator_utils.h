#ifndef VOXBLOX_INTEGRATOR_INTEGRATOR_UTILS_H_
#define VOXBLOX_INTEGRATOR_INTEGRATOR_UTILS_H_

#include <algorithm>
#include <array>
#include <atomic>
#include <vector>

#include <glog/logging.h>
#include <Eigen/Core>

#include "voxblox/core/block_hash.h"
#include "voxblox/core/common.h"
#include "voxblox/utils/timing.h"

namespace voxblox {

// Small class that can be used by multiple threads that need mutually exclusive
// indexes to the same array, while still covering all elements.
// The class attempts to ensure that the points are read in an order that gives
// good coverage over the pointcloud very quickly. This is so that the
// integrator can be terminated before all points have been read (due to time
// constraints) and still capture most of the geometry.
class ThreadSafeIndex {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ThreadSafeIndex(size_t number_of_points);

  // returns true if index is valid, false otherwise
  bool getNextIndex(size_t* idx);

  void reset();

 private:
  size_t getMixedIndex(size_t base_idx);

  std::atomic<size_t> atomic_idx_;
  const size_t number_of_points_;
  const size_t number_of_groups_;

  static constexpr size_t num_bits = 10;  // 1024 bins
  static constexpr size_t step_size_ = 1 << num_bits;
  static constexpr size_t bit_mask_ = step_size_ - 1;

  // Lookup table for the order points in a group should be read in. This is
  // simply a list from 0 to 1023 where each number has had the order of its
  // bits reversed.
  static const std::array<size_t, step_size_> offset_lookup_;
};

// This class assumes PRE-SCALED coordinates, where one unit = one voxel size.
// The indices are also returned in this scales coordinate system, which should
// map to voxel indices.
class RayCaster {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RayCaster(const Point& origin, const Point& point_G,
            const bool is_clearing_ray, const bool voxel_carving_enabled,
            const FloatingPoint max_ray_length_m,
            const FloatingPoint voxel_size_inv,
            const FloatingPoint truncation_distance,
            const bool cast_from_origin = true);

  RayCaster(const Point& start_scaled, const Point& end_scaled);

  // returns false if ray terminates at ray_index, true otherwise
  bool nextRayIndex(GlobalIndex* ray_index);

 private:
  void setupRayCaster(const Point& start_scaled, const Point& end_scaled);

  Ray t_to_next_boundary_;
  GlobalIndex curr_index_;
  AnyIndex ray_step_signs_;
  Ray t_step_size_;

  uint ray_length_in_steps_;
  uint current_step_;
};

// This function assumes PRE-SCALED coordinates, where one unit = one voxel
// size. The indices are also returned in this scales coordinate system, which
// should map to voxel indices.
inline void castRay(const Point& start_scaled, const Point& end_scaled,
                    AlignedVector<GlobalIndex>* indices) {
  CHECK_NOTNULL(indices);

  RayCaster ray_caster(start_scaled, end_scaled);

  GlobalIndex ray_index;
  while (ray_caster.nextRayIndex(&ray_index)) {
    indices->push_back(ray_index);
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

  AlignedVector<GlobalIndex> global_voxel_index;
  timing::Timer cast_ray_timer("integrate/cast_ray");
  castRay(start_scaled, end_scaled, &global_voxel_index);
  cast_ray_timer.Stop();

  timing::Timer create_index_timer("integrate/create_hi_index");
  for (const GlobalIndex& global_voxel_idx : global_voxel_index) {
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

#endif  // VOXBLOX_INTEGRATOR_INTEGRATOR_UTILS_H_

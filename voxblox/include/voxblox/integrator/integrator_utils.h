#ifndef VOXBLOX_INTEGRATOR_INTEGRATOR_UTILS_H_
#define VOXBLOX_INTEGRATOR_INTEGRATOR_UTILS_H_

#include <algorithm>
#include <vector>

#include <glog/logging.h>
#include <Eigen/Core>

#include "voxblox/core/common.h"
#include "voxblox/utils/timing.h"

namespace voxblox {

// Small class that can be used by multiple threads that need mutually exclusive
// indexes to the same array, while still covering all elements.
// The class attempts to make successive indexes far apart so that if the caller
// also accesses neighboring indexes through thread safe locks, the amount of
// time a thread must wait on a lock is minimized.
class ThreadSafeIndex {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ThreadSafeIndex(const size_t number_of_points, const size_t number_of_threads)
      : number_of_points_(number_of_points),
        number_of_threads_(number_of_threads),
        atomic_idx_(0) {}

  // returns true if index is valid, false otherwise
  bool getNextIndex(size_t* idx) {
    DCHECK(idx != nullptr);
    size_t sequential_idx = atomic_idx_.fetch_add(1);

    if (sequential_idx >= number_of_points_) {
      return false;
    } else {
      *idx = getMixedIndex(sequential_idx);
      return true;
    }
  }

  void reset() { atomic_idx_.store(0); }

 private:
  size_t getMixedIndex(size_t base_idx) {
    const size_t number_of_groups = number_of_threads_;
    const size_t points_per_group = number_of_points_ / number_of_groups;

    if (number_of_groups * points_per_group <= base_idx) {
      return base_idx;
    }

    const size_t group_num = base_idx % number_of_groups;
    const size_t position_in_group = base_idx / number_of_groups;

    return group_num * points_per_group + position_in_group;
  }

  const size_t number_of_points_;
  const size_t number_of_threads_;
  std::atomic<size_t> atomic_idx_;
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
            const bool cast_from_origin = true) {
    const Ray unit_ray = (point_G - origin).normalized();

    const Point ray_end = is_clearing_ray
                              ? origin + unit_ray * max_ray_length_m
                              : point_G + unit_ray * truncation_distance;
    const Point ray_start = voxel_carving_enabled
                                ? origin
                                : (point_G - unit_ray * truncation_distance);

    const Point start_scaled = ray_start * voxel_size_inv;
    const Point end_scaled = ray_end * voxel_size_inv;

    if (cast_from_origin) {
      setupRayCaster(start_scaled, end_scaled);
    } else {
      setupRayCaster(end_scaled, start_scaled);
    }
  }

  RayCaster(const Point& start_scaled, const Point& end_scaled) {
    setupRayCaster(start_scaled, end_scaled);
  }

  // returns false if ray terminates at ray_index, true otherwise
  bool nextRayIndex(AnyIndex* ray_index) {
    if (at_end_) {
      return false;
    }
    DCHECK(ray_index != nullptr);
    *ray_index = curr_index_;
    at_end_ = curr_index_ == end_index_;

    int t_min_idx;
    t_to_next_boundary_.minCoeff(&t_min_idx);
    curr_index_[t_min_idx] += ray_step_signs_[t_min_idx];
    t_to_next_boundary_[t_min_idx] += t_step_size_[t_min_idx];

    return true;
  }

 private:
  void setupRayCaster(const Point& start_scaled, const Point& end_scaled) {
    constexpr FloatingPoint kTolerance = 1e-6;

    curr_index_ = getGridIndexFromPoint(start_scaled);
    end_index_ = getGridIndexFromPoint(end_scaled);

    at_end_ = false;

    const Ray ray_scaled = end_scaled - start_scaled;

    ray_step_signs_ = AnyIndex(signum(ray_scaled.x()), signum(ray_scaled.y()),
                               signum(ray_scaled.z()));

    const AnyIndex corrected_step(std::max(0, ray_step_signs_.x()),
                                  std::max(0, ray_step_signs_.y()),
                                  std::max(0, ray_step_signs_.z()));

    const Point start_scaled_shifted =
        start_scaled - curr_index_.cast<FloatingPoint>();

    Ray distance_to_boundaries(corrected_step.cast<FloatingPoint>() -
                               start_scaled_shifted);

    t_to_next_boundary_ =
        Ray((std::abs(ray_scaled.x()) < kTolerance)
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
    t_step_size_ = Ray(
        (std::abs(ray_scaled.x()) < kTolerance) ? 2.0 : ray_step_signs_.x() /
                                                            ray_scaled.x(),
        (std::abs(ray_scaled.y()) < kTolerance) ? 2.0 : ray_step_signs_.y() /
                                                            ray_scaled.y(),
        (std::abs(ray_scaled.z()) < kTolerance) ? 2.0 : ray_step_signs_.z() /
                                                            ray_scaled.z());
  }

  Ray t_to_next_boundary_;
  AnyIndex curr_index_;
  AnyIndex end_index_;
  AnyIndex ray_step_signs_;
  Ray t_step_size_;
  bool at_end_;
};

// This function assumes PRE-SCALED coordinates, where one unit = one voxel
// size. The indices are also returned in this scales coordinate system, which
// should map to voxel indices.
inline void castRay(
    const Point& start_scaled, const Point& end_scaled,
    std::vector<AnyIndex, Eigen::aligned_allocator<AnyIndex> >* indices) {
  CHECK_NOTNULL(indices);

  RayCaster ray_caster(start_scaled, end_scaled);

  AnyIndex ray_index;
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

#endif  // VOXBLOX_INTEGRATOR_INTEGRATOR_UTILS_H_

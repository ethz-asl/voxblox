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
  bool nextRayIndex(AnyIndex* ray_index);

 private:
  void setupRayCaster(const Point& start_scaled, const Point& end_scaled);

  Ray t_to_next_boundary_;
  AnyIndex curr_index_;
  AnyIndex ray_step_signs_;
  Ray t_step_size_;

  uint ray_length_in_steps_;
  uint current_step_;
};

// This function assumes PRE-SCALED coordinates, where one unit = one voxel
// size. The indices are also returned in this scales coordinate system, which
// should map to voxel indices.
inline void castRay(const Point& start_scaled, const Point& end_scaled,
                    AlignedVector<AnyIndex>* indices) {
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

class TriangleIntersector {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TriangleIntersector(const FloatingPoint voxel_size_inv,
                      const Triangle& triangle)
      : triangle_(voxel_size_inv * triangle) {
    // Compute edge vectors for triangle
    const Ray ray_a = triangle_.row(1) - triangle_.row(0);
    const Ray ray_b = triangle_.row(2) - triangle_.row(1);
    const Ray ray_c = triangle_.row(0) - triangle_.row(2);

    normal_ = ray_a.cross(ray_b).normalized();

    axes_vector_ = {
        Ray(0.0, -ray_a.z(), ray_a.y()), Ray(0.0, -ray_b.z(), ray_b.y()),
        Ray(0.0, -ray_b.z(), ray_b.y()), Ray(ray_a.z(), 0.0, -ray_a.x()),
        Ray(ray_b.z(), 0.0, -ray_b.x()), Ray(ray_b.z(), 0.0, -ray_b.x()),
        Ray(-ray_a.y(), ray_a.x(), 0.0), Ray(-ray_b.y(), ray_b.x(), 0.0),
        Ray(-ray_b.y(), ray_b.x(), 0.0)};
  }

  void getIntersectingVoxels(IndexSet* voxel_indexes) {
    IndexSet to_check;
    IndexSet to_check_after;

    raycastEdge(triangle_.row(0), triangle_.row(1), &to_check);
    raycastEdge(triangle_.row(1), triangle_.row(2), &to_check);
    raycastEdge(triangle_.row(0), triangle_.row(2), &to_check);

    // edges that have just been raycast will always be valid
    bool skip_check = true;

    while (!to_check.empty()) {
      for (const VoxelIndex& index : to_check) {
        // check if valid and add neighbors
        if (skip_check ||
            (!voxel_indexes->count(index) && isValidIndex(index))) {
          voxel_indexes->insert(index);

          // 6 conectivity should always works, though there could be cases
          // where we hit issues due to floating point precision TODO check
          // this.
          to_check_after.insert(index + VoxelIndex(0, 0, 1));
          to_check_after.insert(index + VoxelIndex(0, 0, -1));
          to_check_after.insert(index + VoxelIndex(0, 1, 0));
          to_check_after.insert(index + VoxelIndex(0, -1, 0));
          to_check_after.insert(index + VoxelIndex(1, 0, 0));
          to_check_after.insert(index + VoxelIndex(-1, 0, 0));
        }
      }
      skip_check = false;
      to_check.clear();
      to_check.swap(to_check_after);
    }
  }

 private:
  // loosely based on https://gist.github.com/yomotsu/d845f21e2e1eb49f647f
  bool isValidIndex(const VoxelIndex& index) {
    // Translate triangle as conceptually moving AABB to origin
    Triangle triangle_centered =
        triangle_.rowwise() - index.cast<FloatingPoint>().transpose();

    // Test axes a00..a22 (category 3)
    for (const Ray& axis : axes_vector_) {
      const Point test_point = triangle_centered * axis.transpose();

      if (std::max(-test_point.maxCoeff(), test_point.minCoeff()) >
          axis.cwiseAbs().sum()) {
        return false;  // Axis is a separating axis
      }
    }

    // Test the three axes corresponding to the face normals of AABB b (category
    // 1).
    if ((triangle_centered.array().colwise().maxCoeff() < -1.0).any() ||
        (triangle_centered.array().colwise().minCoeff() > 1.0).any()) {
      return false;
    }

    // Test separating axis corresponding to triangle face normal (category 2)
    return std::abs(normal_.dot(triangle_centered.row(0))) <=
           normal_.cwiseAbs().sum();
  }

  void raycastEdge(const Point& a, const Point& b, IndexSet* voxel_indexes) {
    RayCaster ray_caster(a, b);
    AnyIndex ray_index;
    while (ray_caster.nextRayIndex(&ray_index)) {
      voxel_indexes->insert(ray_index);
    }
  }

  const Triangle triangle_;
  Ray normal_;
  Pointcloud axes_vector_;
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_INTEGRATOR_UTILS_H_

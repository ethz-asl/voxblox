#ifndef VOXBLOX_INTEGRATOR_INTEGRATOR_UTILS_FAST_H_
#define VOXBLOX_INTEGRATOR_INTEGRATOR_UTILS_FAST_H_

#include <algorithm>
#include <vector>

#include <glog/logging.h>
#include <Eigen/Core>

#include "voxblox/core/common.h"
#include "voxblox/utils/timing.h"

using namespace voxblox;

namespace voxblox {
namespace fast {

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

}  // namespace fast
}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_INTEGRATOR_UTILS_FAST_H_

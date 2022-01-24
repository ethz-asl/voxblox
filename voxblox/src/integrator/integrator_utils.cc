#include "voxblox/integrator/integrator_utils.h"

namespace voxblox {

ThreadSafeIndex* ThreadSafeIndexFactory::get(const std::string& mode,
                                             const Pointcloud& points_C) {
  if (mode == "mixed") {
    return new MixedThreadSafeIndex(points_C.size());
  } else if (mode == "sorted") {
    return new SortedThreadSafeIndex(points_C);
  } else {
    LOG(FATAL) << "Unknown integration order mode: '" << mode << "'!";
  }
  return nullptr;
}

ThreadSafeIndex::ThreadSafeIndex(size_t number_of_points)
    : atomic_idx_(0), number_of_points_(number_of_points) {}

MixedThreadSafeIndex::MixedThreadSafeIndex(size_t number_of_points)
    : ThreadSafeIndex(number_of_points),
      number_of_groups_(number_of_points / step_size_) {}

SortedThreadSafeIndex::SortedThreadSafeIndex(const Pointcloud& points_C)
    : ThreadSafeIndex(points_C.size()) {
  indices_and_squared_norms_.reserve(points_C.size());
  size_t idx = 0;
  for (const Point& point_C : points_C) {
    indices_and_squared_norms_.emplace_back(idx, point_C.squaredNorm());
    ++idx;
  }

  std::sort(
      indices_and_squared_norms_.begin(), indices_and_squared_norms_.end(),
      [](const std::pair<size_t, double>& a,
         const std::pair<size_t, double>& b) { return a.second < b.second; });
}

// returns true if index is valid, false otherwise
bool ThreadSafeIndex::getNextIndex(size_t* idx) {
  DCHECK(idx != nullptr);
  size_t sequential_idx = atomic_idx_.fetch_add(1);

  if (sequential_idx >= number_of_points_) {
    return false;
  } else {
    *idx = getNextIndexImpl(sequential_idx);
    return true;
  }
}

void ThreadSafeIndex::reset() { atomic_idx_.store(0); }

size_t MixedThreadSafeIndex::getNextIndexImpl(size_t sequential_idx) {
  if (number_of_groups_ * step_size_ <= sequential_idx) {
    return sequential_idx;
  }

  const size_t group_num = sequential_idx % number_of_groups_;
  const size_t position_in_group = sequential_idx / number_of_groups_;

  return group_num * step_size_ + position_in_group;
}

size_t SortedThreadSafeIndex::getNextIndexImpl(size_t sequential_idx) {
  return indices_and_squared_norms_[sequential_idx].first;
}

// This class assumes PRE-SCALED coordinates, where one unit = one voxel size.
// The indices are also returned in this scales coordinate system, which should
// map to voxel indices.
RayCaster::RayCaster(const Point& origin, const Point& point_G,
                     const bool is_clearing_ray,
                     const bool voxel_carving_enabled, // if enabled, then begin from the origin
                     const FloatingPoint max_ray_length_m,
                     const FloatingPoint voxel_size_inv,
                     const FloatingPoint truncation_distance,
                     const bool cast_from_origin) { // defualt: true
  const Ray unit_ray = (point_G - origin).normalized();

  Point ray_start, ray_end;
  if (is_clearing_ray) {
    FloatingPoint ray_length = (point_G - origin).norm();
    ray_length = std::min(std::max(ray_length - truncation_distance,
                                   static_cast<FloatingPoint>(0.0)),
                          max_ray_length_m);
    ray_end = origin + unit_ray * ray_length;
    ray_start = voxel_carving_enabled ? origin : ray_end;
  } else {
    ray_end = point_G + unit_ray * truncation_distance;
    ray_start = voxel_carving_enabled
                    ? origin
                    : (point_G - unit_ray * truncation_distance);
  }

  const Point start_scaled = ray_start * voxel_size_inv;
  const Point end_scaled = ray_end * voxel_size_inv;

  if (cast_from_origin) { // from start to end
    setupRayCaster(start_scaled, end_scaled);
  } else {
    setupRayCaster(end_scaled, start_scaled);
  }
}

RayCaster::RayCaster(const Point& start_scaled, const Point& end_scaled) {
  setupRayCaster(start_scaled, end_scaled);
}

// returns false if ray terminates at ray_index, true otherwise
bool RayCaster::nextRayIndex(GlobalIndex* ray_index) {
  if (current_step_++ > ray_length_in_steps_) {
    return false;
  }

  DCHECK(ray_index != nullptr);
  *ray_index = curr_index_;

  int t_min_idx;
  t_to_next_boundary_.minCoeff(&t_min_idx);
  curr_index_[t_min_idx] += ray_step_signs_[t_min_idx];
  t_to_next_boundary_[t_min_idx] += t_step_size_[t_min_idx];

  return true;
}

void RayCaster::setupRayCaster(const Point& start_scaled,
                               const Point& end_scaled) {
  if (std::isnan(start_scaled.x()) || std::isnan(start_scaled.y()) ||
      std::isnan(start_scaled.z()) || std::isnan(end_scaled.x()) ||
      std::isnan(end_scaled.y()) || std::isnan(end_scaled.z())) {
    ray_length_in_steps_ = 0;
    return;
  }

  curr_index_ = getGridIndexFromPoint<GlobalIndex>(start_scaled);
  const GlobalIndex end_index = getGridIndexFromPoint<GlobalIndex>(end_scaled);
  const GlobalIndex diff_index = end_index - curr_index_;

  current_step_ = 0;

  ray_length_in_steps_ = std::abs(diff_index.x()) + std::abs(diff_index.y()) +
                         std::abs(diff_index.z());

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

  t_to_next_boundary_ = Ray((std::abs(ray_scaled.x()) < 0.0)
                                ? 2.0
                                : distance_to_boundaries.x() / ray_scaled.x(),
                            (std::abs(ray_scaled.y()) < 0.0)
                                ? 2.0
                                : distance_to_boundaries.y() / ray_scaled.y(),
                            (std::abs(ray_scaled.z()) < 0.0)
                                ? 2.0
                                : distance_to_boundaries.z() / ray_scaled.z());

  // Distance to cross one grid cell along the ray in t.
  // Same as absolute inverse value of delta_coord.
  t_step_size_ = Ray(
      (std::abs(ray_scaled.x()) < 0.0) ? 2.0
                                       : ray_step_signs_.x() / ray_scaled.x(),
      (std::abs(ray_scaled.y()) < 0.0) ? 2.0
                                       : ray_step_signs_.y() / ray_scaled.y(),
      (std::abs(ray_scaled.z()) < 0.0) ? 2.0
                                       : ray_step_signs_.z() / ray_scaled.z());
}

}  // namespace voxblox

#ifndef VOXBLOX_INTEGRATOR_RAY_INTEGRATOR_H
#define VOXBLOX_INTEGRATOR_RAY_INTEGRATOR_H

#include <Eigen/Core>
#include <glog/logging.h>

#include <voxblox/core/map.h>

namespace voxblox {

// typedef Eigen::Matrix<FloatingPoint, 3, Eigen::Dynamic> Points;
// typedef Eigen::Matrix<uint8_t, 3, Eigen::Dynamic> Colors;

class Integrator {
 public:
  struct IntegratorConfig {
    float truncation_distance = 0.1;
  };

  Integrator(const TsdfMap::Ptr& map, const IntegratorConfig& config)
      : map_(map), config_(config) {
    CHECK(map_);

    voxel_size_ = map_->getVoxelSize();
    block_size_ = map_->getBlockSize();
    voxels_per_side_ = map_->getVoxelsPerSide();

    voxel_size_inv_ = 1.0 / voxel_size_;
    block_size_inv_ = 1.0 / block_size_;
    voxels_per_side_inv_ = 1.0 / voxels_per_side_;
  }

  int signum(FloatingPoint x) const { return x == 0 ? 0 : x < 0 ? -1 : 1; }

  // Assume side length is 1 -- pre-scale your values accordingly!!!!
  void castRay(const Point& start_coord, const Point& end_coord,
               std::vector<AnyIndex>* indices) {
    CHECK_NOTNULL(indices);

    constexpr FloatingPoint kTolerance = 1e-6;

    // Our code is self-documenting.
    AnyIndex start_index = floorVectorAndDowncast(start_coord);
    AnyIndex end_index = floorVectorAndDowncast(end_coord);

    Ray delta_coord = end_coord - start_coord;
    // Ray ray_direction = (delta_coord)/delta_coord.norm();

    AnyIndex step_direction_signs(signum(delta_coord.x()),
                                  signum(delta_coord.y()),
                                  signum(delta_coord.z()));

    AnyIndex step_correction(std::max(0, step_direction_signs.x()),
                             std::max(0, step_direction_signs.y()),
                             std::max(0, step_direction_signs.z()));

    Point local_start_coord = start_coord - start_index.cast<FloatingPoint>();

    Ray distance_to_boundaries(
        step_correction.cast<FloatingPoint>() - local_start_coord);

    Coordinates t_to_next_boundary(
        (delta_coord.x() < kTolerance) ?
            2.0 : distance_to_boundaries.x() / delta_coord.x(),
        (delta_coord.y() < kTolerance) ?
            2.0 : distance_to_boundaries.y() / delta_coord.y(),
        (delta_coord.x() < kTolerance) ?
            2.0 : distance_to_boundaries.z() / delta_coord.z());

    // Distance to cross one grid cell along the ray in t.
    // Same as absolute inverse value of delta_coord.
    Ray t_step_size = step_direction_signs.cast<FloatingPoint>().cwiseQuotient(
        delta_coord);

    AnyIndex curr_index = start_index;
    indices->push_back(curr_index);

    while (curr_index != end_index) {
      int t_min_idx;
      FloatingPoint t_min = t_to_next_boundary.minCoeff(&t_min_idx);
      CHECK_LT(t_min_idx, 3);
      CHECK_GE(t_min_idx, 0);

      curr_index[t_min_idx] += step_direction_signs[t_min_idx];
      t_to_next_boundary[t_min_idx] += t_step_size[t_min_idx];

      indices->push_back(curr_index);
    }
  }

  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C, const Colors& colors) {
    const Point& origin = T_G_C.getPosition();
    const Point& origin_scaled = origin / block_size_;

    for (size_t pt_idx = 0; pt_idx < points_C.size(); ++pt_idx) {
      const Point& point_G = T_G_C * points_C[pt_idx];
      const Point& point_G_scaled = point_G / block_size_;

      IndexVector block_indices;
      castRay(origin_scaled, point_G_scaled, &block_indices);

      LOG(INFO) << "castRay computed " << block_indices.size() << "block indices between " << origin.transpose() << " and " << point_G.transpose();
    }
  }

 protected:
  TsdfMap::Ptr map_;

  IntegratorConfig config_;

  // Cached map config.
  FloatingPoint voxel_size_;
  FloatingPoint voxels_per_side_;
  FloatingPoint block_size_;

  // Derived types.
  FloatingPoint voxel_size_inv_;
  FloatingPoint voxels_per_side_inv_;
  FloatingPoint block_size_inv_;
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_RAY_INTEGRATOR_H

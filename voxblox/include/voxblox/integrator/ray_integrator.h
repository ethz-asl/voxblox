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

  void integratePointCloud(const Transformation& T_G_C, const Points& points_C,
                           const Colors& colors) {
    const Points points_G = T_G_C * points_C;

    const Point& origin = T_G_C.getPosition();

    for (size_t pt_idx = 0; pt_idx < points_C.cols(); ++pt_idx) {
      const Point& point_G = points_G.col(pt_idx);
      const Ray& ray = point_G - origin;
    }
  }

  int signum(FloatingPoint x) const { return x == 0 ? 0 : x < 0 ? -1 : 1; }

  // Assume side length is 1 -- pre-scale your values accordingly!!!!
  void castRay(const Point& start_coord, const Point& end_coord,
               std::vector<AnyIndex>* indices) {
    // Our code is self-documenting.
    Eigen::Vector3i start_index = floorVectorAndDowncast(start_coord);
    Eigen::Vector3i end_index = floorVectorAndDowncast(end_coord);

    Ray delta_coord = end_coord - start_coord;
    // Ray ray_direction = (delta_coord)/delta_coord.norm();

    Eigen::Vector3i step_direction_signs(signum(delta_coord.x()),
                                        signum(delta_coord.y()),
                                        signum(delta_coord.z()));

    // Direction to increment x,y,z when stepping.
    int stepX = signum(dx);
    int stepY = signum(dy);
    int stepZ = signum(dz);
    // See description above. The initial values depend on the fractional
    // part of the origin.
    float tMaxX = intbound(start.x(), dx);
    float tMaxY = intbound(start.y(), dy);
    float tMaxZ = intbound(start.z(), dz);
    // The change in t when taking a step (always positive).
    float tDeltaX = ((float)stepX) / dx;
    float tDeltaY = ((float)stepY) / dy;
    float tDeltaZ = ((float)stepZ) / dz;

    // Avoids an infinite loop.
    if (stepX == 0 && stepY == 0 && stepZ == 0) return;

    while (true) {
      if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() &&
          z >= min.z() && z < max.z())
        output->push_back(Point3(x, y, z));

      if (x == endX && y == endY && z == endZ) break;

      // tMaxX stores the t-value at which we cross a cube boundary along the
      // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
      // chooses the closest cube boundary. Only the first case of the four
      // has been commented in detail.
      if (tMaxX < tMaxY) {
        if (tMaxX < tMaxZ) {
          // Update which cube we are now in.
          x += stepX;
          // Adjust tMaxX to the next X-oriented boundary crossing.
          tMaxX += tDeltaX;
        } else {
          z += stepZ;
          tMaxZ += tDeltaZ;
        }
      } else {
        if (tMaxY < tMaxZ) {
          y += stepY;
          tMaxY += tDeltaY;
        } else {
          z += stepZ;
          tMaxZ += tDeltaZ;
        }
      }
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

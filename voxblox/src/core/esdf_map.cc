#include "voxblox/core/esdf_map.h"

namespace voxblox {

bool EsdfMap::getDistanceAtPosition(const Eigen::Vector3d& position,
                                    double* distance) const {
  constexpr bool interpolate = false;
  FloatingPoint distance_fp;
  bool success = interpolator_.getDistance(position, &distance_fp, interpolate);
  if (success) {
    *distance = static_cast<double>(distance_fp);
  }
  return success;
}

bool EsdfMap::getDistanceAndGradientAtPosition(
    const Eigen::Vector3d& position, double* distance,
    Eigen::Vector3d* gradient) const {
  bool success = false;

  FloatingPoint distance_fp;
  constexpr bool interpolate = false;
  success = interpolator_.getDistance(position, &distance_fp, interpolate);
  if (!success) {
    return false;
  }

  Point gradient_fp = Point::Zero();
  success = interpolator_.getGradient(position, &gradient_fp, interpolate);

  // if (!success) {
  //   return false;
  // }

  *distance = static_cast<double>(distance_fp);
  *gradient = gradient_fp.cast<double>();

  return true;
}

}  // namespace voxblox

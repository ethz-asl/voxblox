#include "voxblox/core/esdf_map.h"

namespace voxblox {

bool EsdfMap::getDistanceAtPosition(const Eigen::Vector3d& position,
                                    double* distance) const {
  constexpr bool interpolate = true;
  FloatingPoint distance_fp;
  bool success = interpolator_.getDistance(position, distance, interpolate);
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
  constexpr bool interpolate = true;
  success = getDistance(position, &distance_fp, interpolate);
  if (!success) {
    return false;
  }

  Point gradient_fp;
  success = getGradient(position, &gradient_fp, interpolate);

  if (!success) {
    return false;
  }

  *distance = static_cast<double>(distance_fp);
  *gradient = gradient_fp.cast<double>();

  return true;
}

}  // namespace voxblox

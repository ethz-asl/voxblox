#include "voxblox/core/esdf_map.h"

namespace voxblox {

bool EsdfMap::getDistanceAtPosition(const Eigen::Vector3d& position,
                                    double* distance) const {
  constexpr bool interpolate = false;
  FloatingPoint distance_fp;
  bool success = interpolator_.getDistance(position.cast<FloatingPoint>(),
                                           &distance_fp, interpolate);
  if (success) {
    *distance = static_cast<double>(distance_fp);
  }
  return success;
}

bool EsdfMap::getDistanceAndGradientAtPosition(
    const Eigen::Vector3d& position, double* distance,
    Eigen::Vector3d* gradient) const {
  FloatingPoint distance_fp;
  Point gradient_fp = Point::Zero();

  bool success = interpolator_.getAdaptiveDistanceAndGradient(
      position.cast<FloatingPoint>(), &distance_fp, &gradient_fp);

  *distance = static_cast<double>(distance_fp);
  *gradient = gradient_fp.cast<double>();

  return success;
}

bool EsdfMap::isObserved(const Eigen::Vector3d& position) const {
  // Get the block.
  Block<EsdfVoxel>::Ptr block_ptr =
      esdf_layer_->getBlockPtrByCoordinates(position.cast<FloatingPoint>());
  if (block_ptr) {
    const EsdfVoxel& voxel =
        block_ptr->getVoxelByCoordinates(position.cast<FloatingPoint>());
    return voxel.observed;
  }
  return false;
}

}  // namespace voxblox

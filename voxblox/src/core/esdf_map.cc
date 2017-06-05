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

void EsdfMap::batchGetDistanceAtPosition(
    EigenDRef<const Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions,
    Eigen::Ref<Eigen::VectorXd> distances,
    Eigen::Ref<Eigen::VectorXi> observed) const {
  distances.resize(positions.cols());
  observed.resize(positions.cols());
  for (int i = 0; i < positions.cols(); i++) {
    observed[i] = getDistanceAtPosition(positions.col(i), &distances[i]);
  }
}

void EsdfMap::batchGetDistanceAndGradientAtPosition(
    EigenDRef<const Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions,
    Eigen::Ref<Eigen::VectorXd> distances,
    EigenDRef<Eigen::Matrix<double, 3, Eigen::Dynamic>>& gradients,
    Eigen::Ref<Eigen::VectorXi> observed) const {
  distances.resize(positions.cols());
  observed.resize(positions.cols());
  for (int i = 0; i < positions.cols(); i++) {
    Eigen::Vector3d gradient;
    observed[i] = getDistanceAndGradientAtPosition(positions.col(i),
                                                   &distances[i],
                                                   &gradient);

    gradients.col(i) = gradient;
  }
}

void EsdfMap::batchIsObserved(
    EigenDRef<const Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions,
    Eigen::Ref<Eigen::VectorXi> observed) const {
  observed.resize(positions.cols());
  for (int i = 0; i < positions.cols(); i++) {
    observed[i] = isObserved(positions.col(i));
  }
}

}  // namespace voxblox

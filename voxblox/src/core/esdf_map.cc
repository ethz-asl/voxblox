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
  /* TODO(mereweth@jpl.nasa.gov) - this looks like it gets truncated on return
   * to Python anyway. Throw std::runtime_error here if too small?
   */
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
  /* TODO(mereweth@jpl.nasa.gov) - this looks like it gets truncated on return
   * to Python anyway. Throw std::runtime_error here if too small?
   */
  distances.resize(positions.cols());
  gradients.resize(3, positions.cols());
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
  /* TODO(mereweth@jpl.nasa.gov) - this looks like it gets truncated on return
   * to Python anyway. Throw std::runtime_error here if too small?
   */
  observed.resize(positions.cols());
  for (int i = 0; i < positions.cols(); i++) {
    observed[i] = isObserved(positions.col(i));
  }
}

unsigned int EsdfMap::coordPlaneSliceGetDistance(
                unsigned int free_plane_index,
                double free_plane_val,
                EigenDRef<Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions,
                Eigen::Ref<Eigen::VectorXd> distances) const {
  BlockIndexList blocks;
  esdf_layer_->getAllAllocatedBlocks(&blocks);

  // Cache layer settings.
  size_t vps = esdf_layer_->voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  // Temp variables.
  bool did_all_fit = true;
  unsigned int count = 0;

  // Iterate over all blocks.
  for (const BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const Block<EsdfVoxel>& block = esdf_layer_->getBlockByIndex(index);

    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
         ++linear_index) {
      Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      const EsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);
      if ((std::abs(coord(free_plane_index) - free_plane_val)
                                          <= esdf_layer_->voxel_size() / 2.0)
                                          && voxel.observed) {
        if (count < positions.cols()) {
          positions.col(count) = Eigen::Vector3d(coord.x(), coord.y(), coord.z());
        }
        else {
          did_all_fit = false;
        }
        if (count < distances.size()) {
          distances(count) = voxel.distance;
        }
        else {
          did_all_fit = false;
        }
        count++;
      }
    }
  }

  if (!did_all_fit) {
    throw std::runtime_error(std::string("Unable to store ") + std::to_string(count) + " values.");
  }

  return count;
}

}  // namespace voxblox

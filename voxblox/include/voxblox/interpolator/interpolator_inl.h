#ifndef VOXBLOX_INTERPOLATOR_INTERPOLATOR_INL_H_
#define VOXBLOX_INTERPOLATOR_INTERPOLATOR_INL_H_

namespace voxblox {

template <typename VoxelType>
Interpolator<VoxelType>::Interpolator(Layer<VoxelType>* layer)
    : layer_(layer) {}

template <typename VoxelType>
bool Interpolator<VoxelType>::getDistance(const Point& pos,
                                          FloatingPoint* distance,
                                          bool interpolate) const {
  if (interpolate) {
    return getInterpDistance(pos, distance);
  } else {
    return getNearestDistance(pos, distance);
  }
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getGradient(const Point& pos, Point* grad,
                                          const bool interpolate) const {
  CHECK_NOTNULL(grad);

  typename Layer<VoxelType>::BlockType::ConstPtr block_ptr =
      layer_->getBlockPtrByCoordinates(pos);
  if (block_ptr == nullptr) {
    return false;
  }
  // Now get the gradient.
  *grad = Point::Zero();
  // Iterate over all 3 D, and over negative and positive signs in central
  // difference.
  for (unsigned int i = 0u; i < 3u; ++i) {
    for (int sign = -1; sign <= 1; sign += 2) {
      Point offset = Point::Zero();
      offset(i) = sign * block_ptr->voxel_size();
      FloatingPoint offset_distance;
      if (!getDistance(pos + offset, &offset_distance, interpolate)) {
        return false;
      }
      (*grad)(i) += offset_distance * static_cast<FloatingPoint>(sign);
    }
  }
  // Scale by correct size.
  // This is central difference, so it's 2x voxel size between
  // measurements.
  *grad /= (2 * block_ptr->voxel_size());
  return true;
}

template <typename VoxelType>
bool Interpolator<VoxelType>::setIndexes(const Point& pos,
                                         BlockIndex* block_index,
                                         InterpIndexes* voxel_indexes) const {
  // get voxel index
  *block_index = layer_->computeBlockIndexFromCoordinates(pos);
  typename Layer<VoxelType>::BlockType::ConstPtr block_ptr =
      layer_->getBlockPtrByIndex(*block_index);
  if (block_ptr == nullptr) {
    return false;
  }
  VoxelIndex voxel_index = block_ptr->computeVoxelIndexFromCoordinates(pos);

  // shift index to bottom left corner voxel (makes math easier)
  Point center_offset =
      pos - block_ptr->computeCoordinatesFromVoxelIndex(voxel_index);
  for (size_t i = 0; i < center_offset.rows(); ++i) {
    // ensure that the point we are interpolating to is always larger than the
    // center of the voxel_index in all dimensions
    if (center_offset(i) < 0) {
      voxel_index(i)--;
      // move blocks if needed
      if (voxel_index(i) < 0) {
        (*block_index)(i)--;
        voxel_index(i) += block_ptr->voxels_per_side();
      }
    }
  }

  // get indexes of neighbors
  (*voxel_indexes) << 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0,
      0, 1, 1, 1, 1;

  voxel_indexes->colwise() += voxel_index.array();
  return true;
}

template <typename VoxelType>
void Interpolator<VoxelType>::getWeightsVector(
    const Point& voxel_pos, const Point& pos,
    InterpVector* weights_vector) const {
  CHECK_NOTNULL(weights_vector);

  Point voxel_offset = pos - voxel_pos;

  CHECK((voxel_offset.array() >= 0).all());  // NOLINT

  *weights_vector << 1, voxel_offset[0], voxel_offset[1], voxel_offset[2],
      voxel_offset[0] * voxel_offset[1], voxel_offset[1] * voxel_offset[2],
      voxel_offset[2] * voxel_offset[0],
      voxel_offset[0] * voxel_offset[1] * voxel_offset[2];
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getDistancesAndWeights(
    const BlockIndex& block_index, const InterpIndexes& voxel_indexes,
    const Point& pos, InterpVector* distances,
    InterpVector* weights_vector) const {
  CHECK_NOTNULL(distances);

  // for each voxel index
  bool success = true;
  for (size_t i = 0; i < voxel_indexes.cols(); ++i) {
    typename Layer<VoxelType>::BlockType::Ptr block_ptr =
        layer_->getBlockPtrByIndex(block_index);
    if (block_ptr == nullptr) {
      return false;
    }

    VoxelIndex voxel_index = voxel_indexes.col(i);
    // if voxel index is too large get neighboring block and update index
    if ((voxel_index.array() >= block_ptr->voxels_per_side()).any()) {
      BlockIndex new_block_index = block_index;
      for (size_t j = 0; j < block_index.rows(); ++j) {
        if (voxel_index(j) >= block_ptr->voxels_per_side()) {
          new_block_index(j)++;
          voxel_index(j) -= block_ptr->voxels_per_side();
        }
      }
      block_ptr = layer_->getBlockPtrByIndex(new_block_index);
      if (block_ptr == nullptr) {
        return false;
      }
    }
    // use bottom left corner voxel to compute weights vector
    if (i == 0) {
      getWeightsVector(block_ptr->computeCoordinatesFromVoxelIndex(voxel_index),
                       pos, weights_vector);
    }

    const VoxelType& voxel = block_ptr->getVoxelByVoxelIndex(voxel_index);

    (*distances)[i] = getVoxelDistance(voxel);
    success = success && isVoxelValid(voxel);
  }
  return success;
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getInterpDistance(const Point& pos,
                                                FloatingPoint* distance) const {
  CHECK_NOTNULL(distance);

  // get block and voxels indexes (some voxels may have negative indexes)
  BlockIndex block_index;
  InterpIndexes voxel_indexes;
  if (!setIndexes(pos, &block_index, &voxel_indexes)) {
    return false;
  }

  // get distances of 8 surrounding voxels and weights vector
  InterpVector distances;
  InterpVector weights_vector;
  if (!getDistancesAndWeights(block_index, voxel_indexes, pos, &distances,
                              &weights_vector)) {
    return false;
  }

  // table for tri-linear interpolation (http://spie.org/samples/PM159.pdf)
  InterpTable interp_table;
  interp_table << 1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 1, 0, 0, 0, -1, 0, 1, 0,
      0, 0, 0, 0, -1, 1, 0, 0, 0, 0, 0, 0, 1, 0, -1, 0, -1, 0, 1, 0,  // NOLINT
      1, -1, -1, 1, 0, 0, 0, 0,                                       // NOLINT
      1, -1, 0, 0, -1, 1, 0, 0,                                       // NOLINT
      -1, 1, 1, -1, 1, -1, -1, 1;                                     // NOLINT

  // interpolate
  *distance = weights_vector * (interp_table * distances.transpose());
  return true;
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getNearestDistance(
    const Point& pos, FloatingPoint* distance) const {
  CHECK_NOTNULL(distance);

  typename Layer<VoxelType>::BlockType::ConstPtr block_ptr =
      layer_->getBlockPtrByCoordinates(pos);
  if (block_ptr == nullptr) {
    return false;
  }

  const VoxelType& voxel = block_ptr->getVoxelByCoordinates(pos);

  *distance = getVoxelDistance(voxel);

  return isVoxelValid(voxel);
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getNearestDistanceAndWeight(
    const Point& pos, FloatingPoint* distance, float* weight) const {
  CHECK_NOTNULL(distance);
  CHECK_NOTNULL(weight);

  typename Layer<VoxelType>::BlockType::ConstPtr block_ptr =
      layer_->getBlockPtrByCoordinates(pos);
  if (block_ptr == nullptr) {
    return false;
  }
  const VoxelType& voxel = block_ptr->getVoxelByCoordinates(pos);
  *distance = getVoxelDistance(voxel);
  *weight = getVoxelWeight(voxel);
  return true;
}

// Specializations for TSDF and ESDF voxels.
template <>
inline FloatingPoint Interpolator<TsdfVoxel>::getVoxelDistance(
    const TsdfVoxel& voxel) const {
  return voxel.distance;
}

template <>
inline FloatingPoint Interpolator<EsdfVoxel>::getVoxelDistance(
    const EsdfVoxel& voxel) const {
  return voxel.distance;
}

template <>
inline float Interpolator<TsdfVoxel>::getVoxelWeight(
    const TsdfVoxel& voxel) const {
  return voxel.weight;
}

template <>
inline float Interpolator<EsdfVoxel>::getVoxelWeight(
    const EsdfVoxel& voxel) const {
  return voxel.observed ? 1.0f : 0.0f;
}

template <>
inline bool Interpolator<TsdfVoxel>::isVoxelValid(
    const TsdfVoxel& voxel) const {
  return voxel.weight > 0.0;
}

template <>
inline bool Interpolator<EsdfVoxel>::isVoxelValid(
    const EsdfVoxel& voxel) const {
  return voxel.observed;
}

}  // namespace voxblox

#endif  // VOXBLOX_INTERPOLATOR_INTERPOLATOR_INL_H_

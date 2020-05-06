#ifndef VOXBLOX_INTERPOLATOR_INTERPOLATOR_INL_H_
#define VOXBLOX_INTERPOLATOR_INTERPOLATOR_INL_H_

#include <iostream>

#include "voxblox/utils/evaluation_utils.h"

namespace voxblox {

template <typename VoxelType>
Interpolator<VoxelType>::Interpolator(const Layer<VoxelType>* layer)
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
bool Interpolator<VoxelType>::getWeight(const Point& pos, FloatingPoint* weight,
                                        bool interpolate) const {
  CHECK_NOTNULL(weight);
  if (interpolate) {
    return getInterpWeight(pos, weight);
  } else {
    return getNearestWeight(pos, weight);
  }
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getVoxel(const Point& pos, VoxelType* voxel,
                                       bool interpolate) const {
  if (interpolate) {
    return getInterpVoxel(pos, voxel);
  } else {
    return getNearestVoxel(pos, voxel);
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
  // This is central difference, so it's 2x voxel size between measurements.
  *grad /= (2 * block_ptr->voxel_size());
  return true;
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getAdaptiveDistanceAndGradient(
    const Point& pos, FloatingPoint* distance, Point* grad) const {
  // TODO(helenol): try to see how to minimize number of lookups for the
  // gradient and interpolation calculations...

  // Get the nearest neighbor distance first, we need this for the gradient
  // calculations anyway.
  FloatingPoint nearest_neighbor_distance = 0.0f;
  bool interpolate = false;
  if (!getDistance(pos, &nearest_neighbor_distance, interpolate)) {
    // Then there is no data here at all.
    return false;
  }

  // Then try to get the interpolated distance.
  interpolate = true;
  bool has_interpolated_distance = getDistance(pos, distance, interpolate);

  // Now try to estimate the gradient. Same general procedure as getGradient()
  // above, but also allow finite difference methods other than central
  // difference (left difference, right difference).
  typename Layer<VoxelType>::BlockType::ConstPtr block_ptr =
      layer_->getBlockPtrByCoordinates(pos);
  if (block_ptr == nullptr) {
    return false;
  }

  Point gradient = Point::Zero();

  // Try to get the full gradient if possible.
  bool has_interpolated_gradient = false;
  if (has_interpolated_distance) {
    has_interpolated_gradient = getGradient(pos, &gradient, interpolate);
  }

  if (!has_interpolated_gradient) {
    // Otherwise fall back to this...
    interpolate = false;
    for (unsigned int i = 0u; i < 3u; ++i) {
      // First check if we can get both sides for central difference.
      Point offset = Point::Zero();
      offset(i) = block_ptr->voxel_size();
      FloatingPoint left_distance = 0.0f, right_distance = 0.0f;
      bool left_valid = getDistance(pos - offset, &left_distance, interpolate);
      bool right_valid =
          getDistance(pos + offset, &right_distance, interpolate);

      if (left_valid && right_valid) {
        gradient(i) =
            (right_distance - left_distance) / (2.0f * block_ptr->voxel_size());
      } else if (left_valid) {
        gradient(i) = (nearest_neighbor_distance - left_distance) /
                      block_ptr->voxel_size();
      } else if (right_valid) {
        gradient(i) = (right_distance - nearest_neighbor_distance) /
                      block_ptr->voxel_size();
      } else {
        // This has no neighbors on any side in this dimension :(
        return false;
      }
    }
  }

  // If we weren't able to get the original interpolated distance value, then
  // use the computed gradient to estimate what the value should be.
  if (!has_interpolated_distance) {
    VoxelIndex voxel_index =
        block_ptr->computeTruncatedVoxelIndexFromCoordinates(pos);
    Point voxel_pos = block_ptr->computeCoordinatesFromVoxelIndex(voxel_index);

    Point voxel_offset = pos - voxel_pos;
    *distance = nearest_neighbor_distance + voxel_offset.dot(gradient);
  }

  *grad = gradient;
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
  VoxelIndex voxel_index =
      block_ptr->computeTruncatedVoxelIndexFromCoordinates(pos);

  // shift index to bottom left corner voxel (makes math easier)
  Point center_offset =
      pos - block_ptr->computeCoordinatesFromVoxelIndex(voxel_index);
  for (size_t i = 0; i < static_cast<size_t>(center_offset.rows()); ++i) {
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

  // FROM PAPER (http://spie.org/samples/PM159.pdf)
  // clang-format off
  (*voxel_indexes) <<
    0, 0, 0, 0, 1, 1, 1, 1,
    0, 0, 1, 1, 0, 0, 1, 1,
    0, 1, 0, 1, 0, 1, 0, 1;
  // clang-format on

  voxel_indexes->colwise() += voxel_index.array();
  return true;
}

template <typename VoxelType>
void Interpolator<VoxelType>::getQVector(const Point& voxel_pos,
                                         const Point& pos,
                                         const FloatingPoint voxel_size_inv,
                                         InterpVector* q_vector) const {
  CHECK_NOTNULL(q_vector);

  const Point voxel_offset = (pos - voxel_pos) * voxel_size_inv;

  CHECK((voxel_offset.array() >= 0).all());  // NOLINT

  // FROM PAPER (http://spie.org/samples/PM159.pdf)
  // clang-format off
  *q_vector <<
      1,
      voxel_offset[0],
      voxel_offset[1],
      voxel_offset[2],
      voxel_offset[0] * voxel_offset[1],
      voxel_offset[1] * voxel_offset[2],
      voxel_offset[2] * voxel_offset[0],
      voxel_offset[0] * voxel_offset[1] * voxel_offset[2];
  // clang-format on
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getVoxelsAndQVector(
    const BlockIndex& block_index, const InterpIndexes& voxel_indexes,
    const Point& pos, const VoxelType** voxels, InterpVector* q_vector) const {
  CHECK_NOTNULL(q_vector);

  // for each voxel index
  for (size_t i = 0; i < static_cast<size_t>(voxel_indexes.cols()); ++i) {
    typename Layer<VoxelType>::BlockType::ConstPtr block_ptr =
        layer_->getBlockPtrByIndex(block_index);
    if (block_ptr == nullptr) {
      return false;
    }

    VoxelIndex voxel_index = voxel_indexes.col(i);
    // if voxel index is too large get neighboring block and update index
    if ((voxel_index.array() >= block_ptr->voxels_per_side()).any()) {
      BlockIndex new_block_index = block_index;
      for (size_t j = 0; j < static_cast<size_t>(block_index.rows()); ++j) {
        if (voxel_index(j) >=
            static_cast<IndexElement>(block_ptr->voxels_per_side())) {
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
      getQVector(block_ptr->computeCoordinatesFromVoxelIndex(voxel_index), pos,
                 block_ptr->voxel_size_inv(), q_vector);
    }

    const VoxelType& voxel = block_ptr->getVoxelByVoxelIndex(voxel_index);

    voxels[i] = &voxel;
    if (!utils::isObservedVoxel(voxel)) {
      return false;
    }
  }
  return true;
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getVoxelsAndQVector(
    const Point& pos, const VoxelType** voxels, InterpVector* q_vector) const {
  // get block and voxels indexes (some voxels may have negative indexes)
  BlockIndex block_index;
  InterpIndexes voxel_indexes;
  if (!setIndexes(pos, &block_index, &voxel_indexes)) {
    return false;
  }

  // get distances of 8 surrounding voxels and weights vector
  return getVoxelsAndQVector(block_index, voxel_indexes, pos, voxels, q_vector);
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getInterpDistance(const Point& pos,
                                                FloatingPoint* distance) const {
  CHECK_NOTNULL(distance);

  // get distances of 8 surrounding voxels and weights vector
  const VoxelType* voxels[8];
  InterpVector q_vector;
  if (!getVoxelsAndQVector(pos, voxels, &q_vector)) {
    return false;
  } else {
    *distance = interpMember(q_vector, voxels, &getVoxelSdf);
    return true;
  }
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

  *distance = getVoxelSdf(voxel);

  return utils::isObservedVoxel(voxel);
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getInterpWeight(const Point& pos,
                                              FloatingPoint* weight) const {
  CHECK_NOTNULL(weight);

  // get distances of 8 surrounding voxels and weights vector
  const VoxelType* voxels[8];
  InterpVector q_vector;
  if (!getVoxelsAndQVector(pos, voxels, &q_vector)) {
    return false;
  } else {
    *weight = interpMember(q_vector, voxels, &getVoxelWeight);
    return true;
  }
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getNearestWeight(const Point& pos,
                                               FloatingPoint* weight) const {
  CHECK_NOTNULL(weight);

  typename Layer<VoxelType>::BlockType::ConstPtr block_ptr =
      layer_->getBlockPtrByCoordinates(pos);
  if (block_ptr == nullptr) {
    return false;
  }

  const VoxelType& voxel = block_ptr->getVoxelByCoordinates(pos);

  *weight = getVoxelWeight(voxel);

  return utils::isObservedVoxel(voxel);
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getInterpVoxel(const Point& pos,
                                             VoxelType* voxel) const {
  CHECK_NOTNULL(voxel);

  // get voxels of 8 surrounding voxels and weights vector
  const VoxelType* voxels[8];
  InterpVector q_vector;
  if (!getVoxelsAndQVector(pos, voxels, &q_vector)) {
    return false;
  } else {
    *voxel = interpVoxel(q_vector, voxels);
    return true;
  }
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getNearestVoxel(const Point& pos,
                                              VoxelType* voxel) const {
  CHECK_NOTNULL(voxel);

  typename Layer<VoxelType>::BlockType::ConstPtr block_ptr =
      layer_->getBlockPtrByCoordinates(pos);
  if (block_ptr == nullptr) {
    return false;
  }

  *voxel = block_ptr->getVoxelByCoordinates(pos);

  return utils::isObservedVoxel(*voxel);
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
  *distance = getVoxelSdf(voxel);
  *weight = getVoxelWeight(voxel);
  return true;
}

template <>
inline float Interpolator<TsdfVoxel>::getVoxelSdf(const TsdfVoxel& voxel) {
  return voxel.distance;
}

template <>
inline float Interpolator<EsdfVoxel>::getVoxelSdf(const EsdfVoxel& voxel) {
  return voxel.distance;
}

template <typename VoxelType>
inline float Interpolator<VoxelType>::getVoxelWeight(
    const VoxelType& /*voxel*/) {
  return 0.0;
}

template <>
inline float Interpolator<TsdfVoxel>::getVoxelWeight(const TsdfVoxel& voxel) {
  return voxel.weight;
}

template <>
inline float Interpolator<EsdfVoxel>::getVoxelWeight(const EsdfVoxel& voxel) {
  return voxel.observed ? 1.0f : 0.0f;
}

template <>
inline uint8_t Interpolator<TsdfVoxel>::getRed(const TsdfVoxel& voxel) {
  return voxel.color.r;
}

template <>
inline uint8_t Interpolator<TsdfVoxel>::getGreen(const TsdfVoxel& voxel) {
  return voxel.color.g;
}

template <>
inline uint8_t Interpolator<TsdfVoxel>::getBlue(const TsdfVoxel& voxel) {
  return voxel.color.b;
}

template <>
inline uint8_t Interpolator<TsdfVoxel>::getAlpha(const TsdfVoxel& voxel) {
  return voxel.color.a;
}

template <typename VoxelType>
template <typename TGetter>
inline FloatingPoint Interpolator<VoxelType>::interpMember(
    const InterpVector& q_vector, const VoxelType** voxels,
    TGetter (*getter)(const VoxelType&)) {
  InterpVector data;
  for (int i = 0; i < data.size(); ++i) {
    data[i] = static_cast<FloatingPoint>((*getter)(*voxels[i]));
  }

  // FROM PAPER (http://spie.org/samples/PM159.pdf)
  // clang-format off
  static const InterpTable interp_table =
      (InterpTable() <<
        1,  0,  0,  0,  0,  0,  0,  0,
       -1,  0,  0,  0,  1,  0,  0,  0,
       -1,  0,  1,  0,  0,  0,  0,  0,
       -1,  1,  0,  0,  0,  0,  0,  0,
        1,  0, -1,  0, -1,  0,  1,  0,
        1, -1, -1,  1,  0,  0,  0,  0,
        1, -1,  0,  0, -1,  1,  0,  0,
       -1,  1,  1, -1,  1, -1, -1,  1
       )
          .finished();
  // clang-format on
  return q_vector * (interp_table * data.transpose());
}

template <>
inline TsdfVoxel Interpolator<TsdfVoxel>::interpVoxel(
    const InterpVector& q_vector, const TsdfVoxel** voxels) {
  TsdfVoxel voxel;
  voxel.distance = interpMember(q_vector, voxels, &getVoxelSdf);
  voxel.weight = interpMember(q_vector, voxels, &getVoxelWeight);

  voxel.color.r = interpMember(q_vector, voxels, &getRed);
  voxel.color.g = interpMember(q_vector, voxels, &getGreen);
  voxel.color.b = interpMember(q_vector, voxels, &getBlue);
  voxel.color.a = interpMember(q_vector, voxels, &getAlpha);

  return voxel;
}

}  // namespace voxblox

#endif  // VOXBLOX_INTERPOLATOR_INTERPOLATOR_INL_H_

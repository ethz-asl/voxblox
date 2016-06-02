#ifndef VOXBLOX_INTERPOLATOR_INL_H_
#define VOXBLOX_INTERPOLATOR_INL_H_

namespace voxblox {

Interpolator::Interpolator(const Layer<TsdfVoxel>& tsdf_layer)
    : tsdf_layer_(tsdf_layer) {}

bool Interpolator::getDistance(const Point& pos, FloatingPoint* distance,
                               bool interpolate) const {
  if (interpolate) {
    return getNearestDistance(pos, distance);
  } else {
    return getNearestDistance(pos, distance);
  }
}

bool Interpolator::getGradient(const Point& pos, Point* grad,
                               const bool interpolate) const {
  CHECK_NOTNULL(grad);

  Layer<TsdfVoxel>::BlockType::ConstPtr block_ptr;
  // = tsdf_layer_.getBlockPtrByCoordinates(pos);
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

bool Interpolator::setIndexes(const Point& pos, BlockIndex* block_index,
                              InterpIndexes* voxel_indexes) const {
  // get voxel index
  *block_index = tsdf_layer_.computeBlockIndexFromCoordinates(pos);
  Layer<TsdfVoxel>::BlockType::ConstPtr block_ptr = tsdf_layer_.getBlockPtrByIndex(*block_index);
  if (block_ptr == nullptr) {
    return false;
  }
  VoxelIndex voxel_index = block_ptr->computeVoxelIndexFromCoordinates(pos);

  // shift index to bottom left corner voxel (makes math easier)
  Point center_offset =
      pos - block_ptr->computeCoordinatesFromVoxelIndex(voxel_index);
  //cast is vital to allow negative indexes
  voxel_indexes->colwise() = voxel_index.cast<typename InterpIndexes::RealScalar>().array() - (center_offset.array() < 0);

  // get indexes of neighbors
  InterpIndexes voxel_indexes_;
  voxel_indexes_ << 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1,
      1, 0, 0, 0, 0, 1, 1, 1, 1;

  *voxel_indexes += voxel_index.array();
  return true;
}

void Interpolator::getWeightsVector(const Point& voxel_pos, const Point& pos,
                                    InterpVector* weights_vector) const {
  DCHECK_NOTNULL(weights_vector);

  Point voxel_offset = pos - voxel_pos;
  DCHECK((voxel_offset.array() > 0).all());

  *weights_vector << 1, voxel_offset[0], voxel_offset[1], voxel_offset[2],
      voxel_offset[0] * voxel_offset[1], voxel_offset[1] * voxel_offset[2],
      voxel_offset[2] * voxel_offset[0],
      voxel_offset[0] * voxel_offset[1] * voxel_offset[2];
}

bool Interpolator::getDistancesAndWeights(const BlockIndex& block_index,
                                          const InterpIndexes& voxel_indexes,
                                          const Point& pos,
                                          InterpVector* distances,
                                          InterpVector* weights_vector) const {
  DCHECK_NOTNULL(distances);

  // for each voxel index
  for (size_t i = 0; i < voxel_indexes.cols(); ++i) {
    Layer<TsdfVoxel>::BlockType::Ptr block_ptr;
    VoxelIndex voxel_index;
    // if voxel index is negative get neighboring block and update index
    if ((voxel_indexes.col(i) < 0).any()) {
      BlockIndex new_block_index;
      new_block_index.array() = block_index.array() - (voxel_indexes.col(i) < 0);
      block_ptr = tsdf_layer_.getBlockPtrByIndex(new_block_index);
      if (block_ptr == nullptr) {
        return false;
      }
      voxel_index.array() = voxel_indexes.col(i) +
                    block_ptr->voxels_per_side() * (voxel_index.col(i).array() > 0);
    } else {
      block_ptr = tsdf_layer_.getBlockPtrByIndex(block_index);
      voxel_index.array() = voxel_indexes.col(i);
      DCHECK_NOTNULL(block_ptr);
    }
    // use bottom left corner voxel to compute weights vector
    if (i == 0) {
      getWeightsVector(block_ptr->computeCoordinatesFromVoxelIndex(voxel_index),
                       pos, weights_vector);
    }
    (*distances)[i] = block_ptr->getVoxelByVoxelIndex(voxel_index).distance;
  }
  return true;
}

bool Interpolator::getInterpDistance(const Point& pos,
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
  if (!getDistancesAndWeights(block_index, voxel_indexes, pos,
                              &distances, &weights_vector)) {
    return false;
  }

  // table for tri-linear interpolation
  InterpTable interp_table;
  interp_table << 1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 1,
      0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, -1, 1, 0, 0, 0, 0, 0, 0, 1, 0, -1, 0,
      -1, 0, 1, 0, 1, -1, -1, 1, 0, 0, 0, 0, 1, -1, 0, 0, -1, 1, 0, 0, -1, 1, 1,
      -1, 1, -1, -1, 1;

  // interpolate
  *distance = weights_vector * (interp_table * distances.transpose());
}

bool Interpolator::getNearestDistance(const Point& pos,
                                      FloatingPoint* distance) const {
  CHECK_NOTNULL(distance);

  Layer<TsdfVoxel>::BlockType::ConstPtr block_ptr = tsdf_layer_.getBlockPtrByCoordinates(pos);
  if (block_ptr == nullptr) {
    return false;
  }
  *distance = block_ptr->getVoxelByCoordinates(pos).distance;

  return true;
}

#endif  // VOXBLOX_INTERPOLATOR_INL_H_
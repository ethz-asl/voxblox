#ifndef VOXBLOX_INTERPOLATOR_INL_H_
#define VOXBLOX_INTERPOLATOR_INL_H_

namespace voxblox {

Interpolator::Interpolator(Layer<TsdfVoxel>* tsdf_layer)
    : tsdf_layer_(tsdf_layer) {}

bool Interpolator::getDistance(const Point& pos, FloatingPoint* distance,
                               bool interpolate) const {
  if (interpolate) {
    return getInterpDistance(pos, distance);
  } else {
    return getNearestDistance(pos, distance);
  }
}

bool Interpolator::getGradient(const Point& pos, Point* grad,
                               const bool interpolate) const {
  CHECK_NOTNULL(grad);

  Layer<TsdfVoxel>::BlockType::ConstPtr block_ptr =
      tsdf_layer_->getBlockPtrByCoordinates(pos);
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
  *block_index = tsdf_layer_->computeBlockIndexFromCoordinates(pos);
  Layer<TsdfVoxel>::BlockType::ConstPtr block_ptr =
      tsdf_layer_->getBlockPtrByIndex(*block_index);
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
  (*voxel_indexes) << 0, 1, 0, 1, 0, 1, 0, 1, 
                      0, 0, 1, 1, 0, 0, 1, 1,
                      0, 0, 0, 0, 1, 1, 1, 1;

  voxel_indexes->colwise() += voxel_index.array();
  return true;
}

void Interpolator::getWeightsVector(const Point& voxel_pos, const Point& pos,
                                    InterpVector* weights_vector) const {
  CHECK_NOTNULL(weights_vector);

  Point voxel_offset = pos - voxel_pos;

  CHECK((voxel_offset.array() >= 0).all());

  *weights_vector << 1, 
                     voxel_offset[0], 
                     voxel_offset[1], 
                     voxel_offset[2],
                     voxel_offset[0] * voxel_offset[1], 
                     voxel_offset[1] * voxel_offset[2],
                     voxel_offset[2] * voxel_offset[0],
                     voxel_offset[0] * voxel_offset[1] * voxel_offset[2];
}

bool Interpolator::getDistancesAndWeights(const BlockIndex& block_index,
                                          const InterpIndexes& voxel_indexes,
                                          const Point& pos,
                                          InterpVector* distances,
                                          InterpVector* weights_vector) const {
  CHECK_NOTNULL(distances);

  // for each voxel index
  for (size_t i = 0; i < voxel_indexes.cols(); ++i) {
    Layer<TsdfVoxel>::BlockType::Ptr block_ptr =
        tsdf_layer_->getBlockPtrByIndex(block_index);
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
      block_ptr = tsdf_layer_->getBlockPtrByIndex(new_block_index);
      if (block_ptr == nullptr) {
        return false;
      }
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
  if (!getDistancesAndWeights(block_index, voxel_indexes, pos, &distances,
                              &weights_vector)) {
    return false;
  }

  // table for tri-linear interpolation (http://spie.org/samples/PM159.pdf)
  InterpTable interp_table;
  interp_table << 1, 0, 0, 0, 0, 0, 0, 0,
                 -1, 0, 0, 0, 1, 0, 0, 0,
                 -1, 0, 1, 0, 0, 0, 0, 0,
                 -1, 1, 0, 0, 0, 0, 0, 0,
                  1, 0,-1, 0,-1, 0, 1, 0,
                  1,-1,-1, 1, 0, 0, 0, 0,
                  1,-1, 0, 0,-1, 1, 0, 0,
                 -1, 1, 1,-1, 1,-1,-1, 1;

  // interpolate
  *distance = weights_vector * (interp_table * distances.transpose());
  return true;
}

bool Interpolator::getNearestDistance(const Point& pos,
                                      FloatingPoint* distance) const {
  CHECK_NOTNULL(distance);

  Layer<TsdfVoxel>::BlockType::ConstPtr block_ptr =
      tsdf_layer_->getBlockPtrByCoordinates(pos);
  if (block_ptr == nullptr) {
    return false;
  }
  *distance = block_ptr->getVoxelByCoordinates(pos).distance;

  return true;
}

}  // namespace voxblox

#endif  // VOXBLOX_INTERPOLATOR_INL_H_
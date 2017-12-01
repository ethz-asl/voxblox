#ifndef VOXBLOX_INTEGRATOR_MERGE_INTEGRATION_H_
#define VOXBLOX_INTEGRATOR_MERGE_INTEGRATION_H_

#include <algorithm>
#include <vector>

#include <glog/logging.h>

#include "voxblox/interpolator/interpolator.h"

#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

static const FloatingPoint kUnitCubeDiagonalLength = std::sqrt(3.0);

template <typename VoxelType>
void mergeVoxelAIntoVoxelB(const VoxelType& voxel_A, VoxelType* voxel_B);

template <>
void mergeVoxelAIntoVoxelB(const TsdfVoxel& voxel_A, TsdfVoxel* voxel_B);

template <>
void mergeVoxelAIntoVoxelB(const OccupancyVoxel& voxel_A,
                           OccupancyVoxel* voxel_B);

template <typename VoxelType>
void mergeBlockAIntoBlockB(const Block<VoxelType>& block_A,
                           Block<VoxelType>* block_B) {
  CHECK_EQ(block_A.voxel_size(), block_B->voxel_size());
  CHECK_EQ(block_A.voxels_per_side(), block_B->voxels_per_side());
  CHECK_NOTNULL(block_B);

  if (!block_A.has_data()) {
    return;
  } else {
    block_B->has_data() = true;
    block_B->updated() = true;

    for (IndexElement voxel_idx = 0; voxel_idx < block_B->num_voxels();
         ++voxel_idx) {
      mergeVoxelAIntoVoxelB<VoxelType>(
          block_A.getVoxelByLinearIndex(voxel_idx),
          &(block_B->getVoxelByLinearIndex(voxel_idx)));
    }
  }
}

// Merges layers, when the voxel or block size differs resampling occurs.
template <typename VoxelType>
void mergeLayerAintoLayerB(const Layer<VoxelType>& layer_A,
                           Layer<VoxelType>* layer_B) {
  CHECK_NOTNULL(layer_B);
  // if voxel layout is different resample layer A to match B
  const Layer<VoxelType>* layer_A_ptr;
  Layer<VoxelType> layer_A_resampled(layer_B->voxel_size(),
                                     layer_B->voxels_per_side());

  if ((layer_A.voxel_size() != layer_B->voxel_size()) ||
      (layer_A.voxels_per_side() != layer_B->voxels_per_side())) {
    resampleLayer(layer_A, &layer_A_resampled);
    layer_A_ptr = &layer_A_resampled;
  } else {
    layer_A_ptr = &layer_A;
  }

  BlockIndexList block_idx_list_A;
  layer_A.getAllAllocatedBlocks(&block_idx_list_A);

  for (const BlockIndex& block_idx : block_idx_list_A) {
    typename Block<VoxelType>::ConstPtr block_A_ptr =
        layer_A_ptr->getBlockPtrByIndex(block_idx);
    typename Block<VoxelType>::Ptr block_B_ptr =
        layer_B->getBlockPtrByIndex(block_idx);

    if (!block_B_ptr) {
      block_B_ptr = layer_B->allocateBlockPtrByIndex(block_idx);
    }

    if ((block_A_ptr != nullptr) && (block_B_ptr != nullptr)) {
      mergeBlockAIntoBlockB(*block_A_ptr, block_B_ptr.get());
    }
  }
}

// Performs a 3D transformation on layer A before merging it. See
// transformLayer for details
template <typename VoxelType>
void mergeLayerAintoLayerB(const Layer<VoxelType>& layer_A,
                           const Transformation& T_A_B,
                           Layer<VoxelType>* layer_B,
                           bool use_naive_method = false) {
  Layer<VoxelType> layer_A_transformed(layer_B->voxel_size(),
                                       layer_B->voxels_per_side());

  if (use_naive_method) {
    naiveTransformLayer(layer_A, T_A_B, &layer_A_transformed);
  } else {
    transformLayer(layer_A, T_A_B, &layer_A_transformed);
  }

  mergeLayerAintoLayerB(layer_A_transformed, layer_B);
}

// copies the information stored in layer_in into layer_out resampling the
// data so that it fits the voxel and block size of the output layer
template <typename VoxelType>
void resampleLayer(const Layer<VoxelType>& layer_in,
                   Layer<VoxelType>* layer_out) {
  CHECK_NOTNULL(layer_out);
  transformLayer(layer_in, Transformation(), layer_out);
}

// Similar to transformLayer in functionality, however the system only makes
// use of the forward transform and nearest neighbor interpolation. This will
// result in artifacts and other issues in the result, however it should be
// several orders of magnitude faster.
template <typename VoxelType>
void naiveTransformLayer(const Layer<VoxelType>& layer_in,
                         const Transformation& T_in_out,
                         Layer<VoxelType>* layer_out) {
  BlockIndexList block_idx_list_in;
  layer_in.getAllAllocatedBlocks(&block_idx_list_in);

  Interpolator<VoxelType> interpolator(&layer_in);

  for (const BlockIndex& block_idx : block_idx_list_in) {
    const Block<VoxelType>& input_block = layer_in.getBlockByIndex(block_idx);

    for (IndexElement input_linear_voxel_idx = 0;
         input_linear_voxel_idx <
         static_cast<IndexElement>(input_block.num_voxels());
         ++input_linear_voxel_idx) {
      const VoxelType& input_voxel =
          input_block.getVoxelByLinearIndex(input_linear_voxel_idx);

      // find voxel centers location in the output
      const Point voxel_center =
          T_in_out *
          input_block.computeCoordinatesFromLinearIndex(input_linear_voxel_idx);

      const VoxelIndex global_output_voxel_idx =
          getGridIndexFromPoint(voxel_center, layer_out->voxel_size_inv());

      // allocate it in the output
      typename Block<VoxelType>::Ptr output_block =
          layer_out->allocateBlockPtrByIndex(getBlockIndexFromGlobalVoxelIndex(
              global_output_voxel_idx, layer_out->voxels_per_side_inv()));

      if (output_block == nullptr) {
        std::cerr << "invalid block" << std::endl;
      }

      // get the output voxel
      VoxelType& output_voxel =
          output_block->getVoxelByVoxelIndex(getLocalFromGlobalVoxelIndex(
              global_output_voxel_idx, layer_out->voxels_per_side()));

      if (interpolator.getVoxel(voxel_center, &output_voxel, false)) {
        output_block->has_data() = true;
      }
    }
  }
}

// Performs a 3D transform on the input layer and writes the results to the
// output layer. During the transformation resampling occurs so that the voxel
// and block size of the input and output layer can differ.
template <typename VoxelType>
void transformLayer(const Layer<VoxelType>& layer_in,
                    const Transformation& T_in_out,
                    Layer<VoxelType>* layer_out) {
  CHECK_NOTNULL(layer_out);

  // first mark all the blocks in the output layer that may be filled by the
  // input layer (we are conservative here approximating the input blocks as
  // spheres of diameter sqrt(3)*block_size)
  IndexSet block_idx_set;

  BlockIndexList block_idx_list_in;
  layer_in.getAllAllocatedBlocks(&block_idx_list_in);

  for (const BlockIndex& block_idx : block_idx_list_in) {
    Point center =
        getCenterPointFromGridIndex(block_idx, layer_in.block_size());

    // forwards transform of center
    center = T_in_out * center;

    // Furthest center point of neighboring blocks.
    FloatingPoint offset =
        kUnitCubeDiagonalLength * layer_in.block_size() * 0.5;

    // Add index of all blocks in range to set.
    for (FloatingPoint x = center.x() - offset; x < center.x() + offset;
         x += layer_out->block_size()) {
      for (FloatingPoint y = center.y() - offset; y < center.y() + offset;
           y += layer_out->block_size()) {
        for (FloatingPoint z = center.z() - offset; z < center.z() + offset;
             z += layer_out->block_size()) {
          Point current_center = Point(x, y, z);
          BlockIndex current_idx = getGridIndexFromPoint(
              current_center, 1.0f / layer_out->block_size());
          block_idx_set.insert(current_idx);
        }
      }
    }
  }

  // get inverse transform
  Transformation T_out_in = T_in_out.inverse();

  Interpolator<VoxelType> interpolator(&layer_in);

  // we now go through all the blocks in the output layer and interpolate the
  // input layer at the center of each output voxel position
  for (const BlockIndex& block_idx : block_idx_set) {
    typename Block<VoxelType>::Ptr block =
        layer_out->allocateBlockPtrByIndex(block_idx);

    for (IndexElement voxel_idx = 0;
          voxel_idx < static_cast<IndexElement>(block->num_voxels());
         ++voxel_idx) {
      VoxelType& voxel = block->getVoxelByLinearIndex(voxel_idx);

      // find voxel centers location in the input
      const Point voxel_center =
          T_out_in * block->computeCoordinatesFromLinearIndex(voxel_idx);

      // interpolate voxel
      if (interpolator.getVoxel(voxel_center, &voxel, true)) {
        block->has_data() = true;

        // if interpolated value fails use nearest
      } else if (interpolator.getVoxel(voxel_center, &voxel, false)) {
        block->has_data() = true;
      }
    }

    if (!block->has_data()) {
      layer_out->removeBlock(block_idx);
    }
  }
}

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_MERGE_INTEGRATION_H_

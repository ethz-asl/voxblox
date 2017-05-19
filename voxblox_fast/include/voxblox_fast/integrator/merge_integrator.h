#ifndef VOXBLOX_FAST_INTEGRATOR_MERGE_INTEGRATOR_H_
#define VOXBLOX_FAST_INTEGRATOR_MERGE_INTEGRATOR_H_

#include <algorithm>
#include <vector>

#include <glog/logging.h>

#include "voxblox_fast/core/common.h"
#include "voxblox_fast/core/layer.h"
#include "voxblox_fast/core/voxel.h"
#include "voxblox_fast/interpolator/interpolator.h"

namespace voxblox_fast {

class MergeIntegrator {
 public:
  // all methods are static so the class does not need a constructor
  MergeIntegrator() = delete;

  template <typename VoxelType>
  static void mergeVoxelAIntoVoxelB(const VoxelType& voxel_A,
                                    VoxelType* voxel_B) {
    LOG(FATAL) << "VOXEL MERGING NOT IMPLEMENTED FOR CURRENT VOXEL TYPE";
  }

  template <typename VoxelType>
  static void MergeBlockAIntoBlockB(const Block<VoxelType>& block_A,
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
  static void MergeLayerAintoLayerB(const Layer<VoxelType>& layer_A,
                                    Layer<VoxelType>* layer_B) {
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

      MergeBlockAIntoBlockB(*block_A_ptr, block_B_ptr.get());
    }
  }

  // Performs a 3D transformation on layer A before merging it. See
  // transformLayer for details
  template <typename VoxelType>
  static void MergeLayerAintoLayerB(const Layer<VoxelType>& layer_A,
                                    const Transformation& T_A_B,
                                    Layer<VoxelType>* layer_B) {
    Layer<VoxelType> layer_A_transformed(layer_B->voxel_size(),
                                         layer_B->voxels_per_side());
    transformLayer(layer_A, T_A_B, &layer_A_transformed);

    MergeLayerAintoLayerB(layer_A_transformed, layer_B);
  }

  // copies the information stored in layer_in into layer_out resampling the
  // data so that it fits the voxel and block size of the output layer
  template <typename VoxelType>
  static void resampleLayer(const Layer<VoxelType>& layer_in,
                            Layer<VoxelType>* layer_out) {
    CHECK_NOTNULL(layer_out);
    transformLayer(layer_in, Transformation(), layer_out);
  }

  // Performs a 3D transform on the input layer and writes the results to the
  // output layer. During the transformation resampling occurs so that the voxel
  // and block size of the input and output layer can differ.
  template <typename VoxelType>
  static void transformLayer(const Layer<VoxelType>& layer_in,
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

      // furthest point that could possibly be inside a rotated input block
      FloatingPoint offset = kUnitCubeDiagonalLength * layer_in.block_size();

      // add index of all blocks in range to set
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
    for (BlockIndex block_idx : block_idx_set) {
      typename Block<VoxelType>::Ptr block =
          layer_out->allocateBlockPtrByIndex(block_idx);

      for (IndexElement voxel_idx = 0; voxel_idx < block->num_voxels();
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

 private:
  static const FloatingPoint kUnitCubeDiagonalLength;
};

const FloatingPoint MergeIntegrator::kUnitCubeDiagonalLength =
    std::sqrt(3.0);

template <>
void MergeIntegrator::mergeVoxelAIntoVoxelB(const TsdfVoxel& voxel_A,
                                            TsdfVoxel* voxel_B) {
  float combined_weight = voxel_A.weight + voxel_B->weight;
  if (combined_weight > 0) {
    voxel_B->distance = (voxel_A.distance * voxel_A.weight +
                         voxel_B->distance * voxel_B->weight) /
                        combined_weight;

    voxel_B->color = Color::blendTwoColors(voxel_A.color, voxel_A.weight,
                                           voxel_B->color, voxel_B->weight);

    voxel_B->weight = combined_weight;
  }
}

template <>
void MergeIntegrator::mergeVoxelAIntoVoxelB(const OccupancyVoxel& voxel_A,
                                            OccupancyVoxel* voxel_B) {
  voxel_B->probability_log += voxel_A.probability_log;
  voxel_B->observed = voxel_B->observed || voxel_A.observed;
}

}  // namespace voxblox

#endif  // VOXBLOX_FAST_INTEGRATOR_MERGE_INTEGRATOR_H_

#ifndef VOXBLOX_SIMULATION_SIMULATION_WORLD_INL_H_
#define VOXBLOX_SIMULATION_SIMULATION_WORLD_INL_H_

#include <algorithm>

#include "voxblox/core/block.h"

namespace voxblox {

template <typename VoxelType>
void SimulationWorld::generateSdfFromWorld(FloatingPoint max_dist,
                                           Layer<VoxelType>* layer) const {
  CHECK_NOTNULL(layer);
  // Iterate over every voxel in the layer and compute its distance to all
  // objects.

  // Get all blocks within bounds. For now, only respect bounds approximately:
  // that is, up to block boundaries.
  FloatingPoint block_size = layer->block_size();

  BlockIndexList blocks;
  for (FloatingPoint x = min_bound_.x(); x <= max_bound_.x(); x += block_size) {
    for (FloatingPoint y = min_bound_.y(); y <= max_bound_.y();
         y += block_size) {
      for (FloatingPoint z = min_bound_.z(); z <= max_bound_.z();
           z += block_size) {
        blocks.push_back(
            layer->computeBlockIndexFromCoordinates(Point(x, y, z)));
      }
    }
  }

  // Iterate over all blocks filling this stuff in.
  for (const BlockIndex& block_index : blocks) {
    typename Block<VoxelType>::Ptr block =
        layer->allocateBlockPtrByIndex(block_index);
    for (size_t i = 0; i < block->num_voxels(); ++i) {
      VoxelType& voxel = block->getVoxelByLinearIndex(i);
      Point coords = block->computeCoordinatesFromLinearIndex(i);

      // Iterate over all objects and get distances to this thing.
      FloatingPoint voxel_dist = max_dist;
      for (size_t j = 0; j < objects_.size(); ++j) {
        voxel_dist =
            std::min(max_dist, objects_[j]->getDistanceToPoint(coords));
      }

      // Then update the thing.
      setVoxel(voxel_dist, &voxel);
    }
  }
}

template <>
void SimulationWorld::setVoxel(FloatingPoint dist, TsdfVoxel* voxel) const {
  voxel->distance = static_cast<float>(dist);
  voxel->weight = 1.0f;  // Just to make sure it gets visualized/meshed/etc.
}

template <>
void SimulationWorld::setVoxel(FloatingPoint dist, EsdfVoxel* voxel) const {
  voxel->distance = static_cast<float>(dist);
  voxel->observed = true;
}

}  // namespace voxblox

#endif  // VOXBLOX_SIMULATION_SIMULATION_WORLD_INL_H_

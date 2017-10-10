#ifndef VOXBLOX_TEST_LAYER_UTILS_H_
#define VOXBLOX_TEST_LAYER_UTILS_H_

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

namespace voxblox {
namespace utils {

template <typename VoxelType>
bool isSameVoxel(const VoxelType& voxel_A, const VoxelType& voxel_B) {
  LOG(FATAL) << "Not implemented for this voxel type!";
  return false;
}

template <typename VoxelType>
bool isSameBlock(const Block<VoxelType>& block_A,
                 const Block<VoxelType>& block_B) {
  bool is_the_same = true;

  constexpr double kTolerance = 1e-10;

  is_the_same &=
      std::abs(block_A.voxel_size() - block_B.voxel_size()) < kTolerance;
  is_the_same &=
      std::abs(block_A.block_size() - block_B.block_size()) < kTolerance;
  is_the_same &= block_A.voxels_per_side() == block_B.voxels_per_side();

  is_the_same &=
      std::abs(block_A.origin().x() - block_B.origin().x()) < kTolerance;
  is_the_same &=
      std::abs(block_A.origin().y() - block_B.origin().y()) < kTolerance;
  is_the_same &=
      std::abs(block_A.origin().z() - block_B.origin().z()) < kTolerance;

  is_the_same &= block_A.num_voxels() == block_B.num_voxels();

  for (size_t voxel_idx = 0u; voxel_idx < block_A.num_voxels(); ++voxel_idx) {
    is_the_same &= isSameVoxel(block_A.getVoxelByLinearIndex(voxel_idx),
                               block_B.getVoxelByLinearIndex(voxel_idx));
  }
  return is_the_same;
}

template <typename VoxelType>
bool isSameLayer(const Layer<VoxelType>& layer_A,
                 const Layer<VoxelType>& layer_B) {
  constexpr double kTolerance = 1e-10;

  bool is_the_same = true;

  is_the_same &=
      std::abs(layer_A.voxel_size() - layer_B.voxel_size()) < kTolerance;
  is_the_same &=
      std::abs(layer_A.block_size() - layer_B.block_size()) < kTolerance;

  is_the_same &= layer_A.voxels_per_side() == layer_B.voxels_per_side();
  is_the_same &= layer_A.getNumberOfAllocatedBlocks() ==
                 layer_B.getNumberOfAllocatedBlocks();

  BlockIndexList blocks_A, blocks_B;
  layer_A.getAllAllocatedBlocks(&blocks_A);
  layer_B.getAllAllocatedBlocks(&blocks_B);
  is_the_same &= blocks_A.size() == blocks_B.size();

  for (const BlockIndex& index_A : blocks_A) {
    const BlockIndexList::const_iterator it =
        std::find(blocks_B.begin(), blocks_B.end(), index_A);
    if (it != blocks_B.end()) {
      const Block<VoxelType>& block_A = layer_A.getBlockByIndex(index_A);
      const Block<VoxelType>& block_B = layer_B.getBlockByIndex(*it);
      bool is_same_block = isSameBlock(block_A, block_B);
      LOG_IF(ERROR, !is_same_block)
          << "Block at index [" << index_A.transpose()
          << "] in layer_A is not the same as in layer_B";
      is_the_same &= is_same_block;
    } else {
      LOG(ERROR) << "Block at index [" << index_A.transpose()
                 << "] in layer_A does not exists in layer_B";
      return false;
    }
  }
  for (const BlockIndex& index_B : blocks_B) {
    const BlockIndexList::const_iterator it =
        std::find(blocks_A.begin(), blocks_A.end(), index_B);
    if (it != blocks_A.end()) {
      const Block<VoxelType>& block_B = layer_A.getBlockByIndex(index_B);
      const Block<VoxelType>& block_A = layer_B.getBlockByIndex(*it);
      bool is_same_block = isSameBlock(block_B, block_A);
      LOG_IF(ERROR, !is_same_block)
          << "Block at index [" << index_B.transpose()
          << "] in layer_B is not the same as in layer_A";
      is_the_same &= is_same_block;
    } else {
      LOG(ERROR) << "Block at index [" << index_B.transpose()
                 << "] in layer_B does not exists in layer_A";
      return false;
    }
  }

  is_the_same &= layer_A.getMemorySize() == layer_B.getMemorySize();
  return is_the_same;
}

template <>
bool isSameVoxel(const TsdfVoxel& voxel_A, const TsdfVoxel& voxel_B);

template <>
bool isSameVoxel(const EsdfVoxel& voxel_A, const EsdfVoxel& voxel_B);

template <>
bool isSameVoxel(const OccupancyVoxel& voxel_A, const OccupancyVoxel& voxel_B);

}  // namespace utils
}  // namespace voxblox

#endif  // VOXBLOX_TEST_LAYER_UTILS_H_

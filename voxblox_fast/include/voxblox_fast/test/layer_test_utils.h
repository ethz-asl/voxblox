#ifndef VOXBLOX_FAST_TEST_LAYER_TEST_UTILS_H_
#define VOXBLOX_FAST_TEST_LAYER_TEST_UTILS_H_

#include <gtest/gtest.h>

#include "voxblox_fast/core/layer.h"
#include "voxblox_fast/core/voxel.h"

namespace voxblox_fast {
namespace test {

template <typename VoxelType>
class LayerTest {
 public:
  void CompareLayers(const Layer<VoxelType>& layer_A,
                     const Layer<VoxelType>& layer_B) const {
    EXPECT_NEAR(layer_A.voxel_size(), layer_B.voxel_size(), kTolerance);
    EXPECT_NEAR(layer_A.block_size(), layer_B.block_size(), kTolerance);
    EXPECT_EQ(layer_A.voxels_per_side(), layer_B.voxels_per_side());
    EXPECT_EQ(layer_A.getNumberOfAllocatedBlocks(),
              layer_B.getNumberOfAllocatedBlocks());

    BlockIndexList blocks_A, blocks_B;
    layer_A.getAllAllocatedBlocks(&blocks_A);
    layer_B.getAllAllocatedBlocks(&blocks_B);
    EXPECT_EQ(blocks_A.size(), blocks_B.size());
    for (const BlockIndex& index_A : blocks_A) {
      BlockIndexList::const_iterator it =
          std::find(blocks_B.begin(), blocks_B.end(), index_A);
      if (it != blocks_B.end()) {
        const Block<VoxelType>& block_A = layer_A.getBlockByIndex(index_A);
        const Block<VoxelType>& block_B = layer_B.getBlockByIndex(*it);
        CompareBlocks(block_A, block_B);
      } else {
        ADD_FAILURE();
        LOG(ERROR) << "Block at index [" << index_A.transpose()
                   << "] in layer_A does not exists in layer_B";
      }
    }
    for (const BlockIndex& index_B : blocks_B) {
      BlockIndexList::const_iterator it =
          std::find(blocks_A.begin(), blocks_A.end(), index_B);
      if (it != blocks_A.end()) {
        const Block<VoxelType>& block_B = layer_A.getBlockByIndex(index_B);
        const Block<VoxelType>& block_A = layer_B.getBlockByIndex(*it);
        CompareBlocks(block_B, block_A);
      } else {
        ADD_FAILURE();
        LOG(ERROR) << "Block at index [" << index_B.transpose()
                   << "] in layer_B does not exists in layer_A";
      }
    }

    EXPECT_EQ(layer_A.getMemorySize(), layer_B.getMemorySize());
  }

  void CompareBlocks(const Block<VoxelType>& block_A,
                     const Block<VoxelType>& block_B) const {
    EXPECT_NEAR(block_A.voxel_size(), block_B.voxel_size(), kTolerance);
    EXPECT_NEAR(block_A.block_size(), block_B.block_size(), kTolerance);
    EXPECT_EQ(block_A.voxels_per_side(), block_B.voxels_per_side());

    EXPECT_NEAR(block_A.origin().x(), block_B.origin().x(), kTolerance);
    EXPECT_NEAR(block_A.origin().y(), block_B.origin().y(), kTolerance);
    EXPECT_NEAR(block_A.origin().z(), block_B.origin().z(), kTolerance);

    EXPECT_EQ(block_A.num_voxels(), block_B.num_voxels());
    for (size_t voxel_idx = 0u; voxel_idx < block_A.num_voxels(); ++voxel_idx) {
      CompareVoxel(block_A.getVoxelByLinearIndex(voxel_idx),
                   block_B.getVoxelByLinearIndex(voxel_idx));
    }
  }

  void CompareVoxel(const VoxelType& voxel_A, const VoxelType& voxel_B) const;

  static constexpr double kTolerance = 1e-10;
};

template <>
void LayerTest<TsdfVoxel>::CompareVoxel(const TsdfVoxel& voxel_A,
                                        const TsdfVoxel& voxel_B) const {
  CHECK_NEAR(voxel_A.distance, voxel_B.distance, kTolerance);
  CHECK_NEAR(voxel_A.weight, voxel_B.weight, kTolerance);
  CHECK_EQ(voxel_A.color.r, voxel_B.color.r);
  CHECK_EQ(voxel_A.color.g, voxel_B.color.g);
  CHECK_EQ(voxel_A.color.b, voxel_B.color.b);
  CHECK_EQ(voxel_A.color.a, voxel_B.color.a);
}

}  // namespace test
}  // namespace voxblox

#endif  // VOXBLOX_FAST_TEST_LAYER_TEST_UTILS_H_

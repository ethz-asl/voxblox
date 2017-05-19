#ifndef HTWFSC_LAYER_TEST_UTILS_H_
#define HTWFSC_LAYER_TEST_UTILS_H_

#include <gtest/gtest.h>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

#include "voxblox_fast/core/layer.h"
#include "voxblox_fast/core/voxel.h"

namespace htwfsc_benchmarks {
namespace test {

template <typename VoxelType, typename VoxelTypeB>
class LayerTest {
 public:
  void CompareLayers(const voxblox::Layer<VoxelType>& layer_A,
                     const voxblox_fast::Layer<VoxelTypeB>& layer_B) const {
    EXPECT_NEAR(layer_A.voxel_size(), layer_B.voxel_size(), kTolerance);
    EXPECT_NEAR(layer_A.block_size(), layer_B.block_size(), kTolerance);
    EXPECT_EQ(layer_A.voxels_per_side(), layer_B.voxels_per_side());
    EXPECT_EQ(layer_A.getNumberOfAllocatedBlocks(),
              layer_B.getNumberOfAllocatedBlocks());

    voxblox::BlockIndexList blocks_A;
    voxblox_fast::BlockIndexList blocks_B;
    layer_A.getAllAllocatedBlocks(&blocks_A);
    layer_B.getAllAllocatedBlocks(&blocks_B);
    EXPECT_EQ(blocks_A.size(), blocks_B.size());
    for (const voxblox::BlockIndex& index_A : blocks_A) {
      voxblox_fast::BlockIndexList::const_iterator it =
          std::find(blocks_B.begin(), blocks_B.end(), index_A);
      if (it != blocks_B.end()) {
        const voxblox::Block<VoxelType>& block_A = layer_A.getBlockByIndex(index_A);
        const voxblox_fast::Block<VoxelTypeB>& block_B = layer_B.getBlockByIndex(*it);
        CompareBlocks(block_A, block_B);
      } else {
        ADD_FAILURE();
        LOG(ERROR) << "Block at index [" << index_A.transpose()
                   << "] in layer_A does not exists in layer_B";
      }
    }
    for (const voxblox_fast::BlockIndex& index_B : blocks_B) {
      voxblox::BlockIndexList::const_iterator it =
          std::find(blocks_A.begin(), blocks_A.end(), index_B);
      if (it != blocks_A.end()) {
        const voxblox::Block<VoxelType>& block_B = layer_A.getBlockByIndex(index_B);
        const voxblox_fast::Block<VoxelTypeB>& block_A = layer_B.getBlockByIndex(*it);
        CompareBlocks(block_B, block_A);
      } else {
        ADD_FAILURE();
        LOG(ERROR) << "Block at index [" << index_B.transpose()
                   << "] in layer_B does not exists in layer_A";
      }
    }

    EXPECT_EQ(layer_A.getMemorySize(), layer_B.getMemorySize());
  }

  void CompareBlocks(const voxblox::Block<VoxelType>& block_A,
                     const voxblox_fast::Block<VoxelTypeB>& block_B) const {
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

  void CompareVoxel(const VoxelType& voxel_A, const VoxelTypeB& voxel_B) const;

  static constexpr double kTolerance = 1e-10;
};

template <>
void LayerTest<voxblox::TsdfVoxel, voxblox_fast::TsdfVoxel>::CompareVoxel(const voxblox::TsdfVoxel& voxel_A,
                                        const voxblox_fast::TsdfVoxel& voxel_B) const {
  CHECK_NEAR(voxel_A.distance, voxel_B.distance, kTolerance);
  CHECK_NEAR(voxel_A.weight, voxel_B.weight, kTolerance);
  CHECK_EQ(voxel_A.color.r, voxel_B.color.r);
  CHECK_EQ(voxel_A.color.g, voxel_B.color.g);
  CHECK_EQ(voxel_A.color.b, voxel_B.color.b);
  CHECK_EQ(voxel_A.color.a, voxel_B.color.a);
}

}  // namespace test
}  // namespace htwfsc_benchmarks

#endif  // HTWFSC_LAYER_TEST_UTILS_H_

#include <iostream>
#include <eigen-checks/gtest.h>
#include <eigen-checks/entrypoint.h>

#include "voxblox/core/tsdf_map.h"

using namespace voxblox;

class TsdfMapTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    TsdfMap::Config config;
    config.tsdf_voxel_size = 0.1;
    config.tsdf_voxels_per_side = 8;
    map_ = std::make_shared<TsdfMap>(config);
  }

  TsdfMap::Ptr map_;
};

TEST_F(TsdfMapTest, BlockAllocation) {
  // Should have no blocks by default.
  EXPECT_EQ(0, map_->getTsdfLayer().getNumberOfAllocatedBlocks());
  map_->getTsdfLayerPtr()->allocateNewBlockByCoordinates(Point(0.0, 0.15, 0.0));
  EXPECT_EQ(1, map_->getTsdfLayerPtr()->getNumberOfAllocatedBlocks());
  map_->getTsdfLayerPtr()->allocateNewBlockByCoordinates(Point(0.0, 0.13, 0.0));
  EXPECT_EQ(1, map_->getTsdfLayerPtr()->getNumberOfAllocatedBlocks());
  map_->getTsdfLayerPtr()->allocateNewBlockByCoordinates(Point(-10.0, 13.5, 20.0));
  EXPECT_EQ(2, map_->getTsdfLayerPtr()->getNumberOfAllocatedBlocks());
}

TEST_F(TsdfMapTest, IndexLookups) {
  TsdfBlock::Ptr block =
      map_->allocateNewBlockByCoordinates(Point(0.0, 0.0, 0.0));

  TsdfBlock::Ptr block2 = map_->getBlockPtrByCoordinates(Point(0.0, 0.1, 0.0));
  EXPECT_EQ(block, block2);

  // Now check voxel indexing within this block.
  Point test_point(0.0, 0.2, 0.0);
  TsdfVoxel& voxel = block->getTsdfVoxelByCoordinates(test_point);
  voxel.weight = 1.0;
  voxel.distance = 0.5;

  size_t linear_index =
      block->getTsdfLayer().computeLinearIndexFromCoordinates(test_point);
  VoxelIndex voxel_index =
      block->getTsdfLayer().computeVoxelIndexFromCoordinates(test_point);

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(
      voxel_index,
      block->getTsdfLayer().computeVoxelIndexFromLinearIndex(linear_index)));

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
      test_point, block->getCoordinatesOfTsdfVoxelByLinearIndex(linear_index),
      0.1));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
      test_point, block->getCoordinatesOfTsdfVoxelByVoxelIndex(voxel_index),
      0.1));

  test_point = Point(-0.4, -0.2, 0.6);
  block = map_->allocateNewBlockByCoordinates(test_point);

  linear_index =
      block->getTsdfLayer().computeLinearIndexFromCoordinates(test_point);
  voxel_index =
      block->getTsdfLayer().computeVoxelIndexFromCoordinates(test_point);

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(
      voxel_index,
      block->getTsdfLayer().computeVoxelIndexFromLinearIndex(linear_index)));

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
      test_point, block->getCoordinatesOfTsdfVoxelByLinearIndex(linear_index),
      0.1));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
      test_point, block->getCoordinatesOfTsdfVoxelByVoxelIndex(voxel_index),
      0.1));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}

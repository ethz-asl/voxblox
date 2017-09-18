#include <eigen-checks/gtest.h>
#include <eigen-checks/entrypoint.h>
#include <gtest/gtest.h>

#include "voxblox/core/tsdf_map.h"

using namespace voxblox;  // NOLINT

class TsdfMapTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    TsdfMap::Config config;
    config.tsdf_voxel_size = 0.1f;
    config.tsdf_voxels_per_side = 8u;
    map_ = aligned_shared<TsdfMap>(config);
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
  map_->getTsdfLayerPtr()->allocateNewBlockByCoordinates(
      Point(-10.0, 13.5, 20.0));
  EXPECT_EQ(2, map_->getTsdfLayerPtr()->getNumberOfAllocatedBlocks());
}

TEST_F(TsdfMapTest, IndexLookups) {
  Block<TsdfVoxel>::Ptr block =
      map_->getTsdfLayerPtr()->allocateNewBlockByCoordinates(
          Point(0.0, 0.0, 0.0));

  Block<TsdfVoxel>::Ptr block2 =
      map_->getTsdfLayerPtr()->getBlockPtrByCoordinates(Point(0.0, 0.1, 0.0));
  EXPECT_EQ(block, block2);

  // Now check voxel indexing within this block.
  Point test_point(0.0, 0.2, 0.0);
  TsdfVoxel& voxel = block->getVoxelByCoordinates(test_point);
  voxel.weight = 1.0;
  voxel.distance = 0.5;

  size_t linear_index = block->computeLinearIndexFromCoordinates(test_point);
  VoxelIndex voxel_index = block->computeVoxelIndexFromCoordinates(test_point);

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(
      voxel_index, block->computeVoxelIndexFromLinearIndex(linear_index)));

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
      test_point, block->computeCoordinatesFromLinearIndex(linear_index), 0.1));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
      test_point, block->computeCoordinatesFromVoxelIndex(voxel_index), 0.1));

  test_point = Point(-0.4, -0.2, 0.6);
  block = map_->getTsdfLayerPtr()->allocateNewBlockByCoordinates(test_point);

  linear_index = block->computeLinearIndexFromCoordinates(test_point);
  voxel_index = block->computeVoxelIndexFromCoordinates(test_point);

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(
      voxel_index, block->computeVoxelIndexFromLinearIndex(linear_index)));

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
      test_point, block->computeCoordinatesFromLinearIndex(linear_index), 0.1));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
      test_point, block->computeCoordinatesFromVoxelIndex(voxel_index), 0.1));
}

TEST_F(TsdfMapTest, ComputeBlockIndexFromOriginFromBlockIndexTest) {

  constexpr size_t kBlockVolumeDiameter = 100u;
  constexpr FloatingPoint kBlockSize = 0.32;
  constexpr FloatingPoint kBlockSizeInv = 1.0 / 0.32;

  int32_t half_index_range = kBlockVolumeDiameter / 2;

  for (int32_t x = -half_index_range; x <= half_index_range; ++x) {
    for (int32_t y = -half_index_range; y <= half_index_range; ++y) {
      for (int32_t z = -half_index_range; z <= half_index_range; ++z) {
        BlockIndex block_idx = { x, y, z };
        Point block_origin = getOriginPointFromGridIndex(block_idx, kBlockSize);
        BlockIndex block_idx_from_origin = getGridIndexFromOriginPoint(
            block_origin, kBlockSizeInv);

        EXPECT_EQ(block_idx.x(), block_idx_from_origin.x());
        EXPECT_EQ(block_idx.y(), block_idx_from_origin.y());
        EXPECT_EQ(block_idx.z(), block_idx_from_origin.z());

        if ((block_idx.x() != block_idx_from_origin.x())
            || (block_idx.y() != block_idx_from_origin.y())
            || (block_idx.z() != block_idx_from_origin.z())) {
          std::cout << std::endl << std::endl;
          std::cout << "Block idx: " << block_idx.transpose() << std::endl;
          std::cout << "Block origin: " << block_origin.transpose()
              << std::endl;
          std::cout << "Block idx after: " << block_idx_from_origin.transpose()
              << std::endl << std::endl;
        }
      }
    }
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}

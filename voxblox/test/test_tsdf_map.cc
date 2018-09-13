#include <eigen-checks/entrypoint.h>
#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>

#include "voxblox/core/tsdf_map.h"

using namespace voxblox;  // NOLINT

class TsdfMapTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    config_.tsdf_voxel_size = 0.1f;
    config_.tsdf_voxels_per_side = 8u;
    block_size_ = config_.tsdf_voxel_size * config_.tsdf_voxels_per_side;
    map_ = aligned_shared<TsdfMap>(config_);
  }

  TsdfMap::Ptr map_;
  TsdfMap::Config config_;
  FloatingPoint block_size_;
  static constexpr FloatingPoint kEpsilon = 1e-12;
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
  // BLOCK 0 0 0  (cordinate at origin)
  Point point_in_0_0_0(0.0, 0.0, 0.0);
  Block<TsdfVoxel>::Ptr block_0_0_0 =
      map_->getTsdfLayerPtr()->allocateNewBlockByCoordinates(point_in_0_0_0);
  ASSERT_TRUE(block_0_0_0.get() != nullptr);
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(
      map_->getTsdfLayerPtr()->computeBlockIndexFromCoordinates(point_in_0_0_0),
      AnyIndex(0, 0, 0)));
  EXPECT_TRUE(
      EIGEN_MATRIX_NEAR(block_0_0_0->origin(), Point(0.0, 0.0, 0.0), kEpsilon));
  EXPECT_TRUE(
      EIGEN_MATRIX_EQUAL(block_0_0_0->block_index(), AnyIndex(0, 0, 0)));
  EXPECT_NEAR(block_0_0_0->block_size(), block_size_, kEpsilon);

  // BLOCK 0 0 0  (cordinate within block)
  Point point_in_0_0_0_v2(0.0, config_.tsdf_voxel_size, 0.0);
  Block<TsdfVoxel>::Ptr block_0_0_0_v2 =
      map_->getTsdfLayerPtr()->getBlockPtrByCoordinates(point_in_0_0_0_v2);
  ASSERT_TRUE(block_0_0_0_v2.get() != nullptr);
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(
      map_->getTsdfLayerPtr()->computeBlockIndexFromCoordinates(
          point_in_0_0_0_v2),
      AnyIndex(0, 0, 0)));
  EXPECT_EQ(block_0_0_0, block_0_0_0_v2);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(block_0_0_0_v2->origin(), Point(0.0, 0.0, 0.0),
                                kEpsilon));
  EXPECT_TRUE(
      EIGEN_MATRIX_EQUAL(block_0_0_0_v2->block_index(), AnyIndex(0, 0, 0)));
  EXPECT_NEAR(block_0_0_0_v2->block_size(), block_size_, kEpsilon);

  // BLOCK 1 1 1 (coordinate at origin)
  Point point_in_1_1_1(block_size_, block_size_, block_size_);
  Block<TsdfVoxel>::Ptr block_1_1_1 =
      map_->getTsdfLayerPtr()->allocateNewBlockByCoordinates(point_in_1_1_1);
  ASSERT_TRUE(block_1_1_1.get() != nullptr);
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(
      map_->getTsdfLayerPtr()->computeBlockIndexFromCoordinates(point_in_1_1_1),
      AnyIndex(1, 1, 1)));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(block_1_1_1->origin(),
                                Point(block_size_, block_size_, block_size_),
                                kEpsilon));
  EXPECT_TRUE(
      EIGEN_MATRIX_EQUAL(block_1_1_1->block_index(), AnyIndex(1, 1, 1)));
  EXPECT_NEAR(block_1_1_1->block_size(), block_size_, kEpsilon);

  // BLOCK 1 1 1 (coordinate within block)
  Point point_in_1_1_1_v2(block_size_, block_size_ + config_.tsdf_voxel_size,
                          block_size_);
  Block<TsdfVoxel>::Ptr block_1_1_1_v2 =
      map_->getTsdfLayerPtr()->getBlockPtrByCoordinates(point_in_1_1_1_v2);
  ASSERT_TRUE(block_1_1_1_v2.get() != nullptr);
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(
      map_->getTsdfLayerPtr()->computeBlockIndexFromCoordinates(
          point_in_1_1_1_v2),
      AnyIndex(1, 1, 1)));
  EXPECT_EQ(block_1_1_1, block_1_1_1_v2);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(block_1_1_1_v2->origin(),
                                Point(block_size_, block_size_, block_size_),
                                kEpsilon));
  EXPECT_TRUE(
      EIGEN_MATRIX_EQUAL(block_1_1_1_v2->block_index(), AnyIndex(1, 1, 1)));
  EXPECT_NEAR(block_1_1_1_v2->block_size(), block_size_, kEpsilon);

  // BLOCK -1 -1 -1 (coordinate at origin)
  Point point_in_neg_1_1_1(-block_size_, -block_size_, -block_size_);
  Block<TsdfVoxel>::Ptr block_negative_1_1_1 =
      map_->getTsdfLayerPtr()->allocateNewBlockByCoordinates(
          point_in_neg_1_1_1);
  ASSERT_TRUE(block_negative_1_1_1.get() != nullptr);
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(
      map_->getTsdfLayerPtr()->computeBlockIndexFromCoordinates(
          point_in_neg_1_1_1),
      AnyIndex(-1, -1, -1)));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(block_negative_1_1_1->origin(),
                                Point(-block_size_, -block_size_, -block_size_),
                                kEpsilon));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(block_negative_1_1_1->block_index(),
                                 AnyIndex(-1, -1, -1)));
  EXPECT_NEAR(block_negative_1_1_1->block_size(), block_size_, kEpsilon);

  // BLOCK -1 -1 -1 (coordinate within block)
  Point point_in_neg_1_1_1_v2(
      -block_size_, -block_size_ + config_.tsdf_voxel_size, -block_size_);
  Block<TsdfVoxel>::Ptr block_negative_1_1_1_v2 =
      map_->getTsdfLayerPtr()->allocateBlockPtrByCoordinates(
          point_in_neg_1_1_1_v2);
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(
      map_->getTsdfLayerPtr()->computeBlockIndexFromCoordinates(
          point_in_neg_1_1_1_v2),
      AnyIndex(-1, -1, -1)));
  ASSERT_TRUE(block_negative_1_1_1_v2.get() != nullptr);
  EXPECT_EQ(block_negative_1_1_1, block_negative_1_1_1_v2);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(block_negative_1_1_1_v2->origin(),
                                Point(-block_size_, -block_size_, -block_size_),
                                kEpsilon));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(block_negative_1_1_1_v2->block_index(),
                                 AnyIndex(-1, -1, -1)));
  EXPECT_NEAR(block_negative_1_1_1_v2->block_size(), block_size_, kEpsilon);

  // Check voxel indexing.

  // Test in Block 0 0 0
  {
    // Test Voxel 0 1 0
    Point point_in_0_0_0(0.0, 1. * config_.tsdf_voxel_size, 0.0);
    EXPECT_NE(block_0_0_0->getVoxelPtrByCoordinates(point_in_0_0_0), nullptr);

    size_t linear_index =
        block_0_0_0->computeLinearIndexFromCoordinates(point_in_0_0_0);
    EXPECT_EQ(linear_index, 8u);
    VoxelIndex voxel_index =
        block_0_0_0->computeTruncatedVoxelIndexFromCoordinates(point_in_0_0_0);
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL(voxel_index, AnyIndex(0, 1, 0)));

    EXPECT_TRUE(EIGEN_MATRIX_EQUAL(
        voxel_index,
        block_0_0_0->computeVoxelIndexFromLinearIndex(linear_index)));
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(
        point_in_0_0_0,
        block_0_0_0->computeCoordinatesFromLinearIndex(linear_index),
        config_.tsdf_voxel_size));
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(
        point_in_0_0_0,
        block_0_0_0->computeCoordinatesFromVoxelIndex(voxel_index),
        config_.tsdf_voxel_size));
  }
  {
    // Test Voxel 0 0 0
    Point point_in_0_0_0(0.0, 0.0, 0.0);
    EXPECT_NE(block_0_0_0->getVoxelPtrByCoordinates(point_in_0_0_0), nullptr);

    size_t linear_index =
        block_0_0_0->computeLinearIndexFromCoordinates(point_in_0_0_0);
    EXPECT_EQ(linear_index, 0u);
    VoxelIndex voxel_index =
        block_0_0_0->computeTruncatedVoxelIndexFromCoordinates(point_in_0_0_0);
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL(voxel_index, AnyIndex(0, 0, 0)));

    EXPECT_TRUE(EIGEN_MATRIX_EQUAL(
        voxel_index,
        block_0_0_0->computeVoxelIndexFromLinearIndex(linear_index)));
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(
        point_in_0_0_0,
        block_0_0_0->computeCoordinatesFromLinearIndex(linear_index),
        config_.tsdf_voxel_size));
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(
        point_in_0_0_0,
        block_0_0_0->computeCoordinatesFromVoxelIndex(voxel_index),
        config_.tsdf_voxel_size));
  }
  {
    // Test Voxel 7 7 7
    Point point_in_0_0_0(7.0 * config_.tsdf_voxel_size,
                         7.0 * config_.tsdf_voxel_size,
                         7.0 * config_.tsdf_voxel_size);
    EXPECT_NE(block_0_0_0->getVoxelPtrByCoordinates(point_in_0_0_0), nullptr);

    size_t linear_index =
        block_0_0_0->computeLinearIndexFromCoordinates(point_in_0_0_0);
    EXPECT_EQ(linear_index, block_0_0_0->num_voxels() - 1u);
    VoxelIndex voxel_index =
        block_0_0_0->computeTruncatedVoxelIndexFromCoordinates(point_in_0_0_0);
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL(voxel_index,
                                   AnyIndex(config_.tsdf_voxels_per_side - 1,
                                            config_.tsdf_voxels_per_side - 1,
                                            config_.tsdf_voxels_per_side - 1)));

    EXPECT_TRUE(EIGEN_MATRIX_EQUAL(
        voxel_index,
        block_0_0_0->computeVoxelIndexFromLinearIndex(linear_index)));
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(
        point_in_0_0_0,
        block_0_0_0->computeCoordinatesFromLinearIndex(linear_index),
        config_.tsdf_voxel_size));
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(
        point_in_0_0_0,
        block_0_0_0->computeCoordinatesFromVoxelIndex(voxel_index),
        config_.tsdf_voxel_size));
  }
  // Test in Block -1 -1 -1
  {
    // Test Voxel 0 0 0
    Point point_in_neg_1_1_1(-block_negative_1_1_1->block_size(),
                             -block_negative_1_1_1->block_size(),
                             -block_negative_1_1_1->block_size());
    EXPECT_NE(
        block_negative_1_1_1->getVoxelPtrByCoordinates(point_in_neg_1_1_1),
        nullptr);

    size_t linear_index =
        block_negative_1_1_1->computeLinearIndexFromCoordinates(
            point_in_neg_1_1_1);
    EXPECT_EQ(linear_index, 0u);
    VoxelIndex voxel_index =
        block_negative_1_1_1->computeTruncatedVoxelIndexFromCoordinates(
            point_in_neg_1_1_1);
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL(voxel_index, AnyIndex(0, 0, 0)));

    EXPECT_TRUE(EIGEN_MATRIX_EQUAL(
        voxel_index,
        block_negative_1_1_1->computeVoxelIndexFromLinearIndex(linear_index)));
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(
        point_in_neg_1_1_1,
        block_negative_1_1_1->computeCoordinatesFromLinearIndex(linear_index),
        config_.tsdf_voxel_size));
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(
        point_in_neg_1_1_1,
        block_negative_1_1_1->computeCoordinatesFromVoxelIndex(voxel_index),
        config_.tsdf_voxel_size));
  }
  {
    // Test Voxel 7 7 7
    Point point_in_neg_1_1_1(-kEpsilon, -kEpsilon, -kEpsilon);
    EXPECT_NE(
        block_negative_1_1_1->getVoxelPtrByCoordinates(point_in_neg_1_1_1),
        nullptr);

    size_t linear_index =
        block_negative_1_1_1->computeLinearIndexFromCoordinates(
            point_in_neg_1_1_1);
    EXPECT_EQ(linear_index, block_negative_1_1_1->num_voxels() - 1u);
    VoxelIndex voxel_index =
        block_negative_1_1_1->computeTruncatedVoxelIndexFromCoordinates(
            point_in_neg_1_1_1);
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL(voxel_index,
                                   AnyIndex(config_.tsdf_voxels_per_side - 1,
                                            config_.tsdf_voxels_per_side - 1,
                                            config_.tsdf_voxels_per_side - 1)));

    EXPECT_TRUE(EIGEN_MATRIX_EQUAL(
        voxel_index,
        block_negative_1_1_1->computeVoxelIndexFromLinearIndex(linear_index)));
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(
        point_in_neg_1_1_1,
        block_negative_1_1_1->computeCoordinatesFromLinearIndex(linear_index),
        config_.tsdf_voxel_size));
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(
        point_in_neg_1_1_1,
        block_negative_1_1_1->computeCoordinatesFromVoxelIndex(voxel_index),
        config_.tsdf_voxel_size));
  }

  // Test in Block -1 -1 0
  {
    // Test Voxel 3 6 5
    Point point_in_neg_1_1_0(-5. * config_.tsdf_voxel_size,
                             -2. * config_.tsdf_voxel_size,
                             5. * config_.tsdf_voxel_size);
    Block<TsdfVoxel>::Ptr block_neg_1_neg_1_0 =
        map_->getTsdfLayerPtr()->allocateNewBlockByCoordinates(
            point_in_neg_1_1_0);
    ASSERT_TRUE(block_neg_1_neg_1_0.get() != nullptr);
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL(
        map_->getTsdfLayerPtr()->computeBlockIndexFromCoordinates(
            point_in_neg_1_1_0),
        AnyIndex(-1, -1, 0)));
    EXPECT_NE(block_neg_1_neg_1_0->getVoxelPtrByCoordinates(point_in_neg_1_1_0),
              nullptr);
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(block_neg_1_neg_1_0->origin(),
                                  Point(-block_size_, -block_size_, 0.0),
                                  kEpsilon));
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL(block_neg_1_neg_1_0->block_index(),
                                   AnyIndex(-1, -1, 0)));
    EXPECT_NEAR(block_neg_1_neg_1_0->block_size(), block_size_, kEpsilon);

    size_t linear_index =
        block_neg_1_neg_1_0->computeLinearIndexFromCoordinates(
            point_in_neg_1_1_0);
    EXPECT_EQ(linear_index, 371u);
    VoxelIndex voxel_index =
        block_neg_1_neg_1_0->computeTruncatedVoxelIndexFromCoordinates(
            point_in_neg_1_1_0);
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL(voxel_index, AnyIndex(3, 6, 5)));

    EXPECT_TRUE(EIGEN_MATRIX_EQUAL(
        voxel_index,
        block_neg_1_neg_1_0->computeVoxelIndexFromLinearIndex(linear_index)));
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(
        point_in_neg_1_1_0,
        block_neg_1_neg_1_0->computeCoordinatesFromLinearIndex(linear_index),
        config_.tsdf_voxel_size));
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(
        point_in_neg_1_1_0,
        block_neg_1_neg_1_0->computeCoordinatesFromVoxelIndex(voxel_index),
        config_.tsdf_voxel_size));
  }
}

TEST_F(TsdfMapTest, ComputeBlockIndexFromOriginFromBlockIndexTest) {
  constexpr size_t kBlockVolumeDiameter = 100u;
  constexpr FloatingPoint kBlockSize = 0.32;
  constexpr FloatingPoint kBlockSizeInv = 1.0 / kBlockSize;

  int32_t half_index_range = kBlockVolumeDiameter / 2;

  for (int32_t x = -half_index_range; x <= half_index_range; ++x) {
    for (int32_t y = -half_index_range; y <= half_index_range; ++y) {
      for (int32_t z = -half_index_range; z <= half_index_range; ++z) {
        BlockIndex block_idx = {x, y, z};
        Point block_origin = getOriginPointFromGridIndex(block_idx, kBlockSize);
        BlockIndex block_idx_from_origin =
            getGridIndexFromOriginPoint<BlockIndex>(block_origin,
                                                    kBlockSizeInv);

        EXPECT_EQ(block_idx.x(), block_idx_from_origin.x());
        EXPECT_EQ(block_idx.y(), block_idx_from_origin.y());
        EXPECT_EQ(block_idx.z(), block_idx_from_origin.z());

        if ((block_idx.x() != block_idx_from_origin.x()) ||
            (block_idx.y() != block_idx_from_origin.y()) ||
            (block_idx.z() != block_idx_from_origin.z())) {
          std::cout << std::endl << std::endl;
          std::cout << "Block idx: " << block_idx.transpose() << std::endl;
          std::cout << "Block origin: " << block_origin.transpose()
                    << std::endl;
          std::cout << "Block idx after: " << block_idx_from_origin.transpose()
                    << std::endl
                    << std::endl;
          ASSERT_TRUE(false);
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

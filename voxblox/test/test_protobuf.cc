#include <eigen-checks/gtest.h>
#include <eigen-checks/entrypoint.h>
#include <gtest/gtest.h>

#include "./Block.pb.h"
#include "./Layer.pb.h"
#include "voxblox/core/block.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

using namespace voxblox;  // NOLINT

template <typename VoxelType>
class ProtobufTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    layer_.reset(new Layer<VoxelType>(voxel_size_, voxels_per_side_));
    SetUpLayer();
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

  void SetUpLayer() const;

  typename Layer<VoxelType>::Ptr layer_;

  const double voxel_size_ = 0.02;
  const size_t voxels_per_side_ = 16u;

  static constexpr size_t kNumDummyBlocks = 5u;
  static constexpr double kTolerance = 1e-10;
};

template <>
void ProtobufTest<TsdfVoxel>::CompareVoxel(const TsdfVoxel& voxel_A,
                                           const TsdfVoxel& voxel_B) const {
  CHECK_NEAR(voxel_A.distance, voxel_B.distance, kTolerance);
  CHECK_NEAR(voxel_A.weight, voxel_B.weight, kTolerance);
  CHECK_EQ(voxel_A.color.r, voxel_B.color.r);
  CHECK_EQ(voxel_A.color.g, voxel_B.color.g);
  CHECK_EQ(voxel_A.color.b, voxel_B.color.b);
  CHECK_EQ(voxel_A.color.a, voxel_B.color.a);
}

template <>
void ProtobufTest<TsdfVoxel>::SetUpLayer() const {
  CHECK(layer_);
  for (size_t block_nr = 0u; block_nr < kNumDummyBlocks; ++block_nr) {
    BlockIndex block_idx;
    block_idx.x() = static_cast<int>(block_nr);
    block_idx.y() = static_cast<int>(block_nr * kNumDummyBlocks);
    block_idx.z() =
        static_cast<int>(block_nr * kNumDummyBlocks * kNumDummyBlocks);

    Block<TsdfVoxel>::Ptr block = layer_->allocateBlockPtrByIndex(block_idx);
    TsdfVoxel& voxel = block->getVoxelByLinearIndex(block_nr);
    voxel.distance = block_nr * 0.5;
    voxel.weight = block_nr * 0.25;
    voxel.color.r = block_nr;
    voxel.color.g = block_nr + 1;
    voxel.color.b = block_nr + 2;
    voxel.color.a = block_nr + 3;
  }
}

typedef ProtobufTest<TsdfVoxel> ProtobufTsdfTest;

TEST_F(ProtobufTsdfTest, BlockSerialization) {
  for (size_t block_nr = 0u; block_nr < kNumDummyBlocks; ++block_nr) {
    BlockIndex block_idx;
    block_idx.x() = static_cast<int>(block_nr);
    block_idx.y() = static_cast<int>(block_nr * kNumDummyBlocks);
    block_idx.z() =
        static_cast<int>(block_nr * kNumDummyBlocks * kNumDummyBlocks);

    const Block<TsdfVoxel>& block = layer_->getBlockByIndex(block_idx);

    // Convert to BlockProto.
    BlockProto proto_block;
    block.getProto(&proto_block);

    // Create from BlockProto.
    Block<TsdfVoxel> block_from_proto(proto_block);

    CompareBlocks(block, block_from_proto);
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}

#include <iostream>  // NOLINT

#include <gtest/gtest.h>

#include "./Block.pb.h"
#include "./Layer.pb.h"
#include "voxblox/core/block.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/io/layer_io.h"

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

  void CompareLayers(const Layer<VoxelType>& layer_A,
                     const Layer<VoxelType>& layer_B) {
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
        std::cout << "Block at index [" << index_A.transpose()
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
        std::cout << std::endl << "Block at index [" << index_B.transpose()
                  << "] in layer_B does not exists in layer_A" << std::endl;
      }
    }

    EXPECT_EQ(layer_A.getMemorySize(), layer_B.getMemorySize());
  }

  void CompareVoxel(const VoxelType& voxel_A, const VoxelType& voxel_B) const;

  void SetUpLayer() const;

  typename Layer<VoxelType>::Ptr layer_;

  const double voxel_size_ = 0.02;
  const size_t voxels_per_side_ = 16u;

  static constexpr size_t kBlockVolumeDiameter = 12u;
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

  int32_t half_index_range = kBlockVolumeDiameter / 2;

  for (int32_t x = -half_index_range; x <= half_index_range; ++x) {
    for (int32_t y = -half_index_range; y <= half_index_range; ++y) {
      for (int32_t z = -half_index_range; z <= half_index_range; ++z) {
        BlockIndex block_idx = {x, y, z};
        Block<TsdfVoxel>::Ptr block =
            layer_->allocateBlockPtrByIndex(block_idx);
        TsdfVoxel& voxel =
            block->getVoxelByLinearIndex((x * z + y) % voxels_per_side_);
        voxel.distance = x * y * 0.66 + z;
        voxel.weight = y * z * 0.33 + x;
        voxel.color.r = static_cast<uint8_t>(x % 255);
        voxel.color.g = static_cast<uint8_t>(y % 255);
        voxel.color.b = static_cast<uint8_t>(z % 255);
        voxel.color.a = static_cast<uint8_t>(x + y % 255);

        block->has_data() = true;
      }
    }
  }

  double size_in_MB = static_cast<double>(layer_->getMemorySize()) * 1e-6;
  std::cout << std::endl << "Set up a test TSDF layer of size " << size_in_MB
            << " MB";
}

typedef ProtobufTest<TsdfVoxel> ProtobufTsdfTest;

TEST_F(ProtobufTsdfTest, BlockSerialization) {
  BlockIndexList block_index_list;
  layer_->getAllAllocatedBlocks(&block_index_list);
  for (const BlockIndex& index : block_index_list) {
    Block<TsdfVoxel>::Ptr block = layer_->getBlockPtrByIndex(index);
    ASSERT_NE(block.get(), nullptr);

    // Convert to BlockProto.
    BlockProto proto_block;
    block->getProto(&proto_block);

    // Create from BlockProto.
    Block<TsdfVoxel> block_from_proto(proto_block);

    CompareBlocks(*block, block_from_proto);
  }
}

TEST_F(ProtobufTsdfTest, LayerSerialization) {
  // Convert to LayerProto.
  LayerProto proto_layer;
  layer_->getProto(&proto_layer);

  // Create from LayerProto header.
  Layer<TsdfVoxel> layer_from_proto(proto_layer);

  // Remove all blocks for comparison.
  layer_->removeAllBlocks();

  CompareLayers(*layer_, layer_from_proto);
}

TEST_F(ProtobufTsdfTest, LayerSerializationToFile) {
  const std::string file = "layer_test.tsdf.voxblox";

  io::SaveLayer(*layer_, file);

  Layer<TsdfVoxel>::Ptr layer_from_file;
  io::LoadLayer<TsdfVoxel>(file, &layer_from_file);

  CompareLayers(*layer_, *layer_from_file);
}

TEST_F(ProtobufTsdfTest, LayerSubsetSerializationToFile) {
  const std::string file = "subset_layer_test_1.tsdf.voxblox";

  BlockIndexList block_index_list;
  BlockIndex block_index_1 = {-4, 5, 0};
  block_index_list.push_back(block_index_1);
  BlockIndex block_index_4 = {3, -1, -2};
  block_index_list.push_back(block_index_4);

  constexpr bool kIncludeAllBlocks = false;

  io::SaveLayerSubset(*layer_, file, block_index_list, kIncludeAllBlocks);

  Layer<TsdfVoxel>::Ptr layer_from_file;
  io::LoadLayer<TsdfVoxel>(file, &layer_from_file);

  // Remove all other blocks for comparison.
  BlockIndexList all_block_indices;
  layer_->getAllAllocatedBlocks(&all_block_indices);
  for (const BlockIndex& index : all_block_indices) {
    if (index != block_index_1 && index != block_index_4) {
      layer_->removeBlock(index);
    }
  }

  CompareLayers(*layer_, *layer_from_file);
}

// Save the example layer to file, and prepare another layer with 4 existing
// blocks. Load all the blocks from the example layer file into layer and
// compare.
TEST_F(ProtobufTsdfTest, LayerSubsetSerializationFromFile) {
  const std::string file = "subset_layer_test_2.tsdf.voxblox";

  io::SaveLayer(*layer_, file);

  // These are the existing blocks in the layer
  BlockIndexList block_index_list;
  BlockIndex block_index_1 = {-4000, 5000, 0};
  block_index_list.push_back(block_index_1);
  BlockIndex block_index_2 = {-7000, 2000, 1000};
  block_index_list.push_back(block_index_2);
  BlockIndex block_index_3 = {3000, 0, 9000};
  block_index_list.push_back(block_index_3);
  BlockIndex block_index_4 = {3000, -1000, -2000};
  block_index_list.push_back(block_index_4);

  Layer<TsdfVoxel> layer_with_blocks_from_file(voxel_size_, voxels_per_side_);
  layer_with_blocks_from_file.allocateNewBlock(block_index_1);
  layer_with_blocks_from_file.allocateNewBlock(block_index_2);
  layer_with_blocks_from_file.allocateNewBlock(block_index_3);
  layer_with_blocks_from_file.allocateNewBlock(block_index_4);

  // Now load the blocks from the file layer and add it.
  io::LoadBlocksFromFile<TsdfVoxel>(
      file, Layer<TsdfVoxel>::BlockMergingStrategy::kProhibit,
      &layer_with_blocks_from_file);

  // Add those blocks to the layer for comparison.
  layer_->allocateNewBlock(block_index_1);
  layer_->allocateNewBlock(block_index_2);
  layer_->allocateNewBlock(block_index_3);
  layer_->allocateNewBlock(block_index_4);

  CompareLayers(*layer_, layer_with_blocks_from_file);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}

#include <iostream>  // NOLINT

#include <gtest/gtest.h>

#include "./Block.pb.h"
#include "./Layer.pb.h"
#include "voxblox_fast/core/block.h"
#include "voxblox_fast/core/layer.h"
#include "voxblox_fast/core/voxel.h"
#include "voxblox_fast/io/layer_io.h"
#include "voxblox_fast/test/layer_test_utils.h"

using namespace voxblox_fast;  // NOLINT

template <typename VoxelType>
class ProtobufTest : public ::testing::Test,
                     public voxblox_fast::test::LayerTest<VoxelType> {
 protected:
  virtual void SetUp() {
    layer_.reset(new Layer<VoxelType>(voxel_size_, voxels_per_side_));
    SetUpLayer();
  }

  void SetUpLayer() const;

  typename Layer<VoxelType>::Ptr layer_;

  const double voxel_size_ = 0.02;
  const size_t voxels_per_side_ = 16u;

  static constexpr size_t kBlockVolumeDiameter = 10u;
};

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

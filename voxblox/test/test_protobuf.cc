#include <iostream>  // NOLINT

#include <gtest/gtest.h>

#include "./Block.pb.h"
#include "./Layer.pb.h"
#include "voxblox/core/block.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/io/layer_io.h"
#include "voxblox/test/layer_test_utils.h"
#include "voxblox/core/esdf_map.h"
#include "voxblox/integrator/esdf_integrator.h"

using namespace voxblox;  // NOLINT

template <typename VoxelType>
class ProtobufTest : public ::testing::Test,
                     public voxblox::test::LayerTest<VoxelType> {
 protected:
  virtual void SetUp() {
    layer_.reset(new Layer<VoxelType>(voxel_size_, voxels_per_side_));
    SetUpLayer();
  }

  void SetUpLayer() const {
    voxblox::test::SetUpTestLayer(kBlockVolumeDiameter, layer_.get());
  }

  typename Layer<VoxelType>::Ptr layer_;

  const double voxel_size_ = 0.02;
  const size_t voxels_per_side_ = 16u;

  static constexpr size_t kBlockVolumeDiameter = 10u;
};

typedef ProtobufTest<TsdfVoxel> ProtobufTsdfTest;
typedef ProtobufTest<EsdfVoxel> ProtobufEsdfTest;
typedef ProtobufTest<OccupancyVoxel> ProtobufOccupancyTest;

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

TEST_F(ProtobufOccupancyTest, BlockSerialization) {
  BlockIndexList block_index_list;
  layer_->getAllAllocatedBlocks(&block_index_list);
  for (const BlockIndex& index : block_index_list) {
    Block<OccupancyVoxel>::Ptr block = layer_->getBlockPtrByIndex(index);
    ASSERT_NE(block.get(), nullptr);

    // Convert to BlockProto.
    BlockProto proto_block;
    block->getProto(&proto_block);

    // Create from BlockProto.
    Block<OccupancyVoxel> block_from_proto(proto_block);

    CompareBlocks(*block, block_from_proto);
  }
}

TEST_F(ProtobufOccupancyTest, LayerSerialization) {
  // Convert to LayerProto.
  LayerProto proto_layer;
  layer_->getProto(&proto_layer);

  // Create from LayerProto header.
  Layer<OccupancyVoxel> layer_from_proto(proto_layer);

  // Remove all blocks for comparison.
  layer_->removeAllBlocks();

  CompareLayers(*layer_, layer_from_proto);
}

TEST_F(ProtobufEsdfTest, BlockSerialization) {
  BlockIndexList block_index_list;
  layer_->getAllAllocatedBlocks(&block_index_list);
  for (const BlockIndex& index : block_index_list) {
    Block<EsdfVoxel>::Ptr block = layer_->getBlockPtrByIndex(index);
    ASSERT_NE(block.get(), nullptr);

    // Convert to BlockProto.
    BlockProto proto_block;
    block->getProto(&proto_block);

    // Create from BlockProto.
    Block<EsdfVoxel> block_from_proto(proto_block);

    CompareBlocks(*block, block_from_proto);
  }
}

TEST_F(ProtobufEsdfTest, LayerSerialization) {
  // Convert to LayerProto.
  LayerProto proto_layer;
  layer_->getProto(&proto_layer);

  // Create from LayerProto header.
  Layer<EsdfVoxel> layer_from_proto(proto_layer);

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

TEST_F(ProtobufTsdfTest, DISABLE_MultipleLayerSerialization) {
  // First, generate an ESDF out of the test TSDF layer.
  // ESDF maps.
  EsdfMap::Config esdf_config;
  // Same number of voxels per side for ESDF as with TSDF
  esdf_config.esdf_voxels_per_side = layer_->voxels_per_side();
  // Same voxel size for ESDF as with TSDF
  esdf_config.esdf_voxel_size = layer_->voxel_size();

  // Default settings are fine, actual content of the ESDF doesn't matter
  // much.
  EsdfIntegrator::Config esdf_integrator_config;

  EsdfMap esdf_map(esdf_config);
  EsdfIntegrator esdf_integrator(esdf_integrator_config, layer_.get(),
                                 esdf_map.getEsdfLayerPtr());

  esdf_integrator.updateFromTsdfLayerBatchFullEuclidean();
  voxblox::test::LayerTest<EsdfVoxel> esdf_test;

  const std::string file = "multi_layer_test.voxblox";
  bool clear_file = true;
  io::SaveLayer(*layer_, file, clear_file);
  clear_file = false;
  io::SaveLayer(*esdf_map.getEsdfLayerPtr(), file, clear_file);

  bool multiple_layer_support = true;
  Layer<TsdfVoxel>::Ptr tsdf_layer_from_file;
  io::LoadLayer<TsdfVoxel>(file, multiple_layer_support, &tsdf_layer_from_file);

  Layer<EsdfVoxel>::Ptr esdf_layer_from_file;
  io::LoadLayer<EsdfVoxel>(file, multiple_layer_support, &esdf_layer_from_file);

  CompareLayers(*layer_, *tsdf_layer_from_file);
  esdf_test.CompareLayers(*esdf_map.getEsdfLayerPtr(), *esdf_layer_from_file);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}

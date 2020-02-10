#include <gtest/gtest.h>

#include "voxblox/Block.pb.h"
#include "voxblox/Layer.pb.h"
#include "voxblox/core/block.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/test/layer_test_utils.h"
#include "voxblox/utils/layer_utils.h"

using namespace voxblox;  // NOLINT

template <typename VoxelType>
class LayerTest : public ::testing::Test,
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

typedef LayerTest<TsdfVoxel> TsdfLayerTest;
typedef LayerTest<EsdfVoxel> EsdfLayerTest;
typedef LayerTest<OccupancyVoxel> OccupancyLayerTest;

TEST_F(TsdfLayerTest, DeepCopyConstructor) {
  Layer<TsdfVoxel> new_layer(*layer_);
  EXPECT_TRUE(voxblox::utils::isSameLayer(new_layer, *layer_));
}

TEST_F(EsdfLayerTest, DeepCopyConstructor) {
  Layer<EsdfVoxel> new_layer(*layer_);
  EXPECT_TRUE(voxblox::utils::isSameLayer(new_layer, *layer_));
}

TEST_F(OccupancyLayerTest, DeepCopyConstructor) {
  Layer<OccupancyVoxel> new_layer(*layer_);
  EXPECT_TRUE(voxblox::utils::isSameLayer(new_layer, *layer_));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}

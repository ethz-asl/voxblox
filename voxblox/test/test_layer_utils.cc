#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>

#include "voxblox/core/block.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/test/layer_test_utils.h"
#include "voxblox/utils/layer_utils.h"

using namespace voxblox;  // NOLINT

template <typename VoxelType>
class LayerUtilsTest : public ::testing::Test,
                       public voxblox::test::LayerTest<VoxelType> {
 protected:
  virtual void SetUp() {
    layer_.reset(new Layer<VoxelType>(voxel_size_, voxels_per_side_));
    SetUpLayer();
  }

  void SetUpLayer() const {
    test::SetUpTestLayer(kBlockVolumeDiameter, kBlockVolumeOffset,
                         layer_.get());
  }

  typename Layer<VoxelType>::Ptr layer_;

  const double voxel_size_ = 0.02;
  const size_t voxels_per_side_ = 16u;

  static constexpr IndexElement kBlockVolumeDiameter = 10u;
  static constexpr IndexElement kBlockVolumeOffset = 5u;

  static constexpr FloatingPoint kPrecision = 1e-6;
};

typedef LayerUtilsTest<TsdfVoxel> TsdfLayerUtilsTest;
typedef LayerUtilsTest<EsdfVoxel> EsdfLayerUtilsTest;
typedef LayerUtilsTest<OccupancyVoxel> OccupancyLayerUtilsTest;

TEST_F(TsdfLayerUtilsTest, centerBlocksOfLayerTsdfTest) {
  Point new_layer_origin;
  utils::centerBlocksOfLayer(layer_.get(), &new_layer_origin);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
      new_layer_origin,
      voxel_size_ * voxels_per_side_ * kBlockVolumeOffset * Point::Ones(),
      kPrecision));
}

TEST_F(EsdfLayerUtilsTest, centerBlocksOfLayerEsdfTest) {
  Point new_layer_origin;
  utils::centerBlocksOfLayer(layer_.get(), &new_layer_origin);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
      new_layer_origin,
      voxel_size_ * voxels_per_side_ * kBlockVolumeOffset * Point::Ones(),
      kPrecision));
}

TEST_F(OccupancyLayerUtilsTest, centerBlocksOfLayerOccupancyTest) {
  Point new_layer_origin;
  utils::centerBlocksOfLayer(layer_.get(), &new_layer_origin);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
      new_layer_origin,
      voxel_size_ * voxels_per_side_ * kBlockVolumeOffset * Point::Ones(),
      kPrecision));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}

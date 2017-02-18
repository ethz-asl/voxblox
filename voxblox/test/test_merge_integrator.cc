#include <eigen-checks/entrypoint.h>
#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>

#include "voxblox/core/tsdf_map.h"

using namespace voxblox;  // NOLINT

class TsdfMergeIntegratorTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    TsdfMap::Config config;
    config.tsdf_voxel_size = 1.0f;
    config.tsdf_voxels_per_side = 8u;
    map_ = std::make_shared<TsdfMap>(config);
  }

  TsdfMap::Ptr map_;
};

TEST_F(TsdfMergeIntegratorTest, NonOverlappingLayers) {
  // DEBUG

  //
  std::cout << "Allocating some shit." << std::endl;

  Layer<TsdfVoxel>* layer_ptr = map_->getTsdfLayerPtr();

  // Creating a test layer
  for (float x = 0.0; x < 8.0; x++) {
    for (float y = 0.0; y < 8.0; y++) {
      for (float z = 0.0; z < 8.0; z++) {
        Point point(x, y, z);
        Block<TsdfVoxel>::Ptr block_ptr = layer_ptr->allocateBlockPtrByCoordinates(point);
        TsdfVoxel& voxel_ref = block_ptr->getVoxelByCoordinates(point);
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

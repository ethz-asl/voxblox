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
  EXPECT_EQ(0, map_->getNumberOfAllocatedBlocks());
  map_->allocateNewBlockByCoordinates(Point(0.0, 0.15, 0.0));
  EXPECT_EQ(1, map_->getNumberOfAllocatedBlocks());
  map_->allocateNewBlockByCoordinates(Point(0.0, 0.13, 0.0));
  EXPECT_EQ(1, map_->getNumberOfAllocatedBlocks());
  map_->allocateNewBlockByCoordinates(Point(-10.0, 13.5, 20.0));
  EXPECT_EQ(2, map_->getNumberOfAllocatedBlocks());
}
/*
TEST_F(ESDFUpdateTest, DeleteObstacle) {
  // map_.setBoxFree(Eigen::Vector2d(1, 0), Eigen::Vector2d(2, 1));
  mav_planning_utils::timing::Timer timer_update("map/update");
  map_.updateMapIncremental();
  timer_update.Stop();
  // map_.printMatlabMatrix();
}

TEST_F(ESDFUpdateTest, AddInside) {
  // map_.setBoxOccupied(Eigen::Vector2d(2, 1), Eigen::Vector2d(3, 2));
  // map_.setBoxOccupied(Eigen::Vector2d(2.25, 1.25), Eigen::Vector2d(2.75,
  // 1.75));
  map_.updateMapIncremental();
  // map_.printMatlabMatrix();
}

TEST_F(ESDFUpdateTest, AddRandom) {
  // map_.setBoxOccupied(Eigen::Vector2d(1.25, 2.25), Eigen::Vector2d(1.75,
  // 2.75));
  int key = map_.getNearestFreeSpace(Eigen::Vector2d(3.5, 3.5), 0.5);
  Eigen::VectorXd pos = map_.getMetricPosFromKey(key);
  std::cout << pos << std::endl;
  map_.updateMapIncremental();
  // map_.printMatlabMatrix();
}
*/
int main(int argc, char **argv) {

  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  //google::ParseCommandLineFlags(&argc, &argv, false);
  //google::InstallFailureSignalHandler();

  int result = RUN_ALL_TESTS();
  //timing::Timing::Print(std::cout);

  return result;
}

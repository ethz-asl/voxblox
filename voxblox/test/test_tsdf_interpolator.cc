#include <eigen-checks/entrypoint.h>
#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>

//#include "voxblox/core/tsdf_map.h"
#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/interpolator/interpolator.h"

using namespace voxblox;  // NOLINT

class TsdfMergeIntegratorTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    tsdf_voxels_per_side_ = 10u;
    tsdf_voxel_size_ = 1.0f;
    compare_tol_ = 0.0000001f;
  }

  size_t tsdf_voxels_per_side_;
  float tsdf_voxel_size_;
  float compare_tol_;

  void setBlockSameValues(const float distance,
                          Block<TsdfVoxel>::Ptr block_ptr) {
    // Looping and setting all voxels
    for (size_t x_idx = 0; x_idx < tsdf_voxels_per_side_; x_idx++) {
      for (size_t y_idx = 0; y_idx < tsdf_voxels_per_side_; y_idx++) {
        for (size_t z_idx = 0; z_idx < tsdf_voxels_per_side_; z_idx++) {
          // Creating the point
          VoxelIndex index(x_idx, y_idx, z_idx);
          TsdfVoxel& voxel_ref = block_ptr->getVoxelByVoxelIndex(index);
          // Setting voxel
          voxel_ref.distance = distance;
          voxel_ref.weight = 1.0;
        }
      }
    }
    // Setting has data flag
    block_ptr->has_data() = true;
  }
};

TEST_F(TsdfMergeIntegratorTest, WithinBlock) {
  // New layer
  Layer<TsdfVoxel> tsdf_layer(tsdf_voxel_size_, tsdf_voxels_per_side_);

  // Allocating the block;
  Point origin(0.0, 0.0, 0.0);
  Block<TsdfVoxel>::Ptr block_ptr =
      tsdf_layer.allocateBlockPtrByCoordinates(origin);

  // This is a block where the distance of each voxel indicates height
  for (float x = 0.5; x < tsdf_voxels_per_side_; x++) {
    for (float y = 0.5; y < tsdf_voxels_per_side_; y++) {
      for (float z = 0.5; z < tsdf_voxels_per_side_; z++) {
        // Creating the point
        Point point(x, y, z);
        // Getting a voxel
        TsdfVoxel& voxel_ref = block_ptr->getVoxelByCoordinates(point);
        // Setting voxel
        voxel_ref.distance = z;
        voxel_ref.weight = 1.0;
        block_ptr->has_data() = true;
      }
    }
  }

  // Creating the interpolator
  Interpolator<TsdfVoxel> interpolator(&tsdf_layer);

  // A test output voxel
  TsdfVoxel voxel;

  // Interpolating below the block
  Point point_out_1(1.0, 1.0, 0.1);
  Point point_out_2(1.0, 1.0, 0.4);
  // Expecting this to not work
  EXPECT_FALSE(interpolator.getVoxel(point_out_1, &voxel, true));
  EXPECT_FALSE(interpolator.getVoxel(point_out_2, &voxel, true));

  // Interpolating within the block
  bool failed_to_interpolate = false;
  bool wrong_answer = false;
  for (float x = 1.0; x < tsdf_voxels_per_side_ - 0.5; x = x + 0.1) {
    for (float y = 1.0; y < tsdf_voxels_per_side_ - 0.5; y = y + 0.1) {
      for (float z = 1.0; z < tsdf_voxels_per_side_ - 0.5; z = z + 0.1) {
        // The point to interpolate at
        Point point(x, y, z);
        // Interpolating at this point
        if (interpolator.getVoxel(point, &voxel, true)) {
          if (fabs(voxel.distance - z) > compare_tol_) {
            wrong_answer = true;
          }
        } else {
          failed_to_interpolate = true;
        }
      }
    }
  }
  // Testing the results
  EXPECT_FALSE(wrong_answer);
  EXPECT_FALSE(failed_to_interpolate);
}

TEST_F(TsdfMergeIntegratorTest, BetweenBlocks) {
  // New layer
  Layer<TsdfVoxel> tsdf_layer(tsdf_voxel_size_, tsdf_voxels_per_side_);

  // Allocating some blocks (in a sort of 3D plus pattern)
  Block<TsdfVoxel>::Ptr block_ptr_0 =
      tsdf_layer.allocateBlockPtrByCoordinates(Point(0.0, 0.0, 0.0));
  Block<TsdfVoxel>::Ptr block_ptr_x_p1 =
      tsdf_layer.allocateBlockPtrByCoordinates(
          Point(tsdf_voxels_per_side_, 0.0, 0.0));
  Block<TsdfVoxel>::Ptr block_ptr_x_n1 =
      tsdf_layer.allocateBlockPtrByCoordinates(
          Point(-tsdf_voxel_size_, 0.0, 0.0));
  Block<TsdfVoxel>::Ptr block_ptr_y_p1 =
      tsdf_layer.allocateBlockPtrByCoordinates(
          Point(0.0, tsdf_voxels_per_side_, 0.0));
  Block<TsdfVoxel>::Ptr block_ptr_y_n1 =
      tsdf_layer.allocateBlockPtrByCoordinates(
          Point(0.0, -tsdf_voxel_size_, 0.0));
  Block<TsdfVoxel>::Ptr block_ptr_z_p1 =
      tsdf_layer.allocateBlockPtrByCoordinates(
          Point(0.0, 0.0, tsdf_voxels_per_side_));
  Block<TsdfVoxel>::Ptr block_ptr_z_n1 =
      tsdf_layer.allocateBlockPtrByCoordinates(
          Point(0.0, 0.0, -tsdf_voxel_size_));

  // Assigning the blocks some values
  setBlockSameValues(0.0, block_ptr_0);
  setBlockSameValues(1.0, block_ptr_x_p1);
  setBlockSameValues(-1.0, block_ptr_x_n1);
  setBlockSameValues(1.0, block_ptr_y_p1);
  setBlockSameValues(-1.0, block_ptr_y_n1);
  setBlockSameValues(1.0, block_ptr_z_p1);
  setBlockSameValues(-1.0, block_ptr_z_n1);

  // Interpolating some bullshit
  Interpolator<TsdfVoxel> interpolator(&tsdf_layer);

  // Points to interpolate (below origin block)
  AlignedVector<Point> points_below;
  points_below.push_back(Point(1.0, 1.0, 0.5));
  points_below.push_back(Point(1.0, 1.0, 0.25));
  points_below.push_back(Point(1.0, 1.0, 0.0));
  points_below.push_back(Point(1.0, 1.0, -0.25));
  points_below.push_back(Point(1.0, 1.0, -0.5));

  // Expected answers (hand calculated)
  float expected_answers_below[5] = {0, -0.25, -0.5, -0.75, -1.0};

  // Looping and testing
  TsdfVoxel voxel;
  for (size_t point_index = 0; point_index < points_below.size();
       point_index++) {
    // Getting the interpolated point
    Point point = points_below[point_index];
    interpolator.getVoxel(point, &voxel, true);
    // Testing
    EXPECT_NEAR(voxel.distance, expected_answers_below[point_index],
                compare_tol_);
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}

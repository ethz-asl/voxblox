#include <eigen-checks/entrypoint.h>
#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>

#include "voxblox/core/tsdf_map.h"
#include "voxblox/interpolator/interpolator.h"

using namespace voxblox;  // NOLINT

class TsdfMergeIntegratorTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    tsdf_voxels_per_side_ = 10u;
    tsdf_voxel_size_ = 1.0f;
    config_.tsdf_voxel_size = tsdf_voxel_size_;
    config_.tsdf_voxels_per_side = tsdf_voxels_per_side_;
  }

  TsdfMap::Config config_;
  size_t tsdf_voxels_per_side_;
  float tsdf_voxel_size_;

  void setBlockSameValues(const float distance, Block<TsdfVoxel>::Ptr block_ptr) {
    // Looping and setting all voxels
    for (int x_idx = 0; x_idx < tsdf_voxels_per_side_; x_idx++) {
      for (int y_idx = 0; y_idx < tsdf_voxels_per_side_; y_idx++) {
        for (int z_idx = 0; z_idx < tsdf_voxels_per_side_; z_idx++) {
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
  Layer<TsdfVoxel> tsdf_layer(config_.tsdf_voxel_size,
                              config_.tsdf_voxels_per_side);

  // This is a cube where the distance of each voxel indicates height
  for (float x = 0.5; x < tsdf_voxels_per_side_; x++) {
    for (float y = 0.5; y < tsdf_voxels_per_side_; y++) {
      for (float z = 0.5; z < tsdf_voxels_per_side_; z++) {
        // Creating the point
        Point point(x, y, z);
        // Getting a voxel
        Block<TsdfVoxel>::Ptr block_ptr = tsdf_layer.allocateBlockPtrByCoordinates(point);
        TsdfVoxel& voxel_ref = block_ptr->getVoxelByCoordinates(point);
        // Setting voxel
        voxel_ref.distance = z;
        voxel_ref.weight = 1.0;
        block_ptr->has_data() = true;
      }
    }
  }

  // Checking
  size_t num_allocated_blocks = tsdf_layer.getNumberOfAllocatedBlocks();
  std::cout << "num_allocated_blocks: " << num_allocated_blocks << std::endl;

/*
  // Checking all the voxels
  BlockIndexList blocks_index_list;
  tsdf_layer->getAllAllocatedBlocks(&blocks_index_list);
  for (BlockIndex block_index : blocks_index_list) {
    // Getting the allocated block
    Block<TsdfVoxel>& block = tsdf_layer->getBlockByIndex(block_index);
    // Checking the voxels
    for (int x = 0; x < tsdf_voxels_per_side_; x++) {
      for (int y = 0; y < tsdf_voxels_per_side_; y++) {
        for (int z = 0; z < tsdf_voxels_per_side_; z++) {
          // Getting a voxel
          VoxelIndex index(x, y, z);
          Point coordinates = block.computeCoordinatesFromVoxelIndex(index);
          TsdfVoxel voxel = block.getVoxelByVoxelIndex(index);
          // DEBUG
          std::cout << "coordinates: " << coordinates << std::endl;
          std::cout << "voxel.distance: " << voxel.distance << std::endl;
        }
      }
    }
  }

*/
  // Creating the interpolator
  Interpolator<TsdfVoxel> interpolator(&tsdf_layer);

  // Interpolating within a block
  bool failed_to_interpolate = false;
  bool wrong_answer = false;
  for (float x = 1.0; x < tsdf_voxels_per_side_-0.5; x++) {
    for (float y = 1.0; y < tsdf_voxels_per_side_-0.5; y++) {
      for (float z = 1.0; z < tsdf_voxels_per_side_-0.5; z++) {
        // The point to interpolate at
        Point point(x, y, z);
        //std::cout << "point: " << point << std::endl;
        // Interpolating at this point
        TsdfVoxel voxel;
        if (interpolator.getVoxel(point, &voxel, true)) {
          // DEBUG
          //std::cout << "voxel.distance: " << voxel.distance << std::endl;
          if (voxel.distance != z) {
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
  // DEBUG

  // New layer
  Layer<TsdfVoxel> tsdf_layer(config_.tsdf_voxel_size,
                              config_.tsdf_voxels_per_side);

  // Allocating some blocks
  Block<TsdfVoxel>::Ptr block_ptr_0 = tsdf_layer.allocateBlockPtrByCoordinates(Point(0.0, 0.0, 0.0));
  Block<TsdfVoxel>::Ptr block_ptr_x_p1 = tsdf_layer.allocateBlockPtrByCoordinates(Point(tsdf_voxels_per_side_, 0.0, 0.0));
  Block<TsdfVoxel>::Ptr block_ptr_x_n1 = tsdf_layer.allocateBlockPtrByCoordinates(Point(-tsdf_voxel_size_, 0.0, 0.0));
  Block<TsdfVoxel>::Ptr block_ptr_y_p1 = tsdf_layer.allocateBlockPtrByCoordinates(Point(0.0, tsdf_voxels_per_side_, 0.0));
  Block<TsdfVoxel>::Ptr block_ptr_y_n1 = tsdf_layer.allocateBlockPtrByCoordinates(Point(0.0, -tsdf_voxel_size_, 0.0));
  Block<TsdfVoxel>::Ptr block_ptr_z_p1 = tsdf_layer.allocateBlockPtrByCoordinates(Point(0.0, 0.0, tsdf_voxels_per_side_));
  Block<TsdfVoxel>::Ptr block_ptr_z_n1 = tsdf_layer.allocateBlockPtrByCoordinates(Point(0.0, 0.0, -tsdf_voxel_size_));

  // Checking allocated
  size_t num_allocated_blocks = tsdf_layer.getNumberOfAllocatedBlocks();
  std::cout << "num_allocated_blocks: " << num_allocated_blocks << std::endl;

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

  // Point to interpolate
  Point point(1.0, 1.0, 0.4);

  TsdfVoxel voxel;
  //if (interpolator.getVoxel(point, &voxel, true)) {
  const TsdfVoxel* voxels[8];
  bool result = interpolator.getInterpVoxelTest(point, &voxel, voxels);
  if (result) {
    //std::cout << "result voxel.distance: " << voxel.distance << std::endl;
  } else {
    std::cout << "Failed to interpolate" << std::endl;
  }


  //DEBUG
  std::cout << "Point to interpolate at:" << std::endl << point << std::endl;
  for (size_t voxel_index = 0; voxel_index < 8; voxel_index++) {
    const TsdfVoxel* voxel = voxels[voxel_index];
    std::cout << "voxel_index: " << voxel_index << std::endl;
    std::cout << "voxel.distance: " << voxel->distance << std::endl;
  }
  std::cout << "result voxel.distance: " << voxel.distance << std::endl;



}







int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}

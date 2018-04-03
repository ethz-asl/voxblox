#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>

#include "voxblox/core/block.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/simulation/simulation_world.h"
#include "voxblox/test/layer_test_utils.h"
#include "voxblox/utils/layer_utils.h"

using namespace voxblox;  // NOLINT

template <typename VoxelType>
class LayerUtilsTest : public ::testing::Test,
                       public voxblox::test::LayerTest<VoxelType> {
 protected:
  void initializeSimulatedWorld() {
    world_.reset(new Layer<VoxelType>(voxel_size_, voxels_per_side_));

    SimulationWorld simulation;
    Point lower_bound, upper_bound;
    lower_bound << -0.51, -0.51, -0.51;
    upper_bound << 0.9, 0.9, 0.9;
    expected_new_coordinate_origin_ << 0.16, 0.16, 0.16;

    simulation.setBounds(lower_bound, upper_bound);

    // Define sphere 1.
    Point c1_A;
    c1_A << 0.104, 0.104, 0.106;
    constexpr FloatingPoint sphere_1_radius_m = 0.05;

    // Define sphere 2.
    Point c2_A;
    c2_A << -0.104, 0.103, 0.203;
    constexpr FloatingPoint sphere_2_radius_m = 0.03;

    // Define sphere 3.
    Point c3_A;
    c3_A << 0.401, 0.151, 0.203;
    constexpr FloatingPoint sphere_3_radius_m = 0.075;

    // Prepare world A
    simulation.addObject(std::unique_ptr<Object>(
        new Sphere(c1_A, sphere_1_radius_m, Color(255u, 0u, 0u))));
    simulation.addObject(std::unique_ptr<Object>(
        new Sphere(c2_A, sphere_2_radius_m, Color(255u, 0u, 0u))));
    simulation.addObject(std::unique_ptr<Object>(
        new Sphere(c3_A, sphere_3_radius_m, Color(255u, 0u, 0u))));
    simulation.generateSdfFromWorld(max_distance_world, world_.get());
  }

  virtual void SetUp() { initializeSimulatedWorld(); }

  typename Layer<VoxelType>::Ptr world_;

  Point expected_new_coordinate_origin_;

  const double voxel_size_ = 0.005;
  const size_t voxels_per_side_ = 16u;
  const double max_distance_world = 2.0;

  static constexpr FloatingPoint kPrecision = 1e-6;
};

typedef LayerUtilsTest<TsdfVoxel> TsdfLayerUtilsTest;
typedef LayerUtilsTest<EsdfVoxel> EsdfLayerUtilsTest;
typedef LayerUtilsTest<OccupancyVoxel> OccupancyLayerUtilsTest;

TEST_F(TsdfLayerUtilsTest, centerBlocksOfLayerTsdfTest) {
  Point new_layer_origin;
  utils::centerBlocksOfLayer(world_.get(), &new_layer_origin);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(new_layer_origin,
                                expected_new_coordinate_origin_, kPrecision));
}

TEST_F(EsdfLayerUtilsTest, centerBlocksOfLayerEsdfTest) {
  Point new_layer_origin;
  utils::centerBlocksOfLayer(world_.get(), &new_layer_origin);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(new_layer_origin,
                                expected_new_coordinate_origin_, kPrecision));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}

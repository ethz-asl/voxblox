#include <random>

#include <eigen-checks/gtest.h>
#include <eigen-checks/entrypoint.h>
#include <gtest/gtest.h>

#include "voxblox/integrator/tsdf_integrator.h"
#include "voxblox/integrator/tsdf_integrator_fast.h"
#include "voxblox/test/layer_test_utils.h"

using namespace voxblox;  // NOLINT

class FastTsdfIntegratorTest : public ::testing::Test {
 protected:
  FastTsdfIntegratorTest(): 
    baseline_layer_(kVoxelSize, kVoxelsPerSide),
    fast_layer_(kVoxelSize, kVoxelsPerSide),
    baseline_integrator_(config_, &baseline_layer_),
    fast_integrator_(fast_config_, &fast_layer_) {
        std::cout << "ctor" << std::endl;
  }
  virtual void SetUp() {
    std::default_random_engine gen(kSeed);
    std::uniform_int_distribution<uint8_t> int_dist(0, 255);
    std::uniform_real_distribution<double> xy_dist(-1.0, 1.0);
    std::normal_distribution<double> z_dist(3.0, 0.5);

    std::cout << "setup" << std::endl;
    for (size_t i = 0u; i < kNumCamerasToGenerate; ++i) {
        Transformation T_G_C;
        T_G_C_.push_back(T_G_C);

        Pointcloud pointcloud;
        Colors colors;
        for (size_t j = 0u; j < kNumPointsToGenerate; ++j) {
          Point point(xy_dist(gen), xy_dist(gen), z_dist(gen));
          pointcloud.push_back(point);

          Color color(int_dist(gen), int_dist(gen), int_dist(gen), int_dist(gen));
          colors.push_back(color);
        }
        CHECK_EQ(pointcloud.size(), colors.size());

        points_C_.push_back(pointcloud);
        colors_.push_back(colors);
    }
    std::cout << "setup done" << std::endl;
  }

  static constexpr size_t kNumCamerasToGenerate = 10u;
  static constexpr size_t kNumPointsToGenerate = 200u;
  static constexpr size_t kSeed = 242u;

  static constexpr double kVoxelSize = 0.01;
  static constexpr size_t kVoxelsPerSide = 16u;

  voxblox::TsdfIntegrator::Config config_;
  voxblox::fast::TsdfIntegrator::Config fast_config_;

  voxblox::TsdfIntegrator baseline_integrator_;
  voxblox::fast::TsdfIntegrator fast_integrator_;

  Layer<TsdfVoxel> baseline_layer_;
  Layer<TsdfVoxel> fast_layer_;

  voxblox::test::LayerTest<TsdfVoxel> layer_test_;

  std::vector<Transformation, Eigen::aligned_allocator<Transformation>> T_G_C_;
  std::vector<Pointcloud, Eigen::aligned_allocator<Pointcloud>> points_C_;
  std::vector<Colors> colors_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

TEST_F(FastTsdfIntegratorTest, CompareIntegratorWithBaseline) {
    std::cout << "test" << std::endl;
    for (size_t i = 0u; i < kNumCamerasToGenerate; ++i) {
      std::cout << i << std::endl;
      CHECK_LT(i, T_G_C_.size());
      CHECK_LT(i, points_C_.size());
      CHECK_LT(i, colors_.size());
      baseline_integrator_.integratePointCloud(T_G_C_[i], points_C_[i], colors_[i]);
      std::cout << i << std::endl;
      fast_integrator_.integratePointCloud(T_G_C_[i], points_C_[i], colors_[i]);
      std::cout << i << std::endl;
    }
    std::cout << "v" << std::endl;
    layer_test_.CompareLayers(baseline_layer_, fast_layer_);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}
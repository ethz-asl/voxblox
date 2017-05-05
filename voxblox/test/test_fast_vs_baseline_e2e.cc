#include <random>

#include <eigen-checks/entrypoint.h>
#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>

#include "voxblox/core/tsdf_map.h"
#include "voxblox/integrator/tsdf_integrator.h"
#include "voxblox/integrator/tsdf_integrator_fast.h"
#include "voxblox/simulation/sphere_simulator.h"
#include "voxblox/test/layer_test_utils.h"

using namespace voxblox;  // NOLINT

class FastTsdfIntegratorTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    baseline_layer_.reset(new Layer<TsdfVoxel>(kVoxelSize, kVoxelsPerSide));
    fast_layer_.reset(new Layer<TsdfVoxel>(kVoxelSize, kVoxelsPerSide));
    baseline_integrator_.reset(
        new TsdfIntegrator(config_, baseline_layer_.get()));
    fast_integrator_.reset(
        new fast::TsdfIntegrator(fast_config_, fast_layer_.get()));
    sphere_points_C.clear();

    constexpr double kMean = 0;
    constexpr double kSigma = 0.05;
    constexpr int kNumPoints = 100;
    constexpr double kRadius = 1.0;

    sphere_sim::createSphere(kMean, kSigma, kRadius, kNumPoints,
                             &sphere_points_C);

    colors.clear();
    colors.resize(sphere_points_C.size(), Color(128, 255, 0));

    T_G_C = Transformation();
  }

  Colors colors;
  Pointcloud sphere_points_C;
  Transformation T_G_C;

  static constexpr size_t kNumCamerasToGenerate = 10u;
  static constexpr size_t kNumPointsToGenerate = 200u;
  static constexpr size_t kSeed = 242u;

  static constexpr double kVoxelSize = 0.01;
  static constexpr size_t kVoxelsPerSide = 16u;

  TsdfIntegrator::Config config_;
  fast::TsdfIntegrator::Config fast_config_;

  std::unique_ptr<TsdfIntegrator> baseline_integrator_;
  std::unique_ptr<fast::TsdfIntegrator> fast_integrator_;

  std::unique_ptr<Layer<TsdfVoxel>> baseline_layer_;
  std::unique_ptr<Layer<TsdfVoxel>> fast_layer_;

  test::LayerTest<TsdfVoxel> layer_test_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

TEST_F(FastTsdfIntegratorTest, CompareIntegratorWithBaseline) {
  baseline_integrator_->integratePointCloud(T_G_C, sphere_points_C, colors);
  fast_integrator_->integratePointCloud(T_G_C, sphere_points_C, colors);
  layer_test_.CompareLayers(*baseline_layer_, *fast_layer_);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}

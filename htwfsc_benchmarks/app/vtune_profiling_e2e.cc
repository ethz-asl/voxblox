#include <memory>

#include <gflags/gflags.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox_fast/core/tsdf_map.h>
#include <voxblox_fast/integrator/tsdf_integrator.h>

#include "htwfsc_benchmarks/simulation/sphere_simulator.h"

DEFINE_bool(use_baseline, false, "If true run the baseline version;"
    "otherwise the fast version.");

class E2EBenchmarkFixture {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void SetUp() {
    config_.max_ray_length_m = 50.0;
    fast_config_.max_ray_length_m = 50.0;

    baseline_layer_.reset(new voxblox::Layer<voxblox::TsdfVoxel>(kVoxelSize, kVoxelsPerSide));
    fast_layer_.reset(new voxblox_fast::Layer<voxblox_fast::TsdfVoxel>(kVoxelSize, kVoxelsPerSide));
    baseline_integrator_.reset(
        new voxblox::TsdfIntegrator(config_, baseline_layer_.get()));
    fast_integrator_.reset(
        new voxblox_fast::TsdfIntegrator(fast_config_, fast_layer_.get()));
    T_G_C = voxblox::Transformation();
  }

  void CreateSphere(const double radius, const size_t num_points) {
    sphere_points_C.clear();
    htwfsc_benchmarks::sphere_sim::createSphere(kMean, kSigma, radius,
                                                num_points, &sphere_points_C);
    colors_.clear();
    colors_.resize(sphere_points_C.size(), voxblox::Color(128, 255, 0));
    fast_colors_.clear();
    fast_colors_.resize(sphere_points_C.size(), voxblox_fast::Color(128, 255, 0));
  }

  void TearDown() {
    baseline_layer_.reset();
    fast_layer_.reset();
    baseline_integrator_.reset();
    fast_integrator_.reset();

    sphere_points_C.clear();
    colors_.clear();
  }

  voxblox::Colors colors_;
  voxblox_fast::Colors fast_colors_;
  voxblox::Pointcloud sphere_points_C;
  voxblox::Transformation T_G_C;

  static constexpr double kVoxelSize = 0.01;
  static constexpr size_t kVoxelsPerSide = 16u;

  static constexpr double kMean = 0;
  static constexpr double kSigma = 0.05;
  static constexpr size_t kNumPoints = 1e5;
  static constexpr double kRadius = 5.0;

  voxblox::TsdfIntegrator::Config config_;
  voxblox_fast::TsdfIntegrator::Config fast_config_;

  std::unique_ptr<voxblox::TsdfIntegrator> baseline_integrator_;
  std::unique_ptr<voxblox_fast::TsdfIntegrator> fast_integrator_;

  std::unique_ptr<voxblox::Layer<voxblox::TsdfVoxel>> baseline_layer_;
  std::unique_ptr<voxblox_fast::Layer<voxblox_fast::TsdfVoxel>> fast_layer_;
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = 1;

  E2EBenchmarkFixture fixture;
  fixture.SetUp();
  fixture.CreateSphere(fixture.kRadius, fixture.kNumPoints);

  LOG(WARNING) << "Running baseline impl: " << FLAGS_use_baseline;
  if (FLAGS_use_baseline) {
    fixture.baseline_integrator_->integratePointCloud(
        fixture.T_G_C, fixture.sphere_points_C, fixture.colors_);
  } else {
    fixture.fast_integrator_->integratePointCloud(
        fixture.T_G_C, fixture.sphere_points_C, fixture.fast_colors_);
  }
  fixture.TearDown();
  return 0;
}

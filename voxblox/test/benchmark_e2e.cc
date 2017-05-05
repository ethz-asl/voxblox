#include <memory>

#include <benchmark/benchmark.h>
#include <benchmark_catkin/benchmark_entrypoint.h>

#include "voxblox/core/tsdf_map.h"
#include "voxblox/integrator/tsdf_integrator.h"
#include "voxblox/integrator/tsdf_integrator_fast.h"
#include "voxblox/simulation/sphere_simulator.h"

using namespace voxblox;  // NOLINT

class E2EBenchmark : public ::benchmark::Fixture {
 public:
  void SetUp(const ::benchmark::State& /*state*/) {
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

  void TearDown(const ::benchmark::State& /*state*/) {
    baseline_layer_.reset();
    fast_layer_.reset();
    baseline_integrator_.reset();
    fast_integrator_.reset();

    sphere_points_C.clear();
    colors.clear();
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
};

BENCHMARK_DEFINE_F(E2EBenchmark, BM_baseline)(benchmark::State& state) {
  while (state.KeepRunning()) {
    baseline_integrator_->integratePointCloud(T_G_C, sphere_points_C, colors);
  }
}
BENCHMARK_REGISTER_F(E2EBenchmark, BM_baseline);

BENCHMARK_DEFINE_F(E2EBenchmark, BM_fast)(benchmark::State& state) {
  while (state.KeepRunning()) {
    fast_integrator_->integratePointCloud(T_G_C, sphere_points_C, colors);
  }
}
BENCHMARK_REGISTER_F(E2EBenchmark, BM_fast);

BENCHMARKING_ENTRY_POINT

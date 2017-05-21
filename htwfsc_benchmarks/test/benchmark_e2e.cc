#include <memory>

#include <benchmark/benchmark.h>
#include <benchmark_catkin/benchmark_entrypoint.h>

#include "voxblox/core/tsdf_map.h"
#include "voxblox/integrator/tsdf_integrator.h"

#include "voxblox_fast/core/tsdf_map.h"
#include "voxblox_fast/integrator/tsdf_integrator.h"

#include "htwfsc_benchmarks/simulation/sphere_simulator.h"

class E2EBenchmark : public ::benchmark::Fixture {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 protected:
  void SetUp(const ::benchmark::State& /*state*/) {
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

  void TearDown(const ::benchmark::State& /*state*/) {
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
  static constexpr size_t kNumPoints = 200u;
  static constexpr double kRadius = 2.0;

  voxblox::TsdfIntegrator::Config config_;
  voxblox_fast::TsdfIntegrator::Config fast_config_;

  std::unique_ptr<voxblox::TsdfIntegrator> baseline_integrator_;
  std::unique_ptr<voxblox_fast::TsdfIntegrator> fast_integrator_;

  std::unique_ptr<voxblox::Layer<voxblox::TsdfVoxel>> baseline_layer_;
  std::unique_ptr<voxblox_fast::Layer<voxblox_fast::TsdfVoxel>> fast_layer_;
};

//////////////////////////////////////////////////////////////
// BENCHMARK CONSTANT NUMBER OF POINTS WITH CHANGING RADIUS //
//////////////////////////////////////////////////////////////

BENCHMARK_DEFINE_F(E2EBenchmark, BM_baseline_radius)(benchmark::State& state) {
  const double radius = static_cast<double>(state.range(0)) / 2.0;
  state.counters["radius_cm"] = radius * 100;
  CreateSphere(radius, kNumPoints);
  while (state.KeepRunning()) {
    baseline_integrator_->integratePointCloud(T_G_C, sphere_points_C, colors_);
  }
}
BENCHMARK_REGISTER_F(E2EBenchmark, BM_baseline_radius)->DenseRange(1, 10, 1);

BENCHMARK_DEFINE_F(E2EBenchmark, BM_fast_radius)(benchmark::State& state) {
  const double radius = static_cast<double>(state.range(0)) / 2.0;
  state.counters["radius_cm"] = radius * 100;
  CreateSphere(radius, kNumPoints);
  while (state.KeepRunning()) {
    fast_integrator_->integratePointCloud(T_G_C, sphere_points_C, fast_colors_);
  }
}
BENCHMARK_REGISTER_F(E2EBenchmark, BM_fast_radius)->DenseRange(1, 10, 1);

//////////////////////////////////////////////////////////////
// BENCHMARK CONSTANT RADIUS WITH CHANGING NUMBER OF POINTS //
//////////////////////////////////////////////////////////////

BENCHMARK_DEFINE_F(E2EBenchmark, BM_baseline_num_points)
(benchmark::State& state) {
  const size_t num_points = static_cast<double>(state.range(0));
  CreateSphere(kRadius, num_points);
  state.counters["num_points"] = sphere_points_C.size();
  while (state.KeepRunning()) {
    baseline_integrator_->integratePointCloud(T_G_C, sphere_points_C, colors_);
  }
}
BENCHMARK_REGISTER_F(E2EBenchmark, BM_baseline_num_points)
    ->RangeMultiplier(2)
    ->Range(1, 1e5);

BENCHMARK_DEFINE_F(E2EBenchmark, BM_fast_num_points)(benchmark::State& state) {
  const size_t num_points = static_cast<double>(state.range(0));
  CreateSphere(kRadius, num_points);
  state.counters["num_points"] = sphere_points_C.size();
  while (state.KeepRunning()) {
    fast_integrator_->integratePointCloud(T_G_C, sphere_points_C, fast_colors_);
  }
}
BENCHMARK_REGISTER_F(E2EBenchmark, BM_fast_num_points)
    ->RangeMultiplier(2)
    ->Range(1, 1e5);

BENCHMARKING_ENTRY_POINT

#include <random>

#include <benchmark/benchmark.h>
#include <benchmark_catkin/benchmark_entrypoint.h>
#include <eigen-checks/entrypoint.h>
#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>

#include "voxblox/core/tsdf_map.h"
#include "voxblox/integrator/integrator_utils.h"

#include "voxblox_fast/core/tsdf_map.h"
#include "voxblox_fast/integrator/integrator_utils.h"

#include "htwfsc_benchmarks/simulation/sphere_simulator.h"

class CastRayBenchmark : public ::benchmark::Fixture {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 protected:
  void SetUp(const ::benchmark::State& st) { T_G_C_ = voxblox::Transformation(); }

  void CreateSphere(const double radius, const size_t num_points) {
    sphere_points_G_.clear();
    htwfsc_benchmarks::sphere_sim::createSphere(kMean, kSigma, radius,
                                                num_points, &sphere_points_G_);
  }

  void TearDown(const ::benchmark::State&) { sphere_points_G_.clear(); }

  static constexpr double kMean = 0;
  static constexpr double kSigma = 0.05;
  static constexpr size_t kNumPoints = 200u;
  static constexpr double kRadius = 2.0;

  voxblox::Pointcloud sphere_points_G_;
  voxblox::Transformation T_G_C_;
};

//////////////////////////////////////////////////////////////
// BENCHMARK CONSTANT NUMBER OF POINTS WITH CHANGING RADIUS //
//////////////////////////////////////////////////////////////

BENCHMARK_DEFINE_F(CastRayBenchmark, BM_baseline_radius)
(benchmark::State& state) {
  const double radius = static_cast<double>(state.range(0)) / 2.0;
  state.counters["radius_cm"] = radius * 100;
  CreateSphere(radius, kNumPoints);
  while (state.KeepRunning()) {
    const voxblox::Point& origin = T_G_C_.getPosition();
    voxblox::IndexVector indices;
    for (const voxblox::Point& point : sphere_points_G_) {
      voxblox::castRay(origin, point, &indices);
    }
  }
}
BENCHMARK_REGISTER_F(CastRayBenchmark, BM_baseline_radius)
    ->DenseRange(1, 10, 1);

BENCHMARK_DEFINE_F(CastRayBenchmark, BM_fast_radius)(benchmark::State& state) {
  const double radius = static_cast<double>(state.range(0)) / 2.0;
  state.counters["radius_cm"] = radius * 100;
  CreateSphere(radius, kNumPoints);
  while (state.KeepRunning()) {
    const voxblox::Point& origin = T_G_C_.getPosition();
    voxblox::IndexVector indices;
    for (const voxblox::Point& point : sphere_points_G_) {
      voxblox_fast::castRay(origin, point, &indices);
    }
  }
}
BENCHMARK_REGISTER_F(CastRayBenchmark, BM_fast_radius)->DenseRange(1, 10, 1);

//////////////////////////////////////////////////////////////
// BENCHMARK CONSTANT RADIUS WITH CHANGING NUMBER OF POINTS //
//////////////////////////////////////////////////////////////

BENCHMARK_DEFINE_F(CastRayBenchmark, BM_baseline_num_points)
(benchmark::State& state) {
  const size_t num_points = static_cast<size_t>(state.range(0));
  CreateSphere(kRadius, num_points);
  state.counters["num_points"] = sphere_points_G_.size();
  while (state.KeepRunning()) {
    const voxblox::Point& origin = T_G_C_.getPosition();
    voxblox::IndexVector indices;
    for (const voxblox::Point& point : sphere_points_G_) {
      voxblox::castRay(origin, point, &indices);
    }
  }
}
BENCHMARK_REGISTER_F(CastRayBenchmark, BM_baseline_num_points)
    ->RangeMultiplier(2)
    ->Range(1, 1e5);

BENCHMARK_DEFINE_F(CastRayBenchmark, BM_fast)(benchmark::State& state) {
  const size_t num_points = static_cast<size_t>(state.range(0));
  CreateSphere(kRadius, num_points);
  state.counters["num_points"] = sphere_points_G_.size();
  while (state.KeepRunning()) {
    const voxblox_fast::Point& origin = T_G_C_.getPosition();
    voxblox_fast::IndexVector indices;
    for (const voxblox::Point& point : sphere_points_G_) {
      voxblox_fast::castRay(origin, point, &indices);
    }
  }
}
BENCHMARK_REGISTER_F(CastRayBenchmark, BM_fast)
    ->RangeMultiplier(2)
    ->Range(1, 1e5);

BENCHMARKING_ENTRY_POINT

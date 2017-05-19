#include <random>

#include <benchmark/benchmark.h>
#include <benchmark_catkin/benchmark_entrypoint.h>
#include <eigen-checks/entrypoint.h>
#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>

#include "../../htwfsc_benchmarks/include/voxblox/core/tsdf_map.h"
#include "../../htwfsc_benchmarks/include/voxblox/integrator/integrator_utils.h"
#include "../../htwfsc_benchmarks/include/voxblox/integrator/integrator_utils_fast.h"
#include "../../htwfsc_benchmarks/include/voxblox/simulation/sphere_simulator.h"

using namespace voxblox;  // NOLINT

class CastRayBenchmark : public ::benchmark::Fixture {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 protected:
  void SetUp(const ::benchmark::State& st) { T_G_C_ = Transformation(); }

  void CreateSphere(const double radius, const size_t num_points) {
    sphere_points_G_.clear();
    sphere_sim::createSphere(kMean, kSigma, radius, num_points,
                             &sphere_points_G_);
  }

  void TearDown(const ::benchmark::State&) { sphere_points_G_.clear(); }

  static constexpr double kMean = 0;
  static constexpr double kSigma = 0.05;
  static constexpr size_t kNumPoints = 200u;
  static constexpr double kRadius = 2.0;

  Pointcloud sphere_points_G_;
  Transformation T_G_C_;
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
    const Point& origin = T_G_C_.getPosition();
    IndexVector indices;
    for (const Point& point : sphere_points_G_) {
      castRay(origin, point, &indices);
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
    const Point& origin = T_G_C_.getPosition();
    IndexVector indices;
    for (const Point& point : sphere_points_G_) {
      fast::castRay(origin, point, &indices);
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
    const Point& origin = T_G_C_.getPosition();
    IndexVector indices;
    for (const Point& point : sphere_points_G_) {
      castRay(origin, point, &indices);
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
    const Point& origin = T_G_C_.getPosition();
    IndexVector indices;
    for (const Point& point : sphere_points_G_) {
      fast::castRay(origin, point, &indices);
    }
  }
}
BENCHMARK_REGISTER_F(CastRayBenchmark, BM_fast)
    ->RangeMultiplier(2)
    ->Range(1, 1e5);

BENCHMARKING_ENTRY_POINT

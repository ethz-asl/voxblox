#include <random>

#include <benchmark/benchmark.h>
#include <benchmark_helpers/benchmark_helpers.h>
#include <eigen-checks/entrypoint.h>
#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>

#include "voxblox/core/tsdf_map.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/integrator/integrator_utils_fast.h"
#include "voxblox/simulation/sphere_simulator.h"

using namespace voxblox;  // NOLINT

class CastRayBenchmark : public ::benchmark::Fixture {
 protected:
  void SetUp(const ::benchmark::State& st) { T_G_C_ = Transformation(); }

  void generateSphere(const size_t num_points) {

    constexpr double kMean = 0;
    constexpr double kSigma = 0.05;
    constexpr double kRadius = 1.0;

    sphere_sim::createSphere(kMean, kSigma, kRadius, num_points,
                             &sphere_points_G_);
  }

  void TearDown(const ::benchmark::State&) { sphere_points_G_.clear(); }

  Pointcloud sphere_points_G_;
  Transformation T_G_C_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

BENCHMARK_DEFINE_F(CastRayBenchmark, BM_baseline)(benchmark::State& state) {
  generateSphere(state.range(0));

  std::vector<AnyIndex, Eigen::aligned_allocator<AnyIndex>> indices;
  while (state.KeepRunning()) {
    const Point& origin = T_G_C_.getPosition();
    for (size_t i = 0u; i < state.range(0); ++i) {
      castRay(origin, sphere_points_G_[i], &indices);
    }
  }
}
BENCHMARK_REGISTER_F(CastRayBenchmark, BM_baseline)
    ->RangeMultiplier(2)
    ->Range(1, 8<<12);

BENCHMARK_DEFINE_F(CastRayBenchmark, BM_fast)(benchmark::State& state) {
  generateSphere(state.range(0));

  std::vector<AnyIndex, Eigen::aligned_allocator<AnyIndex>> indices;
  while (state.KeepRunning()) {
    const Point& origin = T_G_C_.getPosition();
    for (size_t i = 0u; i < state.range(0); ++i) {
      fast::castRay(origin, sphere_points_G_[i], &indices);
    }
  }
}
BENCHMARK_REGISTER_F(CastRayBenchmark, BM_fast)
->RangeMultiplier(2)
->Range(1, 8<<12);

BENCHMARKING_ENTRY_POINT

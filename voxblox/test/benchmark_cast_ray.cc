#include <random>

#include <benchmark/benchmark.h>
#include <benchmark_helpers/benchmark_helpers.h>
#include <eigen-checks/entrypoint.h>
#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>

#include "voxblox/core/tsdf_map.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/integrator/integrator_utils_fast.h"

using namespace voxblox;  // NOLINT

class CastRayBenchmark : public ::benchmark::Fixture {
 public:
  void SetUp(const ::benchmark::State& st) {
    std::default_random_engine gen(kSeed);
    std::uniform_real_distribution<double> xy_dist(-1.0, 1.0);
    std::normal_distribution<double> z_dist(3.0, 0.5);

    for (size_t i = 0u; i < kNumPointsToGenerate; ++i) {
      // Move first point forward and to the side not to keep it at
      // (0, 0, 0) constantly.
      Point point_1(0.01 * i, 0.02 * i, 0.5 * i);
      points_1_.push_back(point_1);

      Point shift(xy_dist(gen), xy_dist(gen), z_dist(gen));
      Point point_2 = point_1 + shift;
      points_2_.push_back(point_2);
    }
  }

  void TearDown(const ::benchmark::State&) {
    points_1_.clear();
    points_2_.clear();
  }

  static constexpr size_t kNumPointsToGenerate = 2000u;
  static constexpr size_t kSeed = 242u;

  std::vector<Point> points_1_;
  std::vector<Point> points_2_;
};

BENCHMARK_DEFINE_F(CastRayBenchmark, BM_castRay)(benchmark::State& state) {
  while (state.KeepRunning())
    for (size_t i = 0u; i < kNumPointsToGenerate; ++i) {
      std::vector<AnyIndex, Eigen::aligned_allocator<AnyIndex>> indices;
      voxblox::castRay(points_1_[i], points_2_[i], &indices);
    }
}
BENCHMARK_REGISTER_F(CastRayBenchmark, BM_castRay);

BENCHMARK_DEFINE_F(CastRayBenchmark, BM_castRay_fast)
(benchmark::State& state) {
  while (state.KeepRunning())
    for (size_t i = 0u; i < kNumPointsToGenerate; ++i) {
      std::vector<AnyIndex, Eigen::aligned_allocator<AnyIndex>> indices;
      voxblox::fast::castRay(points_1_[i], points_2_[i], &indices);
    }
}
BENCHMARK_REGISTER_F(CastRayBenchmark, BM_castRay_fast);

BENCHMARKING_ENTRY_POINT

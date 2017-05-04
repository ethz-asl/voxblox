#include <random>

#include <benchmark/benchmark.h>
#include <eigen-checks/entrypoint.h>
#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>

#include "voxblox/core/tsdf_map.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/integrator/integrator_utils_fast.h"

using namespace voxblox;  // NOLINT

class CastRayBenchmarkSetup {
 public:
  CastRayBenchmarkSetup() : points_1_(), points_2_() {
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

  static constexpr size_t kNumPointsToGenerate = 2000u;
  static constexpr size_t kSeed = 242u;

  std::vector<Point> points_1_;
  std::vector<Point> points_2_;
};

static void BM_cast_ray(benchmark::State& state) {
  CastRayBenchmarkSetup setup;
  while (state.KeepRunning())
    for (size_t i = 0u; i < setup.kNumPointsToGenerate; ++i) {
      std::vector<AnyIndex, Eigen::aligned_allocator<AnyIndex>> indices;
      voxblox::castRay(setup.points_1_[i], setup.points_2_[i], &indices);
    }
}
BENCHMARK(BM_cast_ray);

static void BM_cast_ray_fast(benchmark::State& state) {
  CastRayBenchmarkSetup setup;
  while (state.KeepRunning())
    for (size_t i = 0u; i < setup.kNumPointsToGenerate; ++i) {
      std::vector<AnyIndex, Eigen::aligned_allocator<AnyIndex>> indices;
      voxblox::castRay(setup.points_1_[i], setup.points_2_[i], &indices);
    }
}
BENCHMARK(BM_cast_ray_fast);

BENCHMARK_MAIN();

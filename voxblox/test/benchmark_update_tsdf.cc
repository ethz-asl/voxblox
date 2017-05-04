#include <benchmark/benchmark.h>
#include <benchmark_helpers/benchmark_helpers.h>

#include "voxblox/core/tsdf_map.h"
#include "voxblox/integrator/tsdf_integrator.h"

using namespace voxblox;  // NOLINT

class UpdateTsdfBenchmark : public ::benchmark::Fixture {
 public:
  void SetUp(const ::benchmark::State& st) {
    // TODO(mfehr): IMPLEMENT
  }

  void TearDown(const ::benchmark::State&) {
    // TODO(mfehr): IMPLEMENT
  }
};

BENCHMARK_DEFINE_F(UpdateTsdfBenchmark, BM_TODO)(benchmark::State& state) {
  while (state.KeepRunning()) {
    // TODO(mfehr): IMPLEMENT
  }
}
BENCHMARK_REGISTER_F(UpdateTsdfBenchmark, BM_TODO);

BENCHMARKING_ENTRY_POINT

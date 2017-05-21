#include <benchmark/benchmark.h>
#include <benchmark_catkin/benchmark_entrypoint.h>

#include "voxblox/core/tsdf_map.h"
#include "voxblox/integrator/tsdf_integrator.h"

class UpdateTsdfBenchmark : public ::benchmark::Fixture {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 protected:
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

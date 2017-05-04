#ifndef BENCHMARK_HELPERS_H_
#define BENCHMARK_HELPERS_H_

#include <benchmark/benchmark.h>
#include <benchmark/benchmark_api.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

class BenchmarkEntryPointBase {
 public:
  virtual ~BenchmarkEntryPointBase() {}

  // This function must be inline to avoid linker errors.
  inline int run(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    ::benchmark::Initialize(&argc, argv);
    ::benchmark::RunSpecifiedBenchmarks();
    return 0;
  }
};

#define BENCHMARKING_ENTRY_POINT                 \
  int main(int argc, char** argv) {              \
    BenchmarkEntryPointBase entry_point; \
    return entry_point.run(argc, argv);          \
  }

#endif /* BENCHMARK_HELPERS_H_ */

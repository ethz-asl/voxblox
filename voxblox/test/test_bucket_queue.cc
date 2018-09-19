#include <gtest/gtest.h>

#include "voxblox/utils/bucket_queue.h"

namespace voxblox {

double randomDoubleInRange(double f_min, double f_max) {
  double f = static_cast<double>(rand()) / RAND_MAX;  // NOLINT
  return f_min + f * (f_max - f_min);
}

TEST(BucketQueueTest, GetSmallestValue) {
  // Make sure this is deterministic.
  std::srand(0);

  // We're going to have a vector of random distances from 0.
  constexpr size_t kNumDistances = 100;
  constexpr double kMaxDistance = 20.0;
  constexpr int kNumBuckets = 20;

  // Create a bucket queue. It maps vector index to distance.
  BucketQueue<size_t> bucket_queue(kNumBuckets, kMaxDistance);

  std::vector<double> distances;
  distances.resize(kNumDistances);
  for (size_t i = 0; i < kNumDistances; i++) {
    distances[i] = randomDoubleInRange(-kMaxDistance, kMaxDistance);
    bucket_queue.push(i, distances[i]);
  }

  double last_val = 0.0;
  // Twice because we could switch buckets between.
  const double max_diff = 2 * kMaxDistance / (kNumBuckets - 1);
  while (!bucket_queue.empty()) {
    size_t ind = bucket_queue.front();
    double val = distances[ind];
    bucket_queue.pop();
    EXPECT_LT(std::abs(std::abs(val) - std::abs(last_val)), max_diff);
    last_val = val;
  }

  for (size_t i = 0; i < kNumDistances; i++) {
    bucket_queue.push(i, distances[i]);
  }
  last_val = 0.0;
  for (size_t i = 0; i < kNumDistances / 2.0; i++) {
    size_t ind = bucket_queue.front();
    double val = distances[ind];
    bucket_queue.pop();
    EXPECT_LT(std::abs(std::abs(val) - std::abs(last_val)), max_diff);
    last_val = val;
  }

  for (size_t i = 0; i < kNumDistances / 2.0; i++) {
    bucket_queue.push(i, distances[i]);
  }
  last_val = 0.0;
  while (!bucket_queue.empty()) {
    size_t ind = bucket_queue.front();
    double val = distances[ind];
    bucket_queue.pop();
    EXPECT_LT(std::abs(std::abs(val) - std::abs(last_val)), max_diff);
    last_val = val;
  }
}

}  // namespace voxblox

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}

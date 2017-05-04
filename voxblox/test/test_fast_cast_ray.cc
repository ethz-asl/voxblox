#include <random>

#include <eigen-checks/gtest.h>
#include <eigen-checks/entrypoint.h>
#include <gtest/gtest.h>
\
#include "voxblox/core/tsdf_map.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/integrator/integrator_utils_fast.h"

using namespace voxblox;  // NOLINT

class FastTsdfCastRayTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    std::default_random_engine gen(kSeed);
    std::uniform_real_distribution<double> xy_dist(-1.0, 1.0);
    std::normal_distribution<double> z_dist(3.0, 0.5);

    for (size_t i = 0u; i < kNumPointsToGenerate; ++i) {
        // Move first point forward and to the side not to keep it at
        // (0, 0, 0) constantly.
        Point point1(0.01 * i, 0.02 * i, 0.5 * i);
        points1_.push_back(point1);

        Point shift(xy_dist(gen), xy_dist(gen), z_dist(gen));
        Point point2 = point1 + shift;
        points2_.push_back(point2);
    }
  }

  static constexpr size_t kNumPointsToGenerate = 2000u;
  static constexpr size_t kSeed = 242u;

  std::vector<Point> points1_;
  std::vector<Point> points2_;
};

TEST_F(FastTsdfCastRayTest, CompareCastRayWithBaseline) {
    for (size_t i = 0u; i < kNumPointsToGenerate; ++i) {
      std::vector<AnyIndex, Eigen::aligned_allocator<AnyIndex>> indices_baseline;
      std::vector<AnyIndex, Eigen::aligned_allocator<AnyIndex>> indices_fast;
      voxblox::castRay(points1_[i], points2_[i], &indices_baseline); 
      voxblox::fast::castRay(points1_[i], points2_[i], &indices_fast); 

      ASSERT_EQ(indices_baseline, indices_fast);
    }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}
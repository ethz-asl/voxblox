#include <random>

#include <eigen-checks/entrypoint.h>
#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>

#include "voxblox/core/tsdf_map.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/integrator/integrator_utils_fast.h"
#include "voxblox/simulation/sphere_simulator.h"

using namespace voxblox;  // NOLINT

class FastTsdfCastRayTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    sphere_points.clear();

    constexpr double kMean = 0;
    constexpr double kSigma = 0.05;
    constexpr int kNumPoints = 100;
    constexpr double kRadius = 1.0;

    sphere_sim::createSphere(kMean, kSigma, kRadius, kNumPoints,
                             &sphere_points);

    origin = Point(0., 0., 0.);
  }

  Pointcloud sphere_points;
  Point origin;

  static constexpr size_t kNumCamerasToGenerate = 10u;
  static constexpr size_t kNumPointsToGenerate = 200u;

  static constexpr double kVoxelSize = 0.01;
  static constexpr size_t kVoxelsPerSide = 16u;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

TEST_F(FastTsdfCastRayTest, CompareCastRayWithBaseline) {
  std::vector<AnyIndex, Eigen::aligned_allocator<AnyIndex>> indices_baseline;
  std::vector<AnyIndex, Eigen::aligned_allocator<AnyIndex>> indices_fast;
  for (const Point& sphere_point : sphere_points) {
    indices_baseline.clear();
    castRay(origin, sphere_point, &indices_baseline);

    indices_fast.clear();
    fast::castRay(origin, sphere_point, &indices_fast);
    ASSERT_EQ(indices_baseline, indices_fast);
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}

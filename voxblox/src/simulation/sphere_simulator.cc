#include "voxblox/simulation/sphere_simulator.h"

#include <cmath>
#include <memory>
#include <random>

#include <algorithm>
#include <mutex>
#include <queue>
#include <thread>

#include <glog/logging.h>

namespace voxblox {
namespace sphere_sim {

void createSphere(const double mean, const double variance,
                  const double radius_m, const size_t num_points,
                  Pointcloud* points_3D) {
  CHECK_NOTNULL(points_3D)->resize(num_points);

  typedef std::random_device RandomDevice;
  typedef std::mt19937 Mt19937Distribution;
  typedef std::normal_distribution<double> NormalDistribution;

  // Distributions
  RandomDevice random_device;
  Mt19937Distribution mt_distribution(random_device());
  NormalDistribution normal_distribution(mean, variance);

  int n = num_points - 2;      // 1 for top and bottom point
  int s = round(sqrt(n - 1));  // sphere slices
  double step = 2 * radius_m / (static_cast<double>(s) + 1);

  {
    // top point
    Point& top_point = (*points_3D)[0];
    top_point.x() = 0.0;
    top_point.y() = 0.0;
    top_point.z() = radius_m;
  }

  double SumA = 0;
  for (int it = 1; it <= s; ++it) {
    double z = radius_m - static_cast<double>(it) * step;
    SumA += 2 * M_PI * sqrt(radius_m * radius_m - z * z);
  }

  for (int it = 1; it <= s; ++it) {
    double zi = radius_m - static_cast<double>(it) * step;
    double ri = sqrt(radius_m * radius_m - zi * zi);
    double Ai = 2 * M_PI * ri;
    int ni = round(n * Ai / SumA);
    double stepi = 2 * M_PI / static_cast<double>(ni);

    for (int it2 = 0; it2 < ni; ++it2) {
      double a;
      if (it % 2 == 0)
        a = static_cast<double>(it2) * stepi;
      else
        a = static_cast<double>((it2 + 0.5)) * stepi;

      double x, y, z;

      x = ri * cos(a) + normal_distribution(mt_distribution);
      y = ri * sin(a) + normal_distribution(mt_distribution);
      z = zi + normal_distribution(mt_distribution);

      Point& point = (*points_3D)[it];
      point.x() = x;
      point.y() = y;
      point.z() = z;
    }
  }

  {
    Point& bottom_point = (*points_3D)[num_points - 1u];
    bottom_point.x() = 0.0;
    bottom_point.y() = 0.0;
    bottom_point.z() = -radius_m;
  }
}

}  // namespace sphere_sim
}  // namespace voxblox

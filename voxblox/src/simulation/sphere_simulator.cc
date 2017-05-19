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
  CHECK_NOTNULL(points_3D)->reserve(num_points);

  typedef std::random_device RandomDevice;
  typedef std::mt19937 Mt19937Distribution;
  typedef std::normal_distribution<double> NormalDistribution;

  // Distributions
  RandomDevice random_device;
  Mt19937Distribution mt_distribution(random_device());
  NormalDistribution normal_distribution(mean, variance);

  const int n = num_points - 2;      // 1 for top and bottom point
  const int s = round(sqrt(n - 1));  // sphere slices
  const double step = 2 * radius_m / (static_cast<double>(s) + 1);

  // Top point.
  points_3D->emplace_back(0.0, 0.0, radius_m);

  if (num_points > 2u) {
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

        CHECK(!isnan(x));
        CHECK(!isnan(y));
        CHECK(!isnan(z));

        points_3D->emplace_back(x, y, z);
        if (points_3D->back().norm() < (0.5 * radius_m)) {
          LOG(WARNING) << "Point is too close to center!: "
                       << points_3D->back();
        }
      }
    }
  }

  if (num_points > 1u) {
    // Bottom point.
    points_3D->emplace_back(0.0, 0.0, -radius_m);
  }

  for (const Point& point : *points_3D) {
    CHECK(point.norm() > (0.5 * radius_m));
    CHECK(!isnan(point.x()));
    CHECK(!isnan(point.y()));
    CHECK(!isnan(point.z()));
  }
}

}  // namespace sphere_sim
}  // namespace voxblox

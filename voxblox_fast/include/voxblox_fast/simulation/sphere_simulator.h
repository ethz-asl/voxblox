#ifndef SPHERESIMULATOR_H_
#define SPHERESIMULATOR_H_

#include "voxblox_fast/core/common.h"

namespace voxblox_fast {
namespace sphere_sim {

void createSphere(const double mean, const double variance,
                  const double radius_m, const size_t num_points,
                  Pointcloud* points_3D);

}  // namespace sphere_sim
}  // namespace voxblox

#endif

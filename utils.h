#ifndef NBVP_VOXBLOX_UTILS_H_
#define NBVP_VOXBLOX_UTILS_H_

#include <Eigen/Core>
#include <random>

namespace nbvp_voxblox {

double randMToN(double m, double n) {
  return m + (rand() / (RAND_MAX / (n - m)));
}

}  // namespace nbvp_voxblox

#endif  // NBVP_VOXBLOX_UTILS_H_

#ifndef VOXBLOX_COMMON_H
#define VOXBLOX_COMMON_H

#include <Eigen/Core>

namespace voxblox {

typedef double FloatingPoint;
typedef Eigen::Vector<3, FloatingPoint> Coordinates;
typedef Eigen::Vector3i VoxelIndex;
}  // namespace voxblox

#endif

#ifndef VOXBLOX_CORE_COMMON_H
#define VOXBLOX_CORE_COMMON_H

#include <Eigen/Core>
#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h>
#include <memory>

namespace voxblox {

typedef double FloatingPoint;
typedef Eigen::Matrix<FloatingPoint, 3, 1> Coordinates;
typedef Eigen::Matrix<FloatingPoint, 3, 1> Point;
typedef Eigen::Matrix<FloatingPoint, 3, 1> Ray;
typedef Eigen::Vector3i VoxelIndex;
typedef Eigen::Vector3i BlockIndex;
typedef Eigen::Vector3i AnyIndex;

// Transformation type for defining sensor orientation.
typedef kindr::minimal::QuatTransformation Transformation;

struct Color {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t a;
};

// Pointcloud types for external interface.
typedef std::vector<Point> Pointcloud;
typedef std::vector<Color> Colors;

// TODO(mfehrenol): THIS COULD BE SLOW.
inline Eigen::Vector3i floorVectorAndDowncast(
    const Eigen::Matrix<FloatingPoint, 3, 1>& vector) {
  return Eigen::Vector3i(static_cast<int>(std::floor(vector.x())),
                         static_cast<int>(std::floor(vector.y())),
                         static_cast<int>(std::floor(vector.z())));
}

// TODO: define this.
// struct GlobalVoxelIndex {
//  BlockHash
//  LinearIndex

//  BlockIndex
//  VoxelIndex
//};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_COMMON_H

#ifndef VOXBLOX_CORE_COMMON_H
#define VOXBLOX_CORE_COMMON_H

#include <Eigen/Core>
#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h>
#include <memory>

namespace voxblox {

typedef double FloatingPoint;
typedef Eigen::Vector3d Coordinates;
typedef Eigen::Vector3i VoxelIndex;
typedef Eigen::Vector3i BlockIndex;

// Transformation type for defining sensor orientation.
typedef kindr::minimal::QuatTransformation Transformation;

// TODO: define this.
//struct GlobalVoxelIndex {
//  BlockHash
//  LinearIndex

//  BlockIndex
//  VoxelIndex
//};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_COMMON_H

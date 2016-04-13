#ifndef VOXBLOX_CORE_COMMON_H
#define VOXBLOX_CORE_COMMON_H

#include <glog/logging.h>
#include <Eigen/Core>
#include <memory>

namespace voxblox {

typedef double FloatingPoint;
typedef Eigen::Vector3d Coordinates;
typedef Eigen::Vector3i VoxelIndex;
typedef Eigen::Vector3i BlockIndex;

// TODO: define this.
//struct GlobalVoxelIndex {
//  BlockHash
//  LinearIndex

//  BlockIndex
//  VoxelIndex
//};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_COMMON_H

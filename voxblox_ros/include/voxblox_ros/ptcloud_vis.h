#ifndef VOXBLOX_ROS_MESH_VIS_H_
#define VOXBLOX_ROS_MESH_VIS_H_

#include <algorithm>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>

#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>

#include "voxblox_ros/conversions.h"

// This file contains a set of functions to visualize layers as pointclouds
// (or marker arrays) based on a passed-in function. It also offers some
// specializations of functions as samples.

namespace voxblox {

template <typename VoxelType>
typedef std::function<bool(const VoxelType& voxel, Color* color)> // NOLINT
      ShouldVisualizeVoxelFunctionType;





}  // namespace voxblox

#endif  // VOXBLOX_ROS_MESH_VIS_H_

#ifndef VOXBLOX_ROS_INCLUDE_VOXBLOX_ROS_RAY_CASTING_H_
#define VOXBLOX_ROS_INCLUDE_VOXBLOX_ROS_RAY_CASTING_H_

#include <vector>
#include <voxblox/core/common.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/integrator_utils.h>
#include <Eigen/Eigen>

namespace voxblox {

class RaySurfaceIntersectionFinder {
// NOTE: Sphere tracing is not used, because this class has to work on TSDFs
//       where speedups would be limited by their narrow truncation band and
//       sphere tracing might be less reliable with TSDF projective distances
 public:
  using TsdfLayer = voxblox::Layer<voxblox::TsdfVoxel>;

  explicit RaySurfaceIntersectionFinder(TsdfLayer* tsdf_layer_ptr)
      : tsdf_layer_ptr_(CHECK_NOTNULL(tsdf_layer_ptr)),
        voxel_size_(CHECK_NOTNULL(tsdf_layer_ptr)->voxel_size()),
        voxel_size_inv_(CHECK_NOTNULL(tsdf_layer_ptr)->voxel_size_inv()) {
  }

  bool findIntersectionPoint(
      const voxblox::Point& ray_start_point,
      const voxblox::Point& ray_end_point,
      voxblox::Point* closest_intersection_point);

 private:
  TsdfLayer* tsdf_layer_ptr_;
  const voxblox::FloatingPoint voxel_size_;
  const voxblox::FloatingPoint voxel_size_inv_;

  bool findClosestNegativeVoxelIndex(
      const voxblox::Point& ray_start_point,
      const voxblox::Point& ray_end_point,
      voxblox::GlobalIndex* first_negative_voxel_index);
};
}  // namespace voxblox

#endif  // VOXBLOX_ROS_INCLUDE_VOXBLOX_ROS_RAY_CASTING_H_

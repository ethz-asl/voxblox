#ifndef VOXBLOX_ROS_RAY_CASTING_H_
#define VOXBLOX_ROS_RAY_CASTING_H_

#include <voxblox/core/common.h>
#include <voxblox/core/tsdf_map.h>

namespace voxblox {

class RaySurfaceIntersectionFinder {
// NOTE: Sphere tracing is not used, because this class has to work on TSDFs
//       where speedups would be limited by their narrow truncation band and
//       sphere tracing might be less reliable with TSDF projective distances
 public:
  explicit RaySurfaceIntersectionFinder(Layer<TsdfVoxel>* tsdf_layer_ptr)
      : tsdf_layer_ptr_(CHECK_NOTNULL(tsdf_layer_ptr)),
        voxel_size_(CHECK_NOTNULL(tsdf_layer_ptr)->voxel_size()),
        voxel_size_inv_(CHECK_NOTNULL(tsdf_layer_ptr)->voxel_size_inv()) {}

  bool findIntersectionPoint(const Point& ray_start_point,
                             const Point& ray_end_point,
                             Point* closest_intersection_point);

 private:
  Layer<TsdfVoxel>* tsdf_layer_ptr_;
  const FloatingPoint voxel_size_;
  const FloatingPoint voxel_size_inv_;

  bool findClosestNegativeVoxelIndex(const Point& ray_start_point,
                                     const Point& ray_end_point,
                                     GlobalIndex* first_negative_voxel_index);
};
}  // namespace voxblox

#endif  // VOXBLOX_ROS_RAY_CASTING_H_

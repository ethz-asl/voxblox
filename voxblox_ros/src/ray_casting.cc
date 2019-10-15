#include "voxblox_ros/ray_casting.h"

namespace voxblox {

bool RaySurfaceIntersectionFinder::findIntersectionPoint(
    const voxblox::Point &ray_start_point,
    const Point &ray_end_point,
    voxblox::Point *closest_intersection_point) {
  CHECK_NOTNULL(closest_intersection_point);

  // Find the first negative voxel along the ray
  voxblox::GlobalIndex first_negative_voxel_index;
  if (findClosestNegativeVoxelIndex(
      ray_start_point, ray_end_point, &first_negative_voxel_index)) {
    // TODO(victorr): Compute the interpolated ray surface intersection,
    //                instead of just returning the first negative voxel pos
    *closest_intersection_point = voxblox::getCenterPointFromGridIndex(
        first_negative_voxel_index, voxel_size_);
    return true;
  } else {
    return false;
  }
}

bool RaySurfaceIntersectionFinder::findClosestNegativeVoxelIndex(
    const voxblox::Point &ray_start_point,
    const voxblox::Point &ray_end_point,
    voxblox::GlobalIndex *first_negative_voxel_index) {
  CHECK_NOTNULL(first_negative_voxel_index);

  // Create the ray caster
  voxblox::Point scaled_ray_start_point = ray_start_point * voxel_size_inv_;
  voxblox::Point scaled_ray_end_point = ray_end_point * voxel_size_inv_;
  voxblox::RayCaster ray_caster(scaled_ray_start_point, scaled_ray_end_point);

  // Walk along ray to find sign change
  voxblox::GlobalIndex global_voxel_index;
  while (ray_caster.nextRayIndex(&global_voxel_index)) {
    const voxblox::TsdfVoxel* voxel =
        tsdf_layer_ptr_->getVoxelPtrByGlobalIndex(global_voxel_index);

    if (voxel == nullptr || voxel->weight < 1e-6) {
      continue;
    }

    if (voxel->distance <= 0.0) {
      *first_negative_voxel_index = global_voxel_index;
      return true;
    }
  }
  return false;
}
}  // namespace voxblox

#ifndef VOXBLOX_UTILS_DISTANCE_UTILS_H_
#define VOXBLOX_UTILS_DISTANCE_UTILS_H_

#include "voxblox/core/block.h"
#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

/**
 * Returns true if there is a valid distance (intersection with the surface),
 * false otherwise (no known space, no surface boundary, etc.).
 * VoxelType must have a distance defined.
 */
template <typename VoxelType>
bool getSurfaceDistanceAlongRay(const Layer<VoxelType>& layer,
                                const Point& ray_origin,
                                const Point& bearing_vector,
                                FloatingPoint max_distance,
                                Point* triangulated_pose) {
  CHECK_NOTNULL(triangulated_pose);
  // Make sure bearing vector is normalized.
  const Point ray_direction = bearing_vector.normalized();

  // Keep track of current distance along the ray.
  FloatingPoint t = 0.0;
  // General ray equations: p = o + d * t

  // Cache voxel sizes for faster moving.
  const FloatingPoint voxel_size = layer.voxel_size();

  bool surface_found = false;

  while (t < max_distance) {
    const Point current_pos = ray_origin + t * ray_direction;
    typename Block<VoxelType>::ConstPtr block_ptr =
        layer.getBlockPtrByCoordinates(current_pos);
    if (!block_ptr) {
      // How much should we move up by? 1 voxel? 1 block? Could be close to the
      // block boundary though....
      // Let's start with the naive choice: 1 voxel.
      t += voxel_size;
      continue;
    }
    const VoxelType& voxel = block_ptr->getVoxelByCoordinates(current_pos);
    if (voxel.weight < 1e-6) {
      t += voxel_size;
      continue;
    }
    if (voxel.distance > voxel_size) {
      // Move forward as much as we can.
      t += voxel.distance;
      continue;
    }
    // The other cases are when we are actually at or behind the surface.
    // TODO(helenol): these are gross generalizations; use actual interpolation
    // to get the surface boundary.
    if (voxel.distance < 0.0) {
      surface_found = true;
      break;
    }
    if (voxel.distance < voxel_size) {
      // Also assume this is finding the surface.
      surface_found = true;
      t += voxel.distance;
      break;
    }
    // Default case...
    t += voxel_size;
  }

  if (surface_found) {
    *triangulated_pose = ray_origin + t * ray_direction;
  }

  return surface_found;
}

}  // namespace voxblox

#endif  // VOXBLOX_UTILS_DISTANCE_UTILS_H_

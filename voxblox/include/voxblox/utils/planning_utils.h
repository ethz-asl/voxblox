#ifndef VOXBLOX_UTILS_PLANNING_UTILS_H_
#define VOXBLOX_UTILS_PLANNING_UTILS_H_

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

namespace utils {

/// Gets the indices of all points within the sphere.
template <typename VoxelType>
void getSphereAroundPoint(const Layer<VoxelType>& layer, const Point& center,
                          FloatingPoint radius,
                          HierarchicalIndexMap* block_voxel_list);

/**
 * Gets the indices of all points around a sphere, and also allocates any
 * blocks that don't already exist.
 */
template <typename VoxelType>
void getAndAllocateSphereAroundPoint(const Point& center, FloatingPoint radius,
                                     Layer<VoxelType>* layer,
                                     HierarchicalIndexMap* block_voxel_list);

/**
 * Tools for manually editing a set of voxels. Sets the values around a sphere
 * to be artifically free or occupied, and marks them as hallucinated.
 */
template <typename VoxelType>
void fillSphereAroundPoint(const Point& center, const FloatingPoint radius,
                           const FloatingPoint max_distance_m,
                           Layer<VoxelType>* layer);
template <typename VoxelType>
void clearSphereAroundPoint(const Point& center, const FloatingPoint radius,
                            const FloatingPoint max_distance_m,
                            Layer<VoxelType>* layer);

/**
 * Utility function to get map bounds from an arbitrary layer.
 * Only accurate to block level (i.e., outer bounds of allocated blocks).
 */
template <typename VoxelType>
void computeMapBoundsFromLayer(const voxblox::Layer<VoxelType>& layer,
                               Eigen::Vector3d* lower_bound,
                               Eigen::Vector3d* upper_bound);

}  // namespace utils
}  // namespace voxblox

#include "voxblox/utils/planning_utils_inl.h"

#endif  // VOXBLOX_UTILS_PLANNING_UTILS_H_

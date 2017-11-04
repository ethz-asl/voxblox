#ifndef VOXBLOX_UTILS_PLANNING_UTILS_H_
#define VOXBLOX_UTILS_PLANNING_UTILS_H_

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

namespace voxblox {
namespace utils {

// Tools for manually editing a set of voxels
void fillSphereAroundPoint(Layer<EsdfVoxel>* layer, const Point& center,
                           const FloatingPoint radius);
void clearSphereAroundPoint(Layer<EsdfVoxel>* layer, const Point& center,
                            const FloatingPoint radius);

/* TODO(mereweth@jpl.nasa.gov) - implement something that makes more sense for
 * projective distance. This allows marking a known area as free or clear and
 * preserving that marking across TSDF updates
 */

/* This function sets all voxels within a Euclidean distance of the center to
 * have the specified distance values
 */
inline void fillSphereAroundPoint(Layer<EsdfVoxel>* layer,
                                  const Point& center,
                                  const FloatingPoint radius) {
  CHECK_NOTNULL(layer);
  // search a cube with side length 2*radius
  const FloatingPoint voxel_size = layer->voxel_size();
  for (FloatingPoint x = -radius; x <= radius; x += voxel_size) {
   for (FloatingPoint y = -radius; y <= radius; y += voxel_size) {
     for (FloatingPoint z = -radius; z <= radius; z += voxel_size) {
       Point point(x, y, z);

       // check if point is inside the spheres radius
       const FloatingPoint radius_squared_norm = radius * radius;
       if (point.squaredNorm() <= radius_squared_norm) {
         // convert to global coordinate
         point += center;

         const Block<EsdfVoxel>::Ptr block_ptr =
             layer->allocateBlockPtrByCoordinates(point);
         const VoxelIndex voxel_index =
             block_ptr->computeVoxelIndexFromCoordinates(point);
         EsdfVoxel& voxel = block_ptr->getVoxelByVoxelIndex(voxel_index);
         const Point voxel_center =
             block_ptr->computeCoordinatesFromVoxelIndex(voxel_index);

         const Point voxel_center_vec = voxel_center - point;

         // how far is voxel from center of filled sphere
         const FloatingPoint new_distance = sqrt(voxel_center_vec.squaredNorm());

         if ((!voxel.observed)               ||
             (new_distance < voxel.distance)) {
           voxel.distance = new_distance;
           voxel.observed = true;
           voxel.fixed = true;
           block_ptr->updated() = true;
           block_ptr->has_data() = true;
         }
       }
     }
   }
  }
}

/* This function sets all voxels within a Euclidean distance of the center to
 * have the specified distance values
 */
inline void clearSphereAroundPoint(Layer<EsdfVoxel>* layer,
                                   const Point& center,
                                   const FloatingPoint radius) {
  CHECK_NOTNULL(layer);
  // search a cube with side length 2*radius
  const FloatingPoint voxel_size = layer->voxel_size();
  for (FloatingPoint x = -radius; x <= radius; x += voxel_size) {
    for (FloatingPoint y = -radius; y <= radius; y += voxel_size) {
      for (FloatingPoint z = -radius; z <= radius; z += voxel_size) {
        Point point(x, y, z);

        // check if point is inside the spheres radius
        const FloatingPoint radius_squared_norm = radius * radius;
        if (point.squaredNorm() <= radius_squared_norm) {
          // convert to global coordinate
          point += center;

          const typename Block<EsdfVoxel>::Ptr block_ptr =
              layer->allocateBlockPtrByCoordinates(point);
          const VoxelIndex voxel_index =
              block_ptr->computeVoxelIndexFromCoordinates(point);
          EsdfVoxel& voxel = block_ptr->getVoxelByVoxelIndex(voxel_index);
          const Point voxel_center =
              block_ptr->computeCoordinatesFromVoxelIndex(voxel_index);

          const Point voxel_center_vec = voxel_center - point;

          // how far is voxel from edge of free sphere
          const FloatingPoint new_distance =
              sqrt(radius_squared_norm) - sqrt(voxel_center_vec.squaredNorm());

          if ((!voxel.observed)               ||
              (new_distance > voxel.distance)) {
            voxel.distance = new_distance;
            voxel.observed = true;
            voxel.fixed = true;
            block_ptr->updated() = true;
            block_ptr->has_data() = true;
          }
        }
      }
    }
  }
}

}  // namespace utils
}  // namespace voxblox

#endif //VOXBLOX_UTILS_PLANNING_UTILS_H_

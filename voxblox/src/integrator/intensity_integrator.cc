#include "voxblox/integrator/intensity_integrator.h"

#include "voxblox/utils/distance_utils.h"

namespace voxblox {

IntensityIntegrator::IntensityIntegrator(const Layer<TsdfVoxel>& tsdf_layer,
                                         Layer<IntensityVoxel>* intensity_layer)
    : max_distance_(5.0),
      max_weight_(100.0),
      intensity_prop_voxel_radius_(2),
      tsdf_layer_(tsdf_layer),
      intensity_layer_(intensity_layer) {}

void IntensityIntegrator::addIntensityBearingVectors(
    const Point& origin, const Pointcloud& bearing_vectors,
    const std::vector<float>& intensities) {
  timing::Timer intensity_timer("intensity/integrate");

  CHECK_EQ(bearing_vectors.size(), intensities.size())
      << "Intensity and bearing vector size does not match!";
  const FloatingPoint voxel_size = tsdf_layer_.voxel_size();

  for (size_t i = 0; i < bearing_vectors.size(); ++i) {
    Point surface_intersection = Point::Zero();
    // Cast ray from the origin in the direction of the bearing vector until
    // finding an intersection with a surface.
    bool success = getSurfaceDistanceAlongRay<TsdfVoxel>(
        tsdf_layer_, origin, bearing_vectors[i], max_distance_,
        &surface_intersection);

    if (!success) {
      continue;
    }

    // Now look up the matching voxels in the intensity layer and mark them.
    // Let's just start with 1.
    Block<IntensityVoxel>::Ptr block_ptr =
        intensity_layer_->allocateBlockPtrByCoordinates(surface_intersection);
    IntensityVoxel& voxel =
        block_ptr->getVoxelByCoordinates(surface_intersection);
    voxel.intensity =
        (voxel.weight * voxel.intensity + intensities[i]) / (voxel.weight + 1);
    voxel.weight += 1.0;
    if (voxel.weight > max_weight_) {
      voxel.weight = max_weight_;
    }

    // Now check the surrounding voxels along the bearing vector. If they have
    // never been observed, then fill in their value. Otherwise don't.
    Point close_voxel = surface_intersection;
    for (int voxel_offset = -intensity_prop_voxel_radius_;
         voxel_offset <= intensity_prop_voxel_radius_; voxel_offset++) {
      close_voxel =
          surface_intersection + bearing_vectors[i] * voxel_offset * voxel_size;
      Block<IntensityVoxel>::Ptr new_block_ptr =
          intensity_layer_->allocateBlockPtrByCoordinates(close_voxel);
      IntensityVoxel& new_voxel = block_ptr->getVoxelByCoordinates(close_voxel);
      if (new_voxel.weight < 1e-6) {
        new_voxel.intensity = intensities[i];
        new_voxel.weight += 1.0;
      }
    }
  }
}

}  // namespace voxblox

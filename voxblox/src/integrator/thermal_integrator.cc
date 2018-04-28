#include "voxblox/integrator/thermal_integrator.h"

#include "voxblox/utils/distance_utils.h"

namespace voxblox {

ThermalIntegrator::ThermalIntegrator(const Layer<TsdfVoxel>& tsdf_layer,
                                     Layer<ThermalVoxel>* thermal_layer)
    : max_distance_(5.0),
      max_observations_(100),
      tsdf_layer_(tsdf_layer),
      thermal_layer_(thermal_layer) {}

void ThermalIntegrator::addThermalBearingVectors(
    const Point& origin, const Pointcloud& bearing_vectors,
    const std::vector<float>& temperatures) {
  timing::Timer thermal_timer("thermal/integrate");

  CHECK_EQ(bearing_vectors.size(), temperatures.size())
      << "Temperature and bearing vector size does not match!";
  const int temperature_prop_voxel_radius = 2;
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

    // Now look up the matching voxels in the thermal layer and mark them.
    // Let's just start with 1.
    Block<ThermalVoxel>::Ptr block_ptr =
        thermal_layer_->allocateBlockPtrByCoordinates(surface_intersection);
    ThermalVoxel& voxel =
        block_ptr->getVoxelByCoordinates(surface_intersection);
    voxel.temperature =
        (voxel.observations * voxel.temperature + temperatures[i]) /
        (voxel.observations + 1);
    voxel.observations++;
    if (voxel.observations > max_observations_) {
      voxel.observations = max_observations_;
    }

    // Now check the surrounding voxels. If they have never been observed,
    // then fill in their value. Otherwise don't.
    Point close_voxel = surface_intersection;
    for (int voxel_offset = -temperature_prop_voxel_radius;
         voxel_offset <= temperature_prop_voxel_radius; voxel_offset++) {
      close_voxel =
          surface_intersection + bearing_vectors[i] * voxel_offset * voxel_size;
      Block<ThermalVoxel>::Ptr new_block_ptr =
          thermal_layer_->allocateBlockPtrByCoordinates(close_voxel);
      ThermalVoxel& new_voxel = block_ptr->getVoxelByCoordinates(close_voxel);
      if (new_voxel.observations == 0) {
        new_voxel.temperature = temperatures[i];
        new_voxel.observations++;
      }
    }
  }
}

}  // namespace voxblox

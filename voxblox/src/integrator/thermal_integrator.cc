#include "voxblox/integrator/thermal_integrator.h"

namespace voxblox {

ThermalIntegrator::ThermalIntegrator(const Layer<TsdfVoxel>& tsdf_layer,
                                     Layer<ThermalVoxel>* thermal_layer)
    : tsdf_layer_(tsdf_layer), thermal_layer_(thermal_layer) {}

void ThermalIntegrator::addThermalBearingVectors(
    const Point& origin, const Pointcloud& bearing_vectors,
    const std::vector<float>& temperatures) {
  CHECK_EQ(bearing_vectors.size(), temperatures.size())
      << "Temperature and bearing vector size does not match!";

  for (size_t i = 0; i < bearing_vectors.size(); ++i) {
    // Cast ray from the origin in the direction of the bearing vector until
    // finding an intersection with a surface.






  }



}

}  // namespace voxblox

#ifndef VOXBLOX_INTEGRATOR_THERMAL_INTEGRATOR_H_
#define VOXBLOX_INTEGRATOR_THERMAL_INTEGRATOR_H_

#include <glog/logging.h>
#include <Eigen/Core>
#include <algorithm>
#include <queue>
#include <utility>
#include <vector>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/utils/timing.h"

namespace voxblox {

class ThermalIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ThermalIntegrator(const Layer<TsdfVoxel>& tsdf_layer,
                    Layer<ThermalVoxel>* thermal_layer);

  // Set the max distance for projecting into the TSDF layer.
  void setMaxDistance(FloatingPoint max_distance) {
    max_distance_ = max_distance;
  }

  // Integrates temperatures into the thermal layer by projecting normalized
  // bearing vectors (in the WORLD coordinate frame) from the origin (also in
  // the world coordinate frame) into the TSDF layer, and then setting the
  // temperatures near the surface boundaries.
  void addThermalBearingVectors(const Point& origin,
                                const Pointcloud& bearing_vectors,
                                const std::vector<float>& temperatures);

 private:
  FloatingPoint max_distance_;
  int max_observations_;

  const Layer<TsdfVoxel>& tsdf_layer_;
  Layer<ThermalVoxel>* thermal_layer_;
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_THERMAL_INTEGRATOR_H_

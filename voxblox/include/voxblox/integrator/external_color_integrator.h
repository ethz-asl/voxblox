#ifndef VOXBLOX_INCLUDE_VOXBLOX_INTEGRATOR_EXTERNAL_COLOR_INTEGRATOR_H_
#define VOXBLOX_INCLUDE_VOXBLOX_INTEGRATOR_EXTERNAL_COLOR_INTEGRATOR_H_

#include <glog/logging.h>
#include <Eigen/Core>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/utils/timing.h"

namespace voxblox {
class ExternalColorIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ExternalColorIntegrator(const Layer<TsdfVoxel> &tsdf_layer,
                          Layer<ColorVoxel>* color_layer);

  void integrateColorBearingVectors(const Point& origin,
                                    const Pointcloud& bearing_vectors,
                                    const Colors& colors);

 private:
  static constexpr float measurement_weight_ = 1.0f;

  FloatingPoint max_distance_;
  float max_weight_;

  // Number of voxels to propagate from the surface along the bearing vector
  int intensity_prop_voxel_radius_;

  const Layer<TsdfVoxel>& tsdf_layer_;
  Layer<ColorVoxel>* color_layer_;
};
}

#endif //VOXBLOX_INCLUDE_VOXBLOX_INTEGRATOR_EXTERNAL_COLOR_INTEGRATOR_H_

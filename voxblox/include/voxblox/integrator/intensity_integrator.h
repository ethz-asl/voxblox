#ifndef VOXBLOX_INTEGRATOR_INTENSITY_INTEGRATOR_H_
#define VOXBLOX_INTEGRATOR_INTENSITY_INTEGRATOR_H_

#include <algorithm>
#include <queue>
#include <utility>
#include <vector>

#include <glog/logging.h>
#include <Eigen/Core>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/utils/timing.h"

namespace voxblox {

/**
 * Integrates intensities from a set of bearing vectors (i.e., an intensity
 * image, such as a thermal image) by projecting them onto the TSDF surface
 * and coloring near the surface crossing.
 */
class IntensityIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  IntensityIntegrator(const Layer<TsdfVoxel>& tsdf_layer,
                      Layer<IntensityVoxel>* intensity_layer);

  /// Set the max distance for projecting into the TSDF layer.
  void setMaxDistance(const FloatingPoint max_distance) {
    max_distance_ = max_distance;
  }
  FloatingPoint getMaxDistance() const { return max_distance_; }

  /**
   * Integrates intensities into the intensity layer by projecting normalized
   * bearing vectors (in the WORLD coordinate frame) from the origin (also in
   * the world coordinate frame) into the TSDF layer, and then setting the
   * intensities near the surface boundaries.
   */
  void addIntensityBearingVectors(const Point& origin,
                                  const Pointcloud& bearing_vectors,
                                  const std::vector<float>& intensities);

 private:
  FloatingPoint max_distance_;
  float max_weight_;
  /// Number of voxels to propagate from the surface along the bearing vector.
  int intensity_prop_voxel_radius_;

  const Layer<TsdfVoxel>& tsdf_layer_;
  Layer<IntensityVoxel>* intensity_layer_;
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_INTENSITY_INTEGRATOR_H_

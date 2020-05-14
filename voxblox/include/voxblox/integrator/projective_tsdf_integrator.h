#ifndef VOXBLOX_INCLUDE_VOXBLOX_INTEGRATOR_PROJECTIVE_TSDF_INTEGRATOR_H_
#define VOXBLOX_INCLUDE_VOXBLOX_INTEGRATOR_PROJECTIVE_TSDF_INTEGRATOR_H_

#include <vector>

#include "voxblox/integrator/tsdf_integrator.h"

namespace voxblox {
enum class InterpolationScheme {
  kNearestNeighbor,
  kMinNeighbor,
  kBilinear,
  kAdaptive
};
template <InterpolationScheme interpolation_scheme>
class ProjectiveTsdfIntegrator : public voxblox::TsdfIntegratorBase {
 public:
  ProjectiveTsdfIntegrator(const Config& config,
                           voxblox::Layer<voxblox::TsdfVoxel>* layer);

  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C, const Colors& colors,
                           const bool freespace_points = false) override;
  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C, const Colors& colors,
                           const bool freespace_points = false,
                           const bool deintegrate = false);

 private:
  // Sensor model
  const int horizontal_resolution_;
  const int vertical_resolution_;
  const double vertical_fov_rad_;
  const double ray_intersections_per_distance_squared_;

  Eigen::MatrixXf range_image_;

  // Calculate the number of voxels per block
  const size_t num_voxels_per_block_;

  void parsePointcloud(const Transformation& T_G_C, const Pointcloud& points_C,
                       Eigen::MatrixXf* range_image,
                       voxblox::IndexSet* touched_block_indices);

  void updateTsdfBlocks(const Transformation& T_G_C,
                        const Eigen::MatrixXf& range_image,
                        const voxblox::IndexSet& touched_block_indices,
                        const bool deintegrate = false);
  inline void updateTsdfVoxel(const Transformation& T_G_C,
                              const Eigen::MatrixXf& range_image,
                              const Point& t_C_voxel, TsdfVoxel* tsdf_voxel,
                              const bool deintegrate = false);

  template <typename T>
  Point imageToBearing(const T h, const T w);

  template <typename T>
  bool bearingToImage(const Point& b_C_normalized, T* h, T* w);

  inline float interpolate(const Eigen::MatrixXf& range_image, const float h,
                           const float w);
};
}  // namespace voxblox

#include "voxblox/integrator/projective_tsdf_integrator_inl.h"

#endif  // VOXBLOX_INCLUDE_VOXBLOX_INTEGRATOR_PROJECTIVE_TSDF_INTEGRATOR_H_

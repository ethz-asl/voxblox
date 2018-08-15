#ifndef VOXBLOX_ROS_ROS_PARAMS_H_
#define VOXBLOX_ROS_ROS_PARAMS_H_

#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>

namespace voxblox {

inline TsdfMap::Config getTsdfMapConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  TsdfMap::Config tsdf_config;

  // Workaround for OS X on mac mini not having specializations for float
  // for some reason.
  double voxel_size = tsdf_config.tsdf_voxel_size;
  int voxels_per_side = tsdf_config.tsdf_voxels_per_side;
  nh_private.param("tsdf_voxel_size", voxel_size, voxel_size);
  nh_private.param("tsdf_voxels_per_side", voxels_per_side, voxels_per_side);
  if (!isPowerOfTwo(voxels_per_side)) {
    ROS_ERROR("voxels_per_side must be a power of 2, setting to default value");
    voxels_per_side = tsdf_config.tsdf_voxels_per_side;
  }

  tsdf_config.tsdf_voxel_size = static_cast<FloatingPoint>(voxel_size);
  tsdf_config.tsdf_voxels_per_side = voxels_per_side;

  return tsdf_config;
}

inline TsdfIntegratorBase::Config getTsdfIntegratorConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  TsdfIntegratorBase::Config integrator_config;

  integrator_config.voxel_carving_enabled = true;

  const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(nh_private);
  integrator_config.default_truncation_distance =
      tsdf_config.tsdf_voxel_size * 4;

  double truncation_distance = integrator_config.default_truncation_distance;
  double max_weight = integrator_config.max_weight;
  nh_private.param("voxel_carving_enabled",
                   integrator_config.voxel_carving_enabled,
                   integrator_config.voxel_carving_enabled);
  nh_private.param("truncation_distance", truncation_distance,
                   truncation_distance);
  nh_private.param("max_ray_length_m", integrator_config.max_ray_length_m,
                   integrator_config.max_ray_length_m);
  nh_private.param("min_ray_length_m", integrator_config.min_ray_length_m,
                   integrator_config.min_ray_length_m);
  nh_private.param("max_weight", max_weight, max_weight);
  nh_private.param("use_const_weight", integrator_config.use_const_weight,
                   integrator_config.use_const_weight);
  nh_private.param("allow_clear", integrator_config.allow_clear,
                   integrator_config.allow_clear);
  nh_private.param("start_voxel_subsampling_factor",
                   integrator_config.start_voxel_subsampling_factor,
                   integrator_config.start_voxel_subsampling_factor);
  nh_private.param("max_consecutive_ray_collisions",
                   integrator_config.max_consecutive_ray_collisions,
                   integrator_config.max_consecutive_ray_collisions);
  nh_private.param("clear_checks_every_n_frames",
                   integrator_config.clear_checks_every_n_frames,
                   integrator_config.clear_checks_every_n_frames);
  nh_private.param("max_integration_time_s",
                   integrator_config.max_integration_time_s,
                   integrator_config.max_integration_time_s);
  nh_private.param("anti_grazing", integrator_config.enable_anti_grazing,
                   integrator_config.enable_anti_grazing);
  integrator_config.default_truncation_distance =
      static_cast<float>(truncation_distance);
  integrator_config.max_weight = static_cast<float>(max_weight);

  return integrator_config;
}

inline EsdfMap::Config getEsdfMapConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  EsdfMap::Config esdf_config;

  const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(nh_private);
  esdf_config.esdf_voxel_size = tsdf_config.tsdf_voxel_size;
  esdf_config.esdf_voxels_per_side = tsdf_config.tsdf_voxels_per_side;

  return esdf_config;
}

inline EsdfIntegrator::Config getEsdfIntegratorConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  EsdfIntegrator::Config esdf_integrator_config;

  TsdfIntegratorBase::Config tsdf_integrator_config =
      getTsdfIntegratorConfigFromRosParam(nh_private);

  esdf_integrator_config.min_distance_m =
      tsdf_integrator_config.default_truncation_distance;

  nh_private.param("esdf_max_distance_m", esdf_integrator_config.max_distance_m,
                   esdf_integrator_config.max_distance_m);
  nh_private.param("esdf_default_distance_m",
                   esdf_integrator_config.default_distance_m,
                   esdf_integrator_config.default_distance_m);
  nh_private.param("esdf_min_diff_m", esdf_integrator_config.min_diff_m,
                   esdf_integrator_config.min_diff_m);

  if (esdf_integrator_config.default_distance_m <
      esdf_integrator_config.max_distance_m) {
    esdf_integrator_config.default_distance_m =
        esdf_integrator_config.max_distance_m;
  }

  return esdf_integrator_config;
}

}  // namespace voxblox

#endif  // VOXBLOX_ROS_ROS_PARAMS_H_

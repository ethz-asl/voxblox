#include <voxblox_ros/conversions.h>

#include "voxblox_ros/esdf_server.h"

namespace voxblox {

EsdfServer::EsdfServer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private)
    : TsdfServer(nh, nh_private), clear_sphere_for_planning_(false) {
  esdf_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >("esdf_pointcloud",
                                                              1, true);
  esdf_slice_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "esdf_slice", 1, true);

  esdf_map_pub_ =
      nh_private_.advertise<voxblox_msgs::Layer>("esdf_map_out", 1, false);

  esdf_map_sub_ = nh_private_.subscribe("esdf_map_in", 1,
                                        &EsdfServer::esdfMapCallback, this);

  EsdfMap::Config esdf_config;

  // TODO(helenol): in the future allow different ESDF and TSDF voxel sizes...
  nh_private_.param("tsdf_voxel_size", esdf_config.esdf_voxel_size,
                    esdf_config.esdf_voxel_size);
  // No specialization for ros param for size_t, so have to do this annoying
  // workaround.
  int voxels_per_side = esdf_config.esdf_voxels_per_side;
  nh_private_.param("tsdf_voxels_per_side", voxels_per_side, voxels_per_side);
  esdf_config.esdf_voxels_per_side = voxels_per_side;

  esdf_map_.reset(new EsdfMap(esdf_config));

  EsdfIntegrator::Config esdf_integrator_config;
  // Make sure that this is the same as the truncation distance OR SMALLER!
  esdf_integrator_config.min_distance_m = esdf_config.esdf_voxel_size;
  // esdf_integrator_config.min_distance_m =
  //    tsdf_integrator_->getConfig().default_truncation_distance;
  nh_private_.param("esdf_max_distance_m",
                    esdf_integrator_config.max_distance_m,
                    esdf_integrator_config.max_distance_m);
  nh_private_.param("esdf_default_distance_m",
                    esdf_integrator_config.default_distance_m,
                    esdf_integrator_config.default_distance_m);

  esdf_integrator_.reset(new EsdfIntegrator(esdf_integrator_config,
                                            tsdf_map_->getTsdfLayerPtr(),
                                            esdf_map_->getEsdfLayerPtr()));

  // Whether to clear each new pose as it comes in, and then set a sphere
  // around it to occupied.
  nh_private_.param("clear_sphere_for_planning", clear_sphere_for_planning_,
                    clear_sphere_for_planning_);
}

void EsdfServer::publishAllUpdatedEsdfVoxels() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(), &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  esdf_pointcloud_pub_.publish(pointcloud);
}

void EsdfServer::publishSlices() {
  TsdfServer::publishSlices();

  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  constexpr int kZAxisIndex = 2;
  createDistancePointcloudFromEsdfLayerSlice(
      esdf_map_->getEsdfLayer(), kZAxisIndex, slice_level_, &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  esdf_slice_pub_.publish(pointcloud);
}

bool EsdfServer::generateEsdfCallback(
    std_srvs::Empty::Request& request,      // NOLINT
    std_srvs::Empty::Response& response) {  // NOLINT
  const bool clear_esdf = true;
  if (clear_esdf) {
    esdf_integrator_->updateFromTsdfLayerBatch();
  } else {
    const bool clear_updated_flag = true;
    esdf_integrator_->updateFromTsdfLayer(clear_updated_flag);
  }
  publishAllUpdatedEsdfVoxels();
  publishSlices();
  return true;
}

void EsdfServer::updateMeshEvent(const ros::TimerEvent& event) {
  // Also update the ESDF now, if there's any blocks in the TSDF.
  if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    const bool clear_updated_flag_esdf = false;
    esdf_integrator_->updateFromTsdfLayer(clear_updated_flag_esdf);
  }
  publishAllUpdatedEsdfVoxels();

  if (esdf_map_pub_.getNumSubscribers() > 0) {
    // TODO(helenol): make param!
    const bool only_updated = false;
    voxblox_msgs::Layer layer_msg;
    serializeLayerAsMsg<EsdfVoxel>(esdf_map_->getEsdfLayer(), only_updated,
                                   &layer_msg);
    esdf_map_pub_.publish(layer_msg);
  }

  TsdfServer::updateMeshEvent(event);
}

void EsdfServer::newPoseCallback(const Transformation& T_G_C) {
  if (clear_sphere_for_planning_) {
    esdf_integrator_->addNewRobotPosition(T_G_C.getPosition());
  }
}

void EsdfServer::esdfMapCallback(const voxblox_msgs::Layer& layer_msg) {
  bool success =
      deserializeMsgToLayer<EsdfVoxel>(layer_msg, esdf_map_->getEsdfLayerPtr());

  if (!success) {
    ROS_ERROR_THROTTLE(10, "Got an invalid ESDF map message!");
  } else {
    ROS_INFO("Map callback.");
    publishAllUpdatedEsdfVoxels();
    publishSlices();
  }
}

}  // namespace voxblox

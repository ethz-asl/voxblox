#include <voxblox_ros/conversions.h>

#include "voxblox_ros/esdf_server.h"
#include "voxblox_ros/ros_params.h"

namespace voxblox {

EsdfServer::EsdfServer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private,
                       const EsdfMap::Config& esdf_config,
                       const EsdfIntegrator::Config& esdf_integrator_config,
                       const TsdfMap::Config& tsdf_config,
                       const TsdfIntegratorBase::Config& tsdf_integrator_config)
    : TsdfServer(nh, nh_private, tsdf_config, tsdf_integrator_config),
      clear_sphere_for_planning_(false) {
  // Set up map and integrator.
  esdf_map_.reset(new EsdfMap(esdf_config));
  esdf_integrator_.reset(new EsdfIntegrator(esdf_integrator_config,
                                            tsdf_map_->getTsdfLayerPtr(),
                                            esdf_map_->getEsdfLayerPtr()));

  // Set up publisher.
  esdf_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >("esdf_pointcloud",
                                                              1, true);
  esdf_slice_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "esdf_slice", 1, true);
  esdf_map_pub_ =
      nh_private_.advertise<voxblox_msgs::Layer>("esdf_map_out", 1, false);

  // Set up subscriber.
  esdf_map_sub_ = nh_private_.subscribe("esdf_map_in", 1,
                                        &EsdfServer::esdfMapCallback, this);

  // Whether to clear each new pose as it comes in, and then set a sphere
  // around it to occupied.
  nh_private_.param("clear_sphere_for_planning", clear_sphere_for_planning_,
                    clear_sphere_for_planning_);
}

EsdfServer::EsdfServer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private)
    : TsdfServer(nh, nh_private), clear_sphere_for_planning_(false) {
  // Get config from ros params.
  EsdfIntegrator::Config esdf_integrator_config =
      getEsdfIntegratorConfigFromRosParam(nh_private_);
  EsdfMap::Config esdf_config = getEsdfMapConfigFromRosParam(nh_private_);

  // Set up map and integrator.
  esdf_map_.reset(new EsdfMap(esdf_config));
  esdf_integrator_.reset(new EsdfIntegrator(esdf_integrator_config,
                                            tsdf_map_->getTsdfLayerPtr(),
                                            esdf_map_->getEsdfLayerPtr()));

  // Set up publisher.
  esdf_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >("esdf_pointcloud",
                                                              1, true);
  esdf_slice_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "esdf_slice", 1, true);
  esdf_map_pub_ =
      nh_private_.advertise<voxblox_msgs::Layer>("esdf_map_out", 1, false);

  // Set up subscriber.
  esdf_map_sub_ = nh_private_.subscribe("esdf_map_in", 1,
                                        &EsdfServer::esdfMapCallback, this);

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

void EsdfServer::updateMesh() {
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

  TsdfServer::updateMesh();
}

void EsdfServer::updateEsdf() {
  if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    const bool clear_updated_flag_esdf = true;
    esdf_integrator_->updateFromTsdfLayer(clear_updated_flag_esdf);
  }
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

void EsdfServer::clear() {
  esdf_map_->getEsdfLayerPtr()->removeAllBlocks();
  esdf_integrator_->clear();

  TsdfServer::clear();
}

}  // namespace voxblox

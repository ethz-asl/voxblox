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
      clear_sphere_for_planning_(false),
      publish_esdf_map_(false) {
  // Set up map and integrator.
  esdf_map_.reset(new EsdfMap(esdf_config));
  esdf_integrator_.reset(new EsdfIntegrator(esdf_integrator_config,
                                            tsdf_map_->getTsdfLayerPtr(),
                                            esdf_map_->getEsdfLayerPtr()));

  setupRos();
}

EsdfServer::EsdfServer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private)
    : EsdfServer(nh, nh_private, getEsdfMapConfigFromRosParam(nh_private),
                 getEsdfIntegratorConfigFromRosParam(nh_private),
                 getTsdfMapConfigFromRosParam(nh_private),
                 getTsdfIntegratorConfigFromRosParam(nh_private)) {}

void EsdfServer::setupRos() {
  // Set up publisher.
  esdf_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >("esdf_pointcloud",
                                                              1, true);
  esdf_slice_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "esdf_slice", 1, true);
  traversable_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "traversable", 1, true);

  esdf_map_pub_ =
      nh_private_.advertise<voxblox_msgs::Layer>("esdf_map_out", 1, false);

  // Set up subscriber.
  esdf_map_sub_ = nh_private_.subscribe("esdf_map_in", 1,
                                        &EsdfServer::esdfMapCallback, this);

  // Whether to clear each new pose as it comes in, and then set a sphere
  // around it to occupied.
  nh_private_.param("clear_sphere_for_planning", clear_sphere_for_planning_,
                    clear_sphere_for_planning_);
  nh_private_.param("publish_esdf_map", publish_esdf_map_, publish_esdf_map_);

  // Special output for traversible voxels. Publishes all voxels with distance
  // at least traversibility radius.
  nh_private_.param("publish_traversable", publish_traversable_,
                    publish_traversable_);
  nh_private_.param("traversability_radius", traversability_radius_,
                    traversability_radius_);
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
    std_srvs::Empty::Request& /*request*/,      // NOLINT
    std_srvs::Empty::Response& /*response*/) {  // NOLINT
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
  if (publish_pointclouds_) {
    publishAllUpdatedEsdfVoxels();
  }
  if (publish_traversable_) {
    publishTraversable();
  }

  if (publish_esdf_map_ && esdf_map_pub_.getNumSubscribers() > 0) {
    const bool only_updated = false;
    voxblox_msgs::Layer layer_msg;
    serializeLayerAsMsg<EsdfVoxel>(esdf_map_->getEsdfLayer(), only_updated,
                                   &layer_msg);
    esdf_map_pub_.publish(layer_msg);
  }

  TsdfServer::updateMesh();
}

void EsdfServer::publishPointclouds() {
  publishAllUpdatedEsdfVoxels();
  if (publish_slices_) {
    publishSlices();
  }

  if (publish_traversable_) {
    publishTraversable();
  }

  TsdfServer::publishPointclouds();
}

void EsdfServer::publishTraversable() {
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  createFreePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(),
                                    traversability_radius_, &pointcloud);
  pointcloud.header.frame_id = world_frame_;
  traversable_pub_.publish(pointcloud);
}

void EsdfServer::publishMap(const bool reset_remote_map) {
  if (this->esdf_map_pub_.getNumSubscribers() > 0) {
    const bool only_updated = false;
    timing::Timer publish_map_timer("map/publish_esdf");
    voxblox_msgs::Layer layer_msg;
    serializeLayerAsMsg<EsdfVoxel>(this->esdf_map_->getEsdfLayer(),
                                   only_updated, &layer_msg);
    if (reset_remote_map) {
      layer_msg.action = static_cast<uint8_t>(MapDerializationAction::kReset);
    }
    this->esdf_map_pub_.publish(layer_msg);
    publish_map_timer.Stop();
  }

  TsdfServer::publishMap();
}

bool EsdfServer::saveMap(const std::string& file_path) {
  // Output TSDF map first, then ESDF.
  const bool success = TsdfServer::saveMap(file_path);

  constexpr bool kClearFile = false;
  return success &&
         io::SaveLayer(esdf_map_->getEsdfLayer(), file_path, kClearFile);
}

bool EsdfServer::loadMap(const std::string& file_path) {
  // Load in the same order: TSDF first, then ESDF.
  bool success = TsdfServer::loadMap(file_path);
  constexpr bool kMulitpleLayerSupport = true;
  success = success &&
            io::LoadBlocksFromFile(
                file_path, Layer<EsdfVoxel>::BlockMergingStrategy::kReplace,
                kMulitpleLayerSupport, esdf_map_->getEsdfLayerPtr());
  if (success) {
    LOG(INFO) << "Successfully loaded ESDF layer.";
  }

  publishPointclouds();
  return success;
}

void EsdfServer::updateEsdf() {
  if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    const bool clear_updated_flag_esdf = true;
    esdf_integrator_->updateFromTsdfLayer(clear_updated_flag_esdf);
  }
}

void EsdfServer::updateEsdfBatch(bool full_euclidean) {
  if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    if (full_euclidean) {
      esdf_integrator_->updateFromTsdfLayerBatchFullEuclidean();
    } else {
      esdf_integrator_->updateFromTsdfLayerBatch();
    }
  }
}

float EsdfServer::getEsdfMaxDistance() const {
  return esdf_integrator_->getEsdfMaxDistance();
}

void EsdfServer::setEsdfMaxDistance(float max_distance) {
  esdf_integrator_->setEsdfMaxDistance(max_distance);
}

float EsdfServer::getTraversabilityRadius() const {
  return traversability_radius_;
}

void EsdfServer::setTraversabilityRadius(float traversability_radius) {
  traversability_radius_ = traversability_radius;
}

void EsdfServer::newPoseCallback(const Transformation& T_G_C) {
  if (clear_sphere_for_planning_) {
    esdf_integrator_->addNewRobotPosition(T_G_C.getPosition());
  }

  timing::Timer block_remove_timer("remove_distant_blocks");
  esdf_map_->getEsdfLayerPtr()->removeDistantBlocks(
      T_G_C.getPosition(), max_block_distance_from_body_);
  block_remove_timer.Stop();
}

void EsdfServer::esdfMapCallback(const voxblox_msgs::Layer& layer_msg) {
  bool success =
      deserializeMsgToLayer<EsdfVoxel>(layer_msg, esdf_map_->getEsdfLayerPtr());

  if (!success) {
    ROS_ERROR_THROTTLE(10, "Got an invalid ESDF map message!");
  } else {
    ROS_INFO_ONCE("Got an ESDF map from ROS topic!");
    publishAllUpdatedEsdfVoxels();
    publishSlices();
  }
}

void EsdfServer::clear() {
  esdf_map_->getEsdfLayerPtr()->removeAllBlocks();
  esdf_integrator_->clear();
  CHECK_EQ(esdf_map_->getEsdfLayerPtr()->getNumberOfAllocatedBlocks(), 0);

  TsdfServer::clear();

  // Publish a message to reset the map to all subscribers.
  constexpr bool kResetRemoteMap = true;
  publishMap(kResetRemoteMap);
}

}  // namespace voxblox

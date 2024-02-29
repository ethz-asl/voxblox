#include "voxblox_ros/esdf_server.h"

#include "voxblox_ros/conversions.h"

namespace voxblox {

EsdfServer::EsdfServer()
    : TsdfServer(),
      clear_sphere_for_planning_(false),
      publish_esdf_map_(false),
      publish_traversable_(false),
      traversability_radius_(1.0),
      incremental_update_(true),
      num_subscribers_esdf_map_(0) {
  const EsdfMap::Config esdf_config = getEsdfMapConfigFromRosParam();
  const EsdfIntegrator::Config esdf_integrator_config =
      getEsdfIntegratorConfigFromRosParam();

  // Set up map and integrator.
  esdf_map_.reset(new EsdfMap(esdf_config));
  esdf_integrator_.reset(new EsdfIntegrator(esdf_integrator_config,
                                            tsdf_map_->getTsdfLayerPtr(),
                                            esdf_map_->getEsdfLayerPtr()));

  setupRos();
}

void EsdfServer::setupRos() {
  // Set up publisher.
  esdf_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "esdf_pointcloud", 1);
  esdf_slice_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("esdf_slice", 1);
  traversable_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("traversable", 1);

  esdf_map_pub_ =
      this->create_publisher<voxblox_msgs::msg::Layer>("esdf_map_out", 1);

  // Set up subscriber.
  esdf_map_sub_ = this->create_subscription<voxblox_msgs::msg::Layer>(
      "esdf_map_in", 1,
      std::bind(&EsdfServer::esdfMapCallback, this, std::placeholders::_1));

  // Whether to clear each new pose as it comes in, and then set a sphere
  // around it to occupied.
  clear_sphere_for_planning_ = this->declare_parameter(
      "clear_sphere_for_planning", clear_sphere_for_planning_);
  publish_esdf_map_ =
      this->declare_parameter("publish_esdf_map", publish_esdf_map_);

  // Special output for traversable voxels. Publishes all voxels with distance
  // at least traversibility radius.
  publish_traversable_ =
      this->declare_parameter("publish_traversable", publish_traversable_);
  traversability_radius_ =
      this->declare_parameter("traversability_radius", traversability_radius_);

  double update_esdf_every_n_sec = 1.0;
  update_esdf_every_n_sec = this->declare_parameter("update_esdf_every_n_sec",
                                                    update_esdf_every_n_sec);

  if (update_esdf_every_n_sec > 0.0) {
    update_esdf_timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        rclcpp::Duration::from_seconds(update_esdf_every_n_sec),
        std::bind(&EsdfServer::updateEsdfEvent, this));
  }
}

void EsdfServer::publishAllUpdatedEsdfVoxels() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(), &pointcloud);

  pointcloud.header.frame_id = world_frame_;

  sensor_msgs::msg::PointCloud2 pointcloud_message;
  pcl::toROSMsg(pointcloud, pointcloud_message);

  esdf_pointcloud_pub_->publish(pointcloud_message);
}

void EsdfServer::publishSlices() {
  TsdfServer::publishSlices();

  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  constexpr int kZAxisIndex = 2;
  createDistancePointcloudFromEsdfLayerSlice(
      esdf_map_->getEsdfLayer(), kZAxisIndex, slice_level_, &pointcloud);

  pointcloud.header.frame_id = world_frame_;

  sensor_msgs::msg::PointCloud2 pointcloud_message;
  pcl::toROSMsg(pointcloud, pointcloud_message);

  esdf_slice_pub_->publish(pointcloud_message);
}

bool EsdfServer::generateEsdfCallback(
    std_srvs::srv::Empty::Request::SharedPtr /*request*/,      // NOLINT
    std_srvs::srv::Empty::Response::SharedPtr /*response*/) {  // NOLINT
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

void EsdfServer::updateEsdfEvent() { updateEsdf(); }

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

  sensor_msgs::msg::PointCloud2 pointcloud_message;
  pcl::toROSMsg(pointcloud, pointcloud_message);

  traversable_pub_->publish(pointcloud_message);
}

void EsdfServer::publishMap(bool reset_remote_map) {
  if (!publish_esdf_map_) {
    return;
  }

  int subscribers = this->esdf_map_pub_->get_subscription_count();
  if (subscribers > 0) {
    if (num_subscribers_esdf_map_ < subscribers) {
      // Always reset the remote map and send all when a new subscriber
      // subscribes. A bit of overhead for other subscribers, but better than
      // inconsistent map states.
      reset_remote_map = true;
    }
    const bool only_updated = !reset_remote_map;
    timing::Timer publish_map_timer("map/publish_esdf");
    voxblox_msgs::msg::Layer layer_msg;
    serializeLayerAsMsg<EsdfVoxel>(this->esdf_map_->getEsdfLayer(),
                                   only_updated, &layer_msg);
    if (reset_remote_map) {
      layer_msg.action = static_cast<uint8_t>(MapDerializationAction::kReset);
    }
    this->esdf_map_pub_->publish(layer_msg);
    publish_map_timer.Stop();
  }
  num_subscribers_esdf_map_ = subscribers;
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

  constexpr bool kMultipleLayerSupport = true;
  return success &&
         io::LoadBlocksFromFile(
             file_path, Layer<EsdfVoxel>::BlockMergingStrategy::kReplace,
             kMultipleLayerSupport, esdf_map_->getEsdfLayerPtr());
}

void EsdfServer::updateEsdf() {
  if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    const bool clear_updated_flag_esdf = true;
    esdf_integrator_->updateFromTsdfLayer(clear_updated_flag_esdf);
  }
}

void EsdfServer::updateEsdfBatch(bool full_euclidean) {
  if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    esdf_integrator_->setFullEuclidean(full_euclidean);
    esdf_integrator_->updateFromTsdfLayerBatch();
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

void EsdfServer::esdfMapCallback(
    const voxblox_msgs::msg::Layer::SharedPtr layer_msg) {
  timing::Timer receive_map_timer("map/receive_esdf");

  bool success =
      deserializeMsgToLayer<EsdfVoxel>(layer_msg, esdf_map_->getEsdfLayerPtr());

  if (!success) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 10,
                          "Got an invalid ESDF map message!");
  } else {
    RCLCPP_INFO_ONCE(this->get_logger(), "Got an ESDF map from ROS topic!");
    if (publish_pointclouds_) {
      publishPointclouds();
    }
  }
}

void EsdfServer::clear() {
  esdf_map_->getEsdfLayerPtr()->removeAllBlocks();
  esdf_integrator_->clear();
  CHECK_EQ(esdf_map_->getEsdfLayerPtr()->getNumberOfAllocatedBlocks(), 0u);

  TsdfServer::clear();

  // Publish a message to reset the map to all subscribers.
  constexpr bool kResetRemoteMap = true;
  publishMap(kResetRemoteMap);
}

}  // namespace voxblox

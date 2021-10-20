#include "voxblox_ros/voxfield_server.h"

#include "voxblox_ros/conversions.h"
#include "voxblox_ros/ros_params.h"

namespace voxblox {

VoxfieldServer::VoxfieldServer(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : VoxfieldServer(nh, nh_private,
                     getEsdfMapConfigFromOccMapRosParam(nh_private),
                     getEsdfFiestaIntegratorConfigFromRosParam(nh_private),
                     getTsdfMapConfigFromOccMapRosParam(nh_private),
                     getTsdfIntegratorConfigFromRosParam(nh_private),
                     getOccupancyMapConfigFromRosParam(nh_private),
                     getOccTsdfIntegratorConfigFromRosParam(nh_private),
                     getMeshIntegratorConfigFromRosParam(nh_private)) {}

VoxfieldServer::VoxfieldServer(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    const EsdfMap::Config& esdf_config,
    const EsdfOccFiestaIntegrator::Config& esdf_integrator_config,
    const TsdfMap::Config& tsdf_config,
    const TsdfIntegratorBase::Config& tsdf_integrator_config,
    const OccupancyMap::Config& occ_config,
    const OccTsdfIntegrator::Config& occ_tsdf_integrator_config,
    const MeshIntegratorConfig& mesh_config)
    : TsdfServer(nh, nh_private, tsdf_config, tsdf_integrator_config,
                 mesh_config),
      clear_sphere_for_planning_(false),
      publish_esdf_map_(false),
      publish_traversable_(false),
      traversability_radius_(1.0),
      incremental_update_(true),
      num_subscribers_esdf_map_(0) {
  
  // Set up Occupancy map and integrator
  occupancy_map_.reset(new OccupancyMap(occ_config));
  occupancy_integrator_.reset(new OccTsdfIntegrator(
      occ_tsdf_integrator_config, tsdf_map_->getTsdfLayerPtr(),
      occupancy_map_->getOccupancyLayerPtr()));

  // Set up ESDF map and integrator.
  esdf_map_.reset(new EsdfMap(esdf_config));
  esdf_integrator_.reset(new EsdfOccFiestaIntegrator(
      esdf_integrator_config, occupancy_map_->getOccupancyLayerPtr(),
      esdf_map_->getEsdfLayerPtr()));

  // Mesh related
  mesh_layer_.reset(new MeshLayer(esdf_map_->block_size()));

  mesh_integrator_.reset(new MeshIntegrator<TsdfVoxel>(
      mesh_config, tsdf_map_->getTsdfLayerPtr(), mesh_layer_.get()));

  // main entrance
  setupRos();
}

void VoxfieldServer::setupRos() {
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
                                        &VoxfieldServer::esdfMapCallback, this);

  mesh_pub_ = nh_private_.advertise<voxblox_msgs::Mesh>("mesh", 1, true);

  // Whether to clear each new pose as it comes in, and then set a sphere
  // around it to occupied.
  nh_private_.param("clear_sphere_for_planning", clear_sphere_for_planning_,
                    clear_sphere_for_planning_);
  nh_private_.param("publish_esdf_map", publish_esdf_map_, publish_esdf_map_);

  // Special output for traversable voxels. Publishes all voxels with distance
  // at least traversibility radius.
  nh_private_.param("publish_traversable", publish_traversable_,
                    publish_traversable_);
  nh_private_.param("traversability_radius", traversability_radius_,
                    traversability_radius_);

  nh_private_.param("publish_slices", publish_slices_, publish_slices_);

  // TODO: also use it for TSDF
  nh_private_.param("esdf_slice_level", slice_level_, slice_level_);

  nh_private_.param("verbose", verbose_, verbose_);

  // Mesh output path
  nh_private_.param("mesh_filename", mesh_filename_, mesh_filename_);

  double update_esdf_every_n_sec = 1.0;  // default
  nh_private_.param("update_esdf_every_n_sec", update_esdf_every_n_sec,
                    update_esdf_every_n_sec);

  std::string color_mode("");
  nh_private_.param("color_mode", color_mode, color_mode);
  color_mode_ = getColorModeFromString(color_mode);

  //Color map for intensity pointclouds.
  std::string intensity_colormap("rainbow");
  float intensity_max_value = kDefaultMaxIntensity;
  nh_private_.param("intensity_colormap", intensity_colormap,
                    intensity_colormap);
  nh_private_.param("intensity_max_value", intensity_max_value,
                    intensity_max_value);

  //Default set in constructor.
  if (intensity_colormap == "rainbow") {
    color_map_.reset(new RainbowColorMap());
  } else if (intensity_colormap == "inverse_rainbow") {
    color_map_.reset(new InverseRainbowColorMap());
  } else if (intensity_colormap == "grayscale") {
    color_map_.reset(new GrayscaleColorMap());
  } else if (intensity_colormap == "inverse_grayscale") {
    color_map_.reset(new InverseGrayscaleColorMap());
  } else if (intensity_colormap == "ironbow") {
    color_map_.reset(new IronbowColorMap());
  } else {
    ROS_ERROR_STREAM("Invalid color map: " << intensity_colormap);
  }
  color_map_->setMaxValue(intensity_max_value);

  // Update ESDF per xx second
  if (update_esdf_every_n_sec > 0.0) {
    update_esdf_timer_ =
        nh_private_.createTimer(ros::Duration(update_esdf_every_n_sec),
                                &VoxfieldServer::updateEsdfEvent, this);
  }

  // If set, use a timer to progressively integrate the mesh.
  double update_mesh_every_n_sec = 1.0;
  nh_private_.param("update_mesh_every_n_sec", update_mesh_every_n_sec,
                    update_mesh_every_n_sec);

  // already done in Tsdf server
  // if (update_mesh_every_n_sec > 0.0) {
  //    update_mesh_timer_ =
  //       nh_private_.createTimer(ros::Duration(update_mesh_every_n_sec),
  //                               &VoxfieldServer::updateMeshEvent, this);
  // }
}

void VoxfieldServer::publishAllUpdatedEsdfVoxels() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(), &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  esdf_pointcloud_pub_.publish(pointcloud);
}

void VoxfieldServer::publishSlices() {
  TsdfServer::publishSlices();

  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  constexpr int kZAxisIndex = 2;
  createDistancePointcloudFromEsdfLayerSlice(
      esdf_map_->getEsdfLayer(), kZAxisIndex, slice_level_, &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  esdf_slice_pub_.publish(pointcloud);
}

void VoxfieldServer::updateMeshEvent(const ros::TimerEvent& /*event*/) {
  updateMesh();
}

// bool VoxfieldServer::generateEsdfCallback(
//     std_srvs::Empty::Request& /*request*/,      // NOLINT
//     std_srvs::Empty::Response& /*response*/) {  // NOLINT
//   const bool clear_esdf = true;
//   if (clear_esdf) {
//     esdf_integrator_->updateFromTsdfLayerBatch();
//   } else {
//     const bool clear_updated_flag = true;
//     esdf_integrator_->updateFromTsdfLayer(clear_updated_flag);
//   }
//   publishAllUpdatedEsdfVoxels();
//   publishSlices();
//   return true;
// }

void VoxfieldServer::updateEsdfEvent(const ros::TimerEvent& /*event*/) {
  updateOccFromTsdf();
  updateEsdfFromOcc();
  publishOccupancyOccupiedNodes();
  // publishPointclouds();
  if (publish_slices_) publishSlices();
}

void VoxfieldServer::updateMesh() {
  if (verbose_) {
    ROS_INFO("Updating mesh.");
  }

  timing::Timer generate_mesh_timer("mesh/update");
  constexpr bool only_mesh_updated_blocks = true;
  constexpr bool clear_updated_flag = true;
  mesh_integrator_->generateMesh(only_mesh_updated_blocks, clear_updated_flag);
  generate_mesh_timer.Stop();

  timing::Timer publish_mesh_timer("mesh/publish");

  voxblox_msgs::Mesh mesh_msg;
  generateVoxbloxMeshMsg(mesh_layer_, color_mode_, &mesh_msg);
  mesh_msg.header.frame_id = world_frame_;
  mesh_pub_.publish(mesh_msg);

  if (cache_mesh_) {
    cached_mesh_msg_ = mesh_msg;
  }

  publish_mesh_timer.Stop();

  if (publish_pointclouds_ && !publish_pointclouds_on_update_) {
    publishPointclouds();
  }
}

bool VoxfieldServer::generateMesh() {
  timing::Timer generate_mesh_timer("mesh/generate");
  const bool clear_mesh = true;
  if (clear_mesh) {
    constexpr bool only_mesh_updated_blocks = false;
    constexpr bool clear_updated_flag = true;
    mesh_integrator_->generateMesh(only_mesh_updated_blocks,
                                   clear_updated_flag);
  } else {
    constexpr bool only_mesh_updated_blocks = true;
    constexpr bool clear_updated_flag = true;
    mesh_integrator_->generateMesh(only_mesh_updated_blocks,
                                   clear_updated_flag);
  }
  generate_mesh_timer.Stop();

  timing::Timer publish_mesh_timer("mesh/publish");
  voxblox_msgs::Mesh mesh_msg;

  generateVoxbloxMeshMsg(mesh_layer_, color_mode_, &mesh_msg);
  mesh_msg.header.frame_id = world_frame_;
  mesh_pub_.publish(mesh_msg);

  publish_mesh_timer.Stop();

  if (!mesh_filename_.empty()) {
    timing::Timer output_mesh_timer("mesh/output");
    const bool success = outputMeshLayerAsPly(mesh_filename_, *mesh_layer_);
    output_mesh_timer.Stop();
    if (success) {
      ROS_INFO("Output file as PLY: %s", mesh_filename_.c_str());
    } else {
      ROS_INFO("Failed to output mesh as PLY: %s", mesh_filename_.c_str());
    }
  }

  ROS_INFO_STREAM("Mesh Timings: " << std::endl << timing::Timing::Print());
  return true;
}

void VoxfieldServer::publishOccupancyOccupiedNodes() {
  // Create a pointcloud with elevation = intensity.
  visualization_msgs::MarkerArray marker_array;
  createOccupancyBlocksFromOccupancyLayer(occupancy_map_->getOccupancyLayer(),
                                          world_frame_, &marker_array);
  occupancy_marker_pub_.publish(marker_array);
}

void VoxfieldServer::publishPointclouds() {
  publishAllUpdatedEsdfVoxels();
  if (publish_slices_) {
    publishSlices();
  }

  if (publish_traversable_) {
    publishTraversable();
  }

  // TsdfServer::publishPointclouds(); // TODO:
}

void VoxfieldServer::publishTraversable() {
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  createFreePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(),
                                    traversability_radius_, &pointcloud);
  pointcloud.header.frame_id = world_frame_;
  traversable_pub_.publish(pointcloud);
}

void VoxfieldServer::publishMap(bool reset_remote_map) {
  if (!publish_esdf_map_) {
    return;
  }

  int subscribers = this->esdf_map_pub_.getNumSubscribers();
  if (subscribers > 0) {
    if (num_subscribers_esdf_map_ < subscribers) {
      // Always reset the remote map and send all when a new subscriber
      // subscribes. A bit of overhead for other subscribers, but better than
      // inconsistent map states.
      reset_remote_map = true;
    }
    const bool only_updated = !reset_remote_map;
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
  num_subscribers_esdf_map_ = subscribers;
  TsdfServer::publishMap();
}

bool VoxfieldServer::saveMap(const std::string& file_path) {
  // Output TSDF map first, then ESDF.
  const bool success = TsdfServer::saveMap(file_path);

  constexpr bool kClearFile = false;
  return success &&
         io::SaveLayer(esdf_map_->getEsdfLayer(), file_path, kClearFile);
}

bool VoxfieldServer::loadMap(const std::string& file_path) {
  // Load in the same order: TSDF first, then ESDF.
  bool success = TsdfServer::loadMap(file_path);

  constexpr bool kMultipleLayerSupport = true;
  return success &&
         io::LoadBlocksFromFile(
             file_path, Layer<EsdfVoxel>::BlockMergingStrategy::kReplace,
             kMultipleLayerSupport, esdf_map_->getEsdfLayerPtr());
}

// Incrementally update Esdf from occupancy map via FIESTA
void VoxfieldServer::updateEsdfFromOcc() {
  if (occupancy_map_->getOccupancyLayer().getNumberOfAllocatedBlocks() > 0) {
    GlobalIndexList insert_list = occupancy_integrator_->getInsertList();
    GlobalIndexList delete_list = occupancy_integrator_->getDeleteList();
    occupancy_integrator_->clearList();
    esdf_integrator_->loadInsertList(insert_list);
    esdf_integrator_->loadDeleteList(delete_list);
    if (insert_list.size() + delete_list.size() > 0) {
      if (verbose_)
        ROS_INFO_STREAM("Insert [" << insert_list.size() << "] and delete ["
                                   << delete_list.size()
                                   << "] occupied voxels.");

      const bool clear_updated_flag_esdf = true;
      ros::WallTime start = ros::WallTime::now();

      // set update state to 0 after the processing
      esdf_integrator_->updateFromOccLayer(clear_updated_flag_esdf);

      ros::WallTime end = ros::WallTime::now();

      if (verbose_) {
        ROS_INFO("Finished ESDF integrating in [%f] seconds, have [%lu] blocks(memory : %f MB).", 
                (end - start).toSec(),
                esdf_map_->getEsdfLayer().getNumberOfAllocatedBlocks(),
                esdf_map_->getEsdfLayer().getMemorySize() / 1024.0 / 1024.0); //NOLINT
      }
    }
    esdf_integrator_->clear();
  }
}

// TODO: Processing in batch
// void VoxfieldServer::updateEsdfBatch(bool full_euclidean) {
//   if (occupancy_map_->getOccupancyLayer().getNumberOfAllocatedBlocks() > 0) {
//     esdf_integrator_->setFullEuclidean(full_euclidean);
//     esdf_integrator_->updateFromTsdfLayerBatch();
//   }
// }

// incrementally
void VoxfieldServer::updateOccFromTsdf() {
  if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    const bool clear_updated_flag_esdf = true;
  
    // set update state to 0 after the processing
    occupancy_integrator_->updateFromTsdfLayer(clear_updated_flag_esdf);
  }
}

float VoxfieldServer::getEsdfMaxDistance() const {
  return esdf_integrator_->getEsdfMaxDistance();
}

void VoxfieldServer::setEsdfMaxDistance(float max_distance) {
  esdf_integrator_->setEsdfMaxDistance(max_distance);
}

float VoxfieldServer::getTraversabilityRadius() const {
  return traversability_radius_;
}

void VoxfieldServer::setTraversabilityRadius(float traversability_radius) {
  traversability_radius_ = traversability_radius;
}

// void VoxfieldServer::newPoseCallback(const Transformation& T_G_C) {
//   if (clear_sphere_for_planning_) {
//     esdf_integrator_->addNewRobotPosition(T_G_C.getPosition());
//   }

//   timing::Timer block_remove_timer("remove_distant_blocks");
//   esdf_map_->getEsdfLayerPtr()->removeDistantBlocks(
//       T_G_C.getPosition(), max_block_distance_from_body_);
//   block_remove_timer.Stop();
// }

void VoxfieldServer::esdfMapCallback(const voxblox_msgs::Layer& layer_msg) {
  timing::Timer receive_map_timer("map/receive_esdf");

  bool success =
      deserializeMsgToLayer<EsdfVoxel>(layer_msg, esdf_map_->getEsdfLayerPtr());

  if (!success) {
    ROS_ERROR_THROTTLE(10, "Got an invalid ESDF map message!");
  } else {
    ROS_INFO_ONCE("Got an ESDF map from ROS topic!");
    if (publish_pointclouds_) {
      publishPointclouds();
    }
  }
}

void VoxfieldServer::clear() {
  esdf_map_->getEsdfLayerPtr()->removeAllBlocks();
  esdf_integrator_->clear();
  CHECK_EQ(esdf_map_->getEsdfLayerPtr()->getNumberOfAllocatedBlocks(), 0u);

  TsdfServer::clear();

  // Publish a message to reset the map to all subscribers.
  constexpr bool kResetRemoteMap = true;
  publishMap(kResetRemoteMap);
}

}  // namespace voxblox

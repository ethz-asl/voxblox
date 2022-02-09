#include "voxblox_ros/voxfield_server.h"

#include "voxblox_ros/conversions.h"
#include "voxblox_ros/ros_params.h"

#include <pcl/kdtree/kdtree_flann.h>  //py: added

namespace voxblox {

VoxfieldServer::VoxfieldServer(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : VoxfieldServer(nh, nh_private, getEsdfMapConfigFromRosParam(nh_private),
                     getEsdfVoxfieldIntegratorConfigFromRosParam(nh_private),
                     getTsdfMapConfigFromRosParam(nh_private),
                     getTsdfIntegratorConfigFromRosParam(nh_private),
                     getMeshIntegratorConfigFromRosParam(nh_private)) {}

VoxfieldServer::VoxfieldServer(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    const EsdfMap::Config& esdf_config,
    const EsdfVoxfieldIntegrator::Config& esdf_integrator_config,
    const TsdfMap::Config& tsdf_config,
    const TsdfIntegratorBase::Config& tsdf_integrator_config,
    const MeshIntegratorConfig& mesh_config)
    : TsdfServer(nh, nh_private, tsdf_config, tsdf_integrator_config,
                 mesh_config),
      clear_sphere_for_planning_(false),
      publish_esdf_map_(false),
      publish_traversable_(false),
      traversability_radius_(1.0),
      incremental_update_(true),
      num_subscribers_esdf_map_(0) {
  // py: add
  // Set up Occupancy map and integrator
  OccupancyMap::Config occ_config;
  occ_config.occupancy_voxel_size = esdf_config.esdf_voxel_size;
  occ_config.occupancy_voxels_per_side = esdf_config.esdf_voxels_per_side;
  OccTsdfIntegrator::Config occ_tsdf_integrator_config;
  occupancy_map_.reset(new OccupancyMap(occ_config));
  occupancy_integrator_.reset(new OccTsdfIntegrator(
      occ_tsdf_integrator_config, tsdf_map_->getTsdfLayerPtr(),
      occupancy_map_->getOccupancyLayerPtr()));

  // Set up map and integrator.
  esdf_map_.reset(new EsdfMap(esdf_config));
  esdf_integrator_.reset(new EsdfVoxfieldIntegrator(
      esdf_integrator_config, tsdf_map_->getTsdfLayerPtr(),
      esdf_map_->getEsdfLayerPtr()));

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
  // py: added
  esdf_error_slice_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
          "esdf_error_slice", 1, true);

  esdf_map_pub_ =
      nh_private_.advertise<voxblox_msgs::Layer>("esdf_map_out", 1, false);

  // Set up subscriber.
  esdf_map_sub_ = nh_private_.subscribe("esdf_map_in", 1,
                                        &VoxfieldServer::esdfMapCallback, this);

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

  double update_esdf_every_n_sec = 1.0;
  nh_private_.param("update_esdf_every_n_sec", update_esdf_every_n_sec,
                    update_esdf_every_n_sec);

  save_esdf_map_srv_ = nh_private_.advertiseService(
      "save_esdf_map", &VoxfieldServer::saveEsdfMapCallback, this);

  if (update_esdf_every_n_sec > 0.0) {
    update_esdf_timer_ =
        nh_private_.createTimer(ros::Duration(update_esdf_every_n_sec),
                                &VoxfieldServer::updateEsdfEvent, this);
  }

  // py: added
  bool eval_esdf_on = false;
  nh_private_.param("eval_esdf_on", eval_esdf_on, eval_esdf_on);

  double eval_esdf_every_n_sec = 100.0;  // default
  nh_private_.param("eval_esdf_every_n_sec", eval_esdf_every_n_sec,
                    eval_esdf_every_n_sec);

  esdf_ready_ = false;

  // Evaluate ESDF accuracy per xx second
  if (eval_esdf_every_n_sec > 0.0 && eval_esdf_on) {
    eval_esdf_timer_ =
        nh_private_.createTimer(ros::Duration(eval_esdf_every_n_sec),
                                &VoxfieldServer::evalEsdfEvent, this);
  }
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
  updateEsdf();
  if (publish_slices_) publishSlices();
}

bool VoxfieldServer::saveEsdfMapCallback(
    voxblox_msgs::FilePath::Request& request, voxblox_msgs::FilePath::Response&
    /*response*/) {  // NOLINT
  return saveMap(request.file_path);
}

void VoxfieldServer::publishPointclouds() {
  publishAllUpdatedEsdfVoxels();
  if (publish_slices_) {
    publishSlices();
  }

  if (publish_traversable_) {
    publishTraversable();
  }

  TsdfServer::publishPointclouds();
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
  // const bool success = TsdfServer::saveMap(file_path);
  bool success = true;
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

void VoxfieldServer::updateEsdf() {
  if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    const bool clear_updated_flag_esdf = true;
    esdf_integrator_->updateFromTsdfLayer(clear_updated_flag_esdf);
    esdf_ready_ = true;  // py: added
  }
}

// void VoxfieldServer::updateEsdfBatch(bool full_euclidean) {
//   if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
//     esdf_integrator_->setFullEuclidean(full_euclidean);
//     esdf_integrator_->updateFromTsdfLayerBatch();
//   }
// }

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

// py: added
// incrementally update occupancy map from the updated TSDF map
void VoxfieldServer::updateOccFromTsdf() {
  if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    const bool clear_updated_flag_tsdf = true;
    const bool in_batch = true;

    // set update state to 0 after the processing
    occupancy_integrator_->updateFromTsdfLayer(clear_updated_flag_tsdf,
                                               in_batch);
  }
}

// py: added
void VoxfieldServer::evalEsdfEvent(const ros::TimerEvent& /*event*/) {
  if (esdf_ready_) {
    updateOccFromTsdf();
    evalEsdfRefOcc();
    visualizeEsdfError();
    publishOccupancyOccupiedNodes();
  }
}

void VoxfieldServer::publishOccupancyOccupiedNodes() {
  // Create a pointcloud with elevation = intensity.
  visualization_msgs::MarkerArray marker_array;
  createOccupancyBlocksFromOccupancyLayer(occupancy_map_->getOccupancyLayer(),
                                          world_frame_, &marker_array);
  occupancy_marker_pub_.publish(marker_array);
}

// py: added
// Evaluate the accuracy of ESDF mapping, referenced to current occupancy map
// add it later to a seperate class
void VoxfieldServer::evalEsdfRefOcc() {
  timing::Timer eval_esdf_timer("eval/esdf");

  pcl::PointCloud<pcl::PointXYZ>::Ptr occ_ptcloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  BlockIndexList occ_blocks;
  occupancy_map_->getOccupancyLayer().getAllAllocatedBlocks(&occ_blocks);
  int voxels_per_side = occupancy_map_->getOccupancyLayer().voxels_per_side();
  float voxel_size = occupancy_map_->getOccupancyLayer().voxel_size();
  float error_trunc_limit = voxel_size * 2.0;

  for (const BlockIndex& block_index : occ_blocks) {
    Block<OccupancyVoxel>::ConstPtr occ_block =
        occupancy_map_->getOccupancyLayer().getBlockPtrByIndex(block_index);
    if (!occ_block) {
      continue;
    }
    const size_t num_voxels_per_block = occ_block->num_voxels();
    for (size_t lin_index = 0u; lin_index < num_voxels_per_block; ++lin_index) {
      const OccupancyVoxel& occ_voxel =
          occ_block->getVoxelByLinearIndex(lin_index);
      if (!occ_voxel.observed || occ_voxel.probability_log < 0.7) continue;

      VoxelIndex voxel_index =
          occ_block->computeVoxelIndexFromLinearIndex(lin_index);
      GlobalIndex global_index = getGlobalVoxelIndexFromBlockAndVoxelIndex(
          block_index, voxel_index, voxels_per_side);

      Point point = getCenterPointFromGridIndex(global_index, voxel_size);

      pcl::PointXYZ pt(point(0), point(1), point(2));
      occ_ptcloud->points.push_back(pt);
    }
  }
  // build the kd tree of the occupied grid point cloud
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  std::cout << "Begin to evaluate ESDF mapping accuracy ";
  kdtree.setInputCloud(occ_ptcloud);

  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);

  BlockIndexList esdf_blocks;
  esdf_map_->getEsdfLayer().getAllAllocatedBlocks(&esdf_blocks);

  double mse = 0.0, mae = 0.0;
  uint64_t total_evaluated_voxels = 0;

  for (const BlockIndex& block_index : esdf_blocks) {
    Block<EsdfVoxel>::ConstPtr esdf_block =
        esdf_map_->getEsdfLayer().getBlockPtrByIndex(block_index);
    if (!esdf_block) {
      continue;
    }

    const size_t num_voxels_per_block = esdf_block->num_voxels();
    for (size_t lin_index = 0u; lin_index < num_voxels_per_block; ++lin_index) {
      const EsdfVoxel& esdf_voxel =
          esdf_block->getVoxelByLinearIndex(lin_index);
      if (!esdf_voxel.observed) continue;

      VoxelIndex voxel_index =
          esdf_block->computeVoxelIndexFromLinearIndex(lin_index);
      GlobalIndex global_index = getGlobalVoxelIndexFromBlockAndVoxelIndex(
          block_index, voxel_index, voxels_per_side);

      Point point = getCenterPointFromGridIndex(global_index, voxel_size);

      kdtree.nearestKSearch(pcl::PointXYZ(point(0), point(1), point(2)), 1,
                            pointIdxNKNSearch, pointNKNSquaredDistance);
      float cur_gt_dist = std::sqrt(pointNKNSquaredDistance[0]);
      float cur_est_dist = std::abs(esdf_voxel.distance);
      float cur_error_dist = cur_est_dist - cur_gt_dist;
      cur_error_dist = std::min(error_trunc_limit, std::max(-error_trunc_limit, cur_error_dist)); // truncated error
      mse += (cur_error_dist * cur_error_dist);
      mae += std::abs(cur_error_dist);

      // Clamped with the error limit for visualization
      // esdf_integrator_->assignError(
      //     global_index, std::max(-error_vis_limit,
      //                            std::min(error_vis_limit, cur_error_dist)));

      esdf_integrator_->assignError(global_index, cur_error_dist);

      total_evaluated_voxels++;
    }
  }

  double rms = sqrt(mse / total_evaluated_voxels);
  mae /= total_evaluated_voxels;

  std::cout << "Finished evaluating ESDF map.\n"
            << "\nRMSE:           " << rms << "\nMAE:            " << mae
            << "\nTotal evaluated:     " << total_evaluated_voxels << "\n";

  eval_esdf_timer.Stop();

  ROS_INFO_STREAM("Timings: " << std::endl << timing::Timing::Print());

  occ_ptcloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
}

// py: added
void VoxfieldServer::visualizeEsdfError() {
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;

  constexpr int kZAxisIndex = 2;
  createErrorPointcloudFromEsdfLayerSlice(
      esdf_map_->getEsdfLayer(), kZAxisIndex, slice_level_, &pointcloud);

  pointcloud.header.frame_id = world_frame_;

  esdf_error_slice_pub_.publish(pointcloud);
}

}  // namespace voxblox

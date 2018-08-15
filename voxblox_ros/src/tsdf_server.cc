#include "voxblox_ros/tsdf_server.h"

#include "voxblox_ros/ros_params.h"

namespace voxblox {

void TsdfServer::getServerConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  // Before subscribing, determine minimum time between messages.
  // 0 by default.
  double min_time_between_msgs_sec = 0.0;
  nh_private.param("min_time_between_msgs_sec", min_time_between_msgs_sec,
                   min_time_between_msgs_sec);
  min_time_between_msgs_.fromSec(min_time_between_msgs_sec);

  nh_private.param("max_block_distance_from_body",
                   max_block_distance_from_body_,
                   max_block_distance_from_body_);
  nh_private.param("slice_level", slice_level_, slice_level_);
  nh_private.param("world_frame", world_frame_, world_frame_);
  nh_private.param("publish_tsdf_info", publish_tsdf_info_, publish_tsdf_info_);
  nh_private.param("publish_slices", publish_slices_, publish_slices_);
  nh_private.param("publish_pointclouds", publish_pointclouds_,
                   publish_pointclouds_);

  nh_private.param("use_freespace_pointcloud", use_freespace_pointcloud_,
                   use_freespace_pointcloud_);

  nh_private.param("pointcloud_queue_size", pointcloud_queue_size_,
                   pointcloud_queue_size_);

  nh_private.param("verbose", verbose_, verbose_);

  // Mesh settings.
  nh_private.param("mesh_filename", mesh_filename_, mesh_filename_);
  std::string color_mode("color");
  nh_private.param("color_mode", color_mode, color_mode);
  if (color_mode == "color" || color_mode == "colors") {
    color_mode_ = ColorMode::kColor;
  } else if (color_mode == "height") {
    color_mode_ = ColorMode::kHeight;
  } else if (color_mode == "normals") {
    color_mode_ = ColorMode::kNormals;
  } else if (color_mode == "lambert") {
    color_mode_ = ColorMode::kLambert;
  } else if (color_mode == "lambert_color") {
    color_mode_ = ColorMode::kLambertColor;
  } else {  // Default case is gray.
    color_mode_ = ColorMode::kGray;
  }
}

TsdfServer::TsdfServer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private)
    : TsdfServer(nh, nh_private, getTsdfMapConfigFromRosParam(nh_private),
                 getTsdfIntegratorConfigFromRosParam(nh_private)) {}

TsdfServer::TsdfServer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private,
                       const TsdfMap::Config& config,
                       const TsdfIntegratorBase::Config& integrator_config)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(true),
      world_frame_("world"),
      max_block_distance_from_body_(std::numeric_limits<FloatingPoint>::max()),
      slice_level_(0.5),
      use_freespace_pointcloud_(false),
      publish_tsdf_info_(false),
      publish_slices_(false),
      publish_pointclouds_(false),
      publish_tsdf_map_(false),
      cache_mesh_(false),
      pointcloud_queue_size_(1),
      transformer_(nh, nh_private) {
  getServerConfigFromRosParam(nh_private);

  // Advertise topics.
  mesh_pub_ = nh_private_.advertise<voxblox_msgs::Mesh>("mesh", 1, true);
  surface_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
          "surface_pointcloud", 1, true);
  tsdf_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >("tsdf_pointcloud",
                                                              1, true);
  occupancy_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("occupied_nodes",
                                                             1, true);
  tsdf_slice_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "tsdf_slice", 1, true);

  nh_private_.param("pointcloud_queue_size", pointcloud_queue_size_,
                    pointcloud_queue_size_);
  pointcloud_sub_ = nh_.subscribe("pointcloud", pointcloud_queue_size_,
                                  &TsdfServer::insertPointcloud, this);

  // Publishing/subscribing to a layer from another node (when using this as
  // a library, for example within a planner).
  tsdf_map_pub_ =
      nh_private_.advertise<voxblox_msgs::Layer>("tsdf_map_out", 1, false);
  tsdf_map_sub_ = nh_private_.subscribe("tsdf_map_in", 1,
                                        &TsdfServer::tsdfMapCallback, this);
  nh_private_.param("publish_tsdf_map", publish_tsdf_map_, publish_tsdf_map_);

  if (use_freespace_pointcloud_) {
    // points that are not inside an object, but may also not be on a surface.
    // These will only be used to mark freespace beyond the truncation distance.
    freespace_pointcloud_sub_ =
        nh_.subscribe("freespace_pointcloud", pointcloud_queue_size_,
                      &TsdfServer::insertFreespacePointcloud, this);
  }

  // Initialize TSDF Map and integrator.
  tsdf_map_.reset(new TsdfMap(config));

  std::string method("merged");
  nh_private_.param("method", method, method);
  if (method.compare("simple") == 0) {
    tsdf_integrator_.reset(new SimpleTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else if (method.compare("merged") == 0) {
    tsdf_integrator_.reset(new MergedTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else if (method.compare("fast") == 0) {
    tsdf_integrator_.reset(new FastTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else {
    tsdf_integrator_.reset(new SimpleTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  }

  MeshIntegratorConfig mesh_config;
  nh_private_.param("mesh_min_weight", mesh_config.min_weight,
                    mesh_config.min_weight);

  mesh_layer_.reset(new MeshLayer(tsdf_map_->block_size()));

  mesh_integrator_.reset(new MeshIntegrator<TsdfVoxel>(
      mesh_config, tsdf_map_->getTsdfLayerPtr(), mesh_layer_.get()));

  // Advertise services.
  generate_mesh_srv_ = nh_private_.advertiseService(
      "generate_mesh", &TsdfServer::generateMeshCallback, this);
  clear_map_srv_ = nh_private_.advertiseService(
      "clear_map", &TsdfServer::clearMapCallback, this);
  save_map_srv_ = nh_private_.advertiseService(
      "save_map", &TsdfServer::saveMapCallback, this);
  load_map_srv_ = nh_private_.advertiseService(
      "load_map", &TsdfServer::loadMapCallback, this);
  publish_pointclouds_srv_ = nh_private_.advertiseService(
      "publish_pointclouds", &TsdfServer::publishPointcloudsCallback, this);
  publish_tsdf_map_srv_ = nh_private_.advertiseService(
      "publish_map", &TsdfServer::publishTsdfMapCallback, this);

  // If set, use a timer to progressively integrate the mesh.
  double update_mesh_every_n_sec = 0.0;
  nh_private_.param("update_mesh_every_n_sec", update_mesh_every_n_sec,
                    update_mesh_every_n_sec);

  if (update_mesh_every_n_sec > 0.0) {
    update_mesh_timer_ =
        nh_private_.createTimer(ros::Duration(update_mesh_every_n_sec),
                                &TsdfServer::updateMeshEvent, this);
  }
}

void TsdfServer::processPointCloudMessageAndInsert(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg,
    const Transformation& T_G_C, const bool is_freespace_pointcloud) {
  // Convert the PCL pointcloud into our awesome format.
  // TODO(helenol): improve...
  // Horrible hack fix to fix color parsing colors in PCL.
  for (size_t d = 0; d < pointcloud_msg->fields.size(); ++d) {
    if (pointcloud_msg->fields[d].name == std::string("rgb")) {
      pointcloud_msg->fields[d].datatype = sensor_msgs::PointField::FLOAT32;
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl;
  // pointcloud_pcl is modified below:
  pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);

  timing::Timer ptcloud_timer("ptcloud_preprocess");

  Pointcloud points_C;
  Colors colors;
  points_C.reserve(pointcloud_pcl.size());
  colors.reserve(pointcloud_pcl.size());
  for (size_t i = 0; i < pointcloud_pcl.points.size(); ++i) {
    if (!std::isfinite(pointcloud_pcl.points[i].x) ||
        !std::isfinite(pointcloud_pcl.points[i].y) ||
        !std::isfinite(pointcloud_pcl.points[i].z)) {
      continue;
    }

    points_C.push_back(Point(pointcloud_pcl.points[i].x,
                             pointcloud_pcl.points[i].y,
                             pointcloud_pcl.points[i].z));
    colors.push_back(
        Color(pointcloud_pcl.points[i].r, pointcloud_pcl.points[i].g,
              pointcloud_pcl.points[i].b, pointcloud_pcl.points[i].a));
  }

  ptcloud_timer.Stop();

  if (verbose_) {
    ROS_INFO("Integrating a pointcloud with %lu points.", points_C.size());
  }
  ros::WallTime start = ros::WallTime::now();
  integratePointcloud(T_G_C, points_C, colors, is_freespace_pointcloud);
  ros::WallTime end = ros::WallTime::now();
  if (verbose_) {
    ROS_INFO("Finished integrating in %f seconds, have %lu blocks.",
             (end - start).toSec(),
             tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks());
  }

  timing::Timer block_remove_timer("remove_distant_blocks");
  tsdf_map_->getTsdfLayerPtr()->removeDistantBlocks(
      T_G_C.getPosition(), max_block_distance_from_body_);
  mesh_layer_->clearDistantMesh(T_G_C.getPosition(),
                                max_block_distance_from_body_);
  block_remove_timer.Stop();

  // Callback for inheriting classes.
  newPoseCallback(T_G_C);
}

void TsdfServer::insertPointcloud(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg) {
  // Figure out if we should insert this.
  static ros::Time last_msg_time;
  if (pointcloud_msg->header.stamp - last_msg_time < min_time_between_msgs_) {
    return;
  }
  last_msg_time = pointcloud_msg->header.stamp;

  constexpr bool is_freespace_pointcloud = false;
  Transformation T_G_C;
  if (transformer_.lookupTransform(pointcloud_msg->header.frame_id,
                                   world_frame_, pointcloud_msg->header.stamp,
                                   &T_G_C)) {
    processPointCloudMessageAndInsert(pointcloud_msg, T_G_C,
                                      is_freespace_pointcloud);
  } else {
    ROS_WARN_THROTTLE(60, "Couldn't look up pose for incoming pointcloud.");
  }

  if (publish_tsdf_info_) {
    publishAllUpdatedTsdfVoxels();
    publishTsdfSurfacePoints();
    publishTsdfOccupiedNodes();
  }
  if (publish_slices_) {
    publishSlices();
  }

  if (verbose_) {
    ROS_INFO_STREAM("Timings: " << std::endl << timing::Timing::Print());
    ROS_INFO_STREAM(
        "Layer memory: " << tsdf_map_->getTsdfLayer().getMemorySize());
  }
}

void TsdfServer::insertFreespacePointcloud(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg) {
  // Figure out if we should insert this.
  static ros::Time last_msg_time;
  if (pointcloud_msg->header.stamp - last_msg_time < min_time_between_msgs_) {
    return;
  }
  last_msg_time = pointcloud_msg->header.stamp;

  constexpr bool is_freespace_pointcloud = true;
  Transformation T_G_C;
  if (transformer_.lookupTransform(pointcloud_msg->header.frame_id,
                                   world_frame_, pointcloud_msg->header.stamp,
                                   &T_G_C)) {
    processPointCloudMessageAndInsert(pointcloud_msg, T_G_C,
                                      is_freespace_pointcloud);
  } else {
    ROS_WARN_THROTTLE(60, "Couldn't look up pose for incoming pointcloud.");
  }
}

void TsdfServer::integratePointcloud(const Transformation& T_G_C,
                                     const Pointcloud& ptcloud_C,
                                     const Colors& colors,
                                     const bool is_freespace_pointcloud) {
  tsdf_integrator_->integratePointCloud(T_G_C, ptcloud_C, colors,
                                        is_freespace_pointcloud);
}

void TsdfServer::publishAllUpdatedTsdfVoxels() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromTsdfLayer(tsdf_map_->getTsdfLayer(), &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  tsdf_pointcloud_pub_.publish(pointcloud);
}

void TsdfServer::publishTsdfSurfacePoints() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  const float surface_distance_thresh =
      tsdf_map_->getTsdfLayer().voxel_size() * 0.75;
  createSurfacePointcloudFromTsdfLayer(tsdf_map_->getTsdfLayer(),
                                       surface_distance_thresh, &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  surface_pointcloud_pub_.publish(pointcloud);
}

void TsdfServer::publishTsdfOccupiedNodes() {
  // Create a pointcloud with distance = intensity.
  visualization_msgs::MarkerArray marker_array;
  createOccupancyBlocksFromTsdfLayer(tsdf_map_->getTsdfLayer(), world_frame_,
                                     &marker_array);
  occupancy_marker_pub_.publish(marker_array);
}

void TsdfServer::publishSlices() {
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromTsdfLayerSlice(tsdf_map_->getTsdfLayer(), 2,
                                             slice_level_, &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  tsdf_slice_pub_.publish(pointcloud);
}

void TsdfServer::publishMap(const bool reset_remote_map) {
  if (this->tsdf_map_pub_.getNumSubscribers() > 0) {
    const bool only_updated = false;
    timing::Timer publish_map_timer("map/publish_tsdf");
    voxblox_msgs::Layer layer_msg;
    serializeLayerAsMsg<TsdfVoxel>(this->tsdf_map_->getTsdfLayer(),
                                   only_updated, &layer_msg);
    if (reset_remote_map) {
      layer_msg.action = static_cast<uint8_t>(MapDerializationAction::kReset);
    }
    this->tsdf_map_pub_.publish(layer_msg);
    publish_map_timer.Stop();
  }
}

void TsdfServer::publishPointclouds() {
  // Combined function to publish all possible pointcloud messages -- surface
  // pointclouds, updated points, and occupied points.
  publishAllUpdatedTsdfVoxels();
  publishTsdfSurfacePoints();
  publishTsdfOccupiedNodes();
}

void TsdfServer::updateMesh() {
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

  if (publish_pointclouds_) {
    publishPointclouds();
  }

  if (publish_tsdf_map_ && tsdf_map_pub_.getNumSubscribers() > 0u) {
    constexpr bool only_publish_updated_blocks = false;
    voxblox_msgs::Layer layer_msg;
    serializeLayerAsMsg<TsdfVoxel>(tsdf_map_->getTsdfLayer(),
                                   only_publish_updated_blocks, &layer_msg);
    tsdf_map_pub_.publish(layer_msg);
  }
}

bool TsdfServer::generateMesh() {
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

bool TsdfServer::saveMap(const std::string& file_path) {
  // Inheriting classes should add saving other layers to this function.
  return io::SaveLayer(tsdf_map_->getTsdfLayer(), file_path);
}

bool TsdfServer::loadMap(const std::string& file_path) {
  // Inheriting classes should add other layers to load, as this will only load
  // the TSDF layer.
  constexpr bool kMulitpleLayerSupport = true;
  bool success = io::LoadBlocksFromFile(
      file_path, Layer<TsdfVoxel>::BlockMergingStrategy::kReplace,
      kMulitpleLayerSupport, tsdf_map_->getTsdfLayerPtr());
  if (success) {
    LOG(INFO) << "Successfully loaded TSDF layer.";
  }
  return success;
}

bool TsdfServer::clearMapCallback(
    std_srvs::Empty::Request& /*request*/,
    std_srvs::Empty::Response& /*response*/) {  // NOLINT
  clear();
  return true;
}

bool TsdfServer::generateMeshCallback(
    std_srvs::Empty::Request& /*request*/,
    std_srvs::Empty::Response& /*response*/) {  // NOLINT
  return generateMesh();
}

bool TsdfServer::saveMapCallback(
    voxblox_msgs::FilePath::Request& request,
    voxblox_msgs::FilePath::Response& /*response*/) {  // NOLINT
  return saveMap(request.file_path);
}

bool TsdfServer::loadMapCallback(
    voxblox_msgs::FilePath::Request& request,
    voxblox_msgs::FilePath::Response& /*response*/) {  // NOLINT
  bool success = loadMap(request.file_path);
  return success;
}

bool TsdfServer::publishPointcloudsCallback(
    std_srvs::Empty::Request& /*request*/,
    std_srvs::Empty::Response& /*response*/) {  // NOLINT
  publishPointclouds();
  return true;
}

bool TsdfServer::publishTsdfMapCallback(
    std_srvs::Empty::Request& /*request*/,
    std_srvs::Empty::Response& /*response*/) {  // NOLINT
  publishMap();
  return true;
}

void TsdfServer::updateMeshEvent(const ros::TimerEvent& /*event*/) {
  updateMesh();
}

void TsdfServer::clear() {
  tsdf_map_->getTsdfLayerPtr()->removeAllBlocks();
  mesh_layer_->clear();

  // Publish a message to reset the map to all subscribers.
  constexpr bool kResetRemoteMap = true;
  publishMap(kResetRemoteMap);
}

void TsdfServer::tsdfMapCallback(const voxblox_msgs::Layer& layer_msg) {
  bool success =
      deserializeMsgToLayer<TsdfVoxel>(layer_msg, tsdf_map_->getTsdfLayerPtr());

  if (!success) {
    ROS_ERROR_THROTTLE(10, "Got an invalid TSDF map message!");
  } else {
    ROS_INFO_ONCE("Got an TSDF map from ROS topic!");
    if (publish_tsdf_info_) {
      publishAllUpdatedTsdfVoxels();
    }
    if (publish_slices_) {
      publishSlices();
    }
  }
}

}  // namespace voxblox

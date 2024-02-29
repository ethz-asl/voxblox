#include "voxblox_ros/tsdf_server.h"

#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>

#include "voxblox_ros/conversions.h"
#include "voxblox_ros/node_helper.h"

namespace voxblox {

TsdfServer::TsdfServer(rclcpp::Node::SharedPtr node)
    : node_(node),
      transformer_(node),
      verbose_(true),
      world_frame_("world"),
      icp_corrected_frame_("icp_corrected"),
      pose_corrected_frame_("pose_corrected"),
      max_block_distance_from_body_(std::numeric_limits<FloatingPoint>::max()),
      slice_level_(0.5),
      use_freespace_pointcloud_(false),
      color_map_(new RainbowColorMap()),
      publish_pointclouds_on_update_(false),
      publish_slices_(false),
      publish_pointclouds_(false),
      publish_tsdf_map_(false),
      cache_mesh_(false),
      enable_icp_(false),
      accumulate_icp_corrections_(true),
      pointcloud_queue_size_(1),
      num_subscribers_tsdf_map_(0) {
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);

  tsdf_config = getTsdfMapConfigFromRosParam();
  tsdf_integrator_config = getTsdfIntegratorConfigFromRosParam(tsdf_config);
  const MeshIntegratorConfig mesh_config =
      getMeshIntegratorConfigFromRosParam();

  getServerConfigFromRosParam();

  // Advertise topics.
  surface_pointcloud_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "surface_pointcloud", 1);
  tsdf_pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "tsdf_pointcloud", 1);
  occupancy_marker_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "occupied_nodes", 1);
  tsdf_slice_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("tsdf_slice", 1);

  pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud", rclcpp::QoS(pointcloud_queue_size_).best_effort(),
      std::bind(&TsdfServer::insertPointcloud, this, std::placeholders::_1));

  mesh_pub_ = node_->create_publisher<voxblox_msgs::msg::Mesh>("mesh", 1);

  // Publishing/subscribing to a layer from another node (when using this as
  // a library, for example within a planner).
  tsdf_map_pub_ =
      node_->create_publisher<voxblox_msgs::msg::Layer>("tsdf_map_out", 1);
  tsdf_map_sub_ = node_->create_subscription<voxblox_msgs::msg::Layer>(
      "tsdf_map_in", 1,
      std::bind(&TsdfServer::tsdfMapCallback, this, std::placeholders::_1));
  publish_tsdf_map_ =
      node_->declare_parameter("publish_tsdf_map", publish_tsdf_map_);

  if (use_freespace_pointcloud_) {
    // points that are not inside an object, but may also not be on a surface.
    // These will only be used to mark freespace beyond the truncation distance.
    freespace_pointcloud_sub_ =
        node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            "freespace_pointcloud", pointcloud_queue_size_,
            std::bind(&TsdfServer::insertFreespacePointcloud, this,
                      std::placeholders::_1));
  }

  if (enable_icp_) {
    icp_transform_pub_ =
        node_->create_publisher<geometry_msgs::msg::TransformStamped>(
            "icp_transform", 1);
    icp_corrected_frame_ =
        node_->declare_parameter("icp_corrected_frame", icp_corrected_frame_);
    pose_corrected_frame_ =
        node_->declare_parameter("pose_corrected_frame", pose_corrected_frame_);
  }

  // Initialize TSDF Map and integrator.
  tsdf_map_.reset(new TsdfMap(tsdf_config));

  std::string method("merged");
  method = node_->declare_parameter("method", method);
  if (method.compare("simple") == 0) {
    tsdf_integrator_.reset(new SimpleTsdfIntegrator(
        tsdf_integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else if (method.compare("merged") == 0) {
    tsdf_integrator_.reset(new MergedTsdfIntegrator(
        tsdf_integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else if (method.compare("fast") == 0) {
    tsdf_integrator_.reset(new FastTsdfIntegrator(
        tsdf_integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else {
    tsdf_integrator_.reset(new SimpleTsdfIntegrator(
        tsdf_integrator_config, tsdf_map_->getTsdfLayerPtr()));
  }

  mesh_layer_.reset(new MeshLayer(tsdf_map_->block_size()));

  mesh_integrator_.reset(new MeshIntegrator<TsdfVoxel>(
      mesh_config, tsdf_map_->getTsdfLayerPtr(), mesh_layer_.get()));

  icp_.reset(new ICP(getICPConfigFromRosParam()));

  // Advertise services.

  generate_mesh_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "generate_mesh", std::bind(&TsdfServer::generateMeshCallback, this,
                                 std::placeholders::_1, std::placeholders::_2));
  clear_map_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "clear_map", std::bind(&TsdfServer::clearMapCallback, this,
                             std::placeholders::_1, std::placeholders::_2));
  save_map_srv_ = node_->create_service<voxblox_msgs::srv::FilePath>(
      "save_map", std::bind(&TsdfServer::saveMapCallback, this,
                            std::placeholders::_1, std::placeholders::_2));
  load_map_srv_ = node_->create_service<voxblox_msgs::srv::FilePath>(
      "load_map", std::bind(&TsdfServer::loadMapCallback, this,
                            std::placeholders::_1, std::placeholders::_2));
  publish_pointclouds_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "publish_pointclouds",
      std::bind(&TsdfServer::publishPointcloudsCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  publish_tsdf_map_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "publish_map", std::bind(&TsdfServer::publishTsdfMapCallback, this,
                               std::placeholders::_1, std::placeholders::_2));

  // If set, use a timer to progressively integrate the mesh.
  double update_mesh_every_n_sec = 1.0;
  update_mesh_every_n_sec = node_->declare_parameter("update_mesh_every_n_sec",
                                                     update_mesh_every_n_sec);

  if (update_mesh_every_n_sec > 0.0) {
    update_mesh_timer_ = rclcpp::create_timer(
        node_, node_->get_clock(),
        rclcpp::Duration::from_seconds(update_mesh_every_n_sec),
        std::bind(&TsdfServer::updateMeshEvent, this));
  }

  double publish_map_every_n_sec = 1.0;
  publish_map_every_n_sec = node_->declare_parameter("publish_map_every_n_sec",
                                                     publish_map_every_n_sec);

  if (publish_map_every_n_sec > 0.0) {
    publish_map_timer_ = rclcpp::create_timer(
        node_, node_->get_clock(),
        rclcpp::Duration::from_seconds(publish_map_every_n_sec),
        std::bind(&TsdfServer::publishMapEvent, this));
  }
}

void TsdfServer::getServerConfigFromRosParam() {
  // Before subscribing, determine minimum time between messages.
  // 0 by default.
  min_time_between_msgs_sec_ = node_->declare_parameter(
      "min_time_between_msgs_sec", min_time_between_msgs_sec_);
  min_time_between_msgs_ =
      rclcpp::Duration::from_seconds(min_time_between_msgs_sec_);

  max_block_distance_from_body_ = node_->declare_parameter(
      "max_block_distance_from_body", max_block_distance_from_body_);
  slice_level_ = node_->declare_parameter("slice_level", slice_level_);
  world_frame_ =
      node_helper::declare_or_get_parameter(node_, "world_frame", world_frame_);
  publish_pointclouds_on_update_ = node_->declare_parameter(
      "publish_pointclouds_on_update", publish_pointclouds_on_update_);
  publish_slices_ = node_->declare_parameter("publish_slices", publish_slices_);
  publish_pointclouds_ =
      node_->declare_parameter("publish_pointclouds", publish_pointclouds_);
  use_freespace_pointcloud_ = node_->declare_parameter(
      "use_freespace_pointcloud", use_freespace_pointcloud_);
  pointcloud_queue_size_ =
      node_->declare_parameter("pointcloud_queue_size", pointcloud_queue_size_);
  enable_icp_ = node_->declare_parameter("enable_icp", enable_icp_);
  accumulate_icp_corrections_ = node_->declare_parameter(
      "accumulate_icp_corrections", accumulate_icp_corrections_);
  verbose_ = node_->declare_parameter("verbose", verbose_);

  // Mesh settings.
  mesh_filename_ = node_->declare_parameter("mesh_filename", mesh_filename_);

  std::string color_mode("");
  color_mode = node_->declare_parameter("color_mode", color_mode);
  color_mode_ = getColorModeFromString(color_mode);

  // Color map for intensity pointclouds.
  std::string intensity_colormap("rainbow");
  intensity_colormap =
      node_->declare_parameter("intensity_colormap", intensity_colormap);

  float intensity_max_value = kDefaultMaxIntensity;
  intensity_max_value =
      node_->declare_parameter("intensity_max_value", intensity_max_value);

  // Default set in constructor.
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
    RCLCPP_ERROR_STREAM(node_->get_logger(),
                        "Invalid color map: " << intensity_colormap);
  }
  color_map_->setMaxValue(intensity_max_value);
}

void TsdfServer::processPointCloudMessageAndInsert(
    const sensor_msgs::msg::PointCloud2::SharedPtr& pointcloud_msg,
    const Transformation& T_G_C, const bool is_freespace_pointcloud) {
  // Convert the PCL pointcloud into our awesome format.

  // Horrible hack fix to fix color parsing colors in PCL.
  bool color_pointcloud = false;
  bool has_intensity = false;
  for (size_t d = 0; d < pointcloud_msg->fields.size(); ++d) {
    if (pointcloud_msg->fields[d].name == std::string("rgb")) {
      pointcloud_msg->fields[d].datatype =
          sensor_msgs::msg::PointField::FLOAT32;
      color_pointcloud = true;
    } else if (pointcloud_msg->fields[d].name == std::string("intensity")) {
      has_intensity = true;
    }
  }

  Pointcloud points_C;
  Colors colors;
  timing::Timer ptcloud_timer("ptcloud_preprocess");

  // Convert differently depending on RGB or I type.
  if (color_pointcloud) {
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  } else if (has_intensity) {
    pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  } else {
    pcl::PointCloud<pcl::PointXYZ> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  }
  ptcloud_timer.Stop();

  Transformation T_G_C_refined = T_G_C;
  if (enable_icp_) {
    timing::Timer icp_timer("icp");
    if (!accumulate_icp_corrections_) {
      icp_corrected_transform_.setIdentity();
    }
    static Transformation T_offset;
    const size_t num_icp_updates =
        icp_->runICP(tsdf_map_->getTsdfLayer(), points_C,
                     icp_corrected_transform_ * T_G_C, &T_G_C_refined);
    if (verbose_) {
      RCLCPP_INFO(node_->get_logger(),
                  "ICP refinement performed %zu successful update steps",
                  num_icp_updates);
    }
    icp_corrected_transform_ = T_G_C_refined * T_G_C.inverse();

    if (!icp_->refiningRollPitch()) {
      // its already removed internally but small floating point errors can
      // build up if accumulating transforms
      Transformation::Vector6 T_vec = icp_corrected_transform_.log();
      T_vec[3] = 0.0;
      T_vec[4] = 0.0;
      icp_corrected_transform_ = Transformation::exp(T_vec);
    }

    // Publish transforms as both TF and message.
    tf2::Transform icp_tf_msg;
    tf2::Transform pose_tf_msg;

    tf2::transformKindrToTF(icp_corrected_transform_.cast<double>(),
                            &icp_tf_msg);
    tf2::transformKindrToTF(
        T_G_C.cast<double>(),
        &pose_tf_msg);  // TODO: also publish pose as Pose message

    geometry_msgs::msg::TransformStamped icp_corrected_transform_msg;
    tf2::transformKindrToMsg(icp_corrected_transform_.cast<double>(),
                             &icp_corrected_transform_msg.transform);
    icp_corrected_transform_msg.header.stamp = pointcloud_msg->header.stamp;
    icp_corrected_transform_msg.header.frame_id = world_frame_;
    icp_corrected_transform_msg.child_frame_id = icp_corrected_frame_;
    tf_broadcaster_->sendTransform(icp_corrected_transform_msg);

    geometry_msgs::msg::TransformStamped T_G_C_msg;
    tf2::transformKindrToMsg(T_G_C.cast<double>(), &T_G_C_msg.transform);
    T_G_C_msg.header.stamp = pointcloud_msg->header.stamp;
    T_G_C_msg.header.frame_id = world_frame_;
    T_G_C_msg.child_frame_id = pose_corrected_frame_;
    tf_broadcaster_->sendTransform(T_G_C_msg);

    icp_transform_pub_->publish(icp_corrected_transform_msg);

    icp_timer.Stop();
  }

  if (verbose_) {
    RCLCPP_INFO(node_->get_logger(),
                "Integrating a pointcloud with %lu points.", points_C.size());
  }

  rclcpp::Time start = rclcpp::Clock().now();
  integratePointcloud(T_G_C_refined, points_C, colors, is_freespace_pointcloud);
  rclcpp::Time end = rclcpp::Clock().now();
  auto integrating_time = (start - end);
  if (verbose_) {
    RCLCPP_INFO(node_->get_logger(),
                "Finished integrating in %f seconds, have %lu blocks.",
                integrating_time.seconds(),
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

// Checks if we can get the next message from queue.
bool TsdfServer::getNextPointcloudFromQueue(
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr>* queue,
    sensor_msgs::msg::PointCloud2::SharedPtr* pointcloud_msg,
    Transformation* T_G_C) {
  const size_t kMaxQueueSize = 10;
  if (queue->empty()) {
    return false;
  }
  *pointcloud_msg = queue->front();
  if (transformer_.lookupTransform((*pointcloud_msg)->header.frame_id,
                                   world_frame_,
                                   (*pointcloud_msg)->header.stamp, T_G_C)) {
    queue->pop();
    return true;
  } else {
    if (queue->size() >= kMaxQueueSize) {
      RCLCPP_ERROR_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 60,
          "Input pointcloud queue getting too long! Dropping "
          "some pointclouds. Either unable to look up transform "
          "timestamps or the processing is taking too long.");
      while (queue->size() >= kMaxQueueSize) {
        queue->pop();
      }
    }
  }
  return false;
}

void TsdfServer::insertPointcloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg_in) {
  rclcpp::Time current_msg_time = pointcloud_msg_in->header.stamp;
  if (last_msg_time_ptcloud_.get_clock_type() !=
      current_msg_time.get_clock_type()) {  // TODO: figure out how to remove
                                            // this check at every iteration
    last_msg_time_ptcloud_ = current_msg_time;
  }
  rclcpp::Duration time_since_last_message =
      (current_msg_time - last_msg_time_ptcloud_);

  if (time_since_last_message > min_time_between_msgs_) {
    last_msg_time_ptcloud_ = (pointcloud_msg_in->header.stamp);
    // So we have to process the queue anyway... Push this back.
    pointcloud_queue_.push(pointcloud_msg_in);
  }

  Transformation T_G_C;
  sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg;
  bool processed_any = false;
  while (
      getNextPointcloudFromQueue(&pointcloud_queue_, &pointcloud_msg, &T_G_C)) {
    constexpr bool is_freespace_pointcloud = false;
    processPointCloudMessageAndInsert(pointcloud_msg, T_G_C,
                                      is_freespace_pointcloud);
    processed_any = true;
  }

  if (!processed_any) {
    return;
  }

  if (publish_pointclouds_on_update_) {
    publishPointclouds();
  }

  if (verbose_) {
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "Timings: " << std::endl
                                   << timing::Timing::Print());
    RCLCPP_INFO_STREAM(
        node_->get_logger(),
        "Layer memory: " << tsdf_map_->getTsdfLayer().getMemorySize());
  }
}

void TsdfServer::insertFreespacePointcloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg_in) {
  rclcpp::Time current_msg_time = pointcloud_msg_in->header.stamp;
  if (last_msg_time_freespace_ptcloud_.get_clock_type() !=
      current_msg_time.get_clock_type()) {  // TODO: figure out how to remove
                                            // this check at every iteration
    last_msg_time_ptcloud_ = current_msg_time;
  }
  rclcpp::Duration time_since_last_message =
      (current_msg_time - last_msg_time_freespace_ptcloud_);

  if (time_since_last_message > min_time_between_msgs_) {
    last_msg_time_freespace_ptcloud_ = pointcloud_msg_in->header.stamp;
    // So we have to process the queue anyway... Push this back.
    freespace_pointcloud_queue_.push(pointcloud_msg_in);
  }

  Transformation T_G_C;
  sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg;
  while (getNextPointcloudFromQueue(&freespace_pointcloud_queue_,
                                    &pointcloud_msg, &T_G_C)) {
    constexpr bool is_freespace_pointcloud = true;
    processPointCloudMessageAndInsert(pointcloud_msg, T_G_C,
                                      is_freespace_pointcloud);
  }
}

void TsdfServer::integratePointcloud(const Transformation& T_G_C,
                                     const Pointcloud& ptcloud_C,
                                     const Colors& colors,
                                     const bool is_freespace_pointcloud) {
  CHECK_EQ(ptcloud_C.size(), colors.size());
  tsdf_integrator_->integratePointCloud(T_G_C, ptcloud_C, colors,
                                        is_freespace_pointcloud);
}

void TsdfServer::publishAllUpdatedTsdfVoxels() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromTsdfLayer(tsdf_map_->getTsdfLayer(), &pointcloud);

  pointcloud.header.frame_id = world_frame_;

  sensor_msgs::msg::PointCloud2 pointcloud_message;
  pcl::toROSMsg(pointcloud, pointcloud_message);

  tsdf_pointcloud_pub_->publish(pointcloud_message);
}

void TsdfServer::publishTsdfSurfacePoints() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  const float surface_distance_thresh =
      tsdf_map_->getTsdfLayer().voxel_size() * 0.75;
  createSurfacePointcloudFromTsdfLayer(tsdf_map_->getTsdfLayer(),
                                       surface_distance_thresh, &pointcloud);

  pointcloud.header.frame_id = world_frame_;

  sensor_msgs::msg::PointCloud2 pointcloud_message;
  pcl::toROSMsg(pointcloud, pointcloud_message);

  surface_pointcloud_pub_->publish(pointcloud_message);
}

void TsdfServer::publishTsdfOccupiedNodes() {
  // Create a pointcloud with distance = intensity.
  visualization_msgs::msg::MarkerArray marker_array;
  createOccupancyBlocksFromTsdfLayer(tsdf_map_->getTsdfLayer(), world_frame_,
                                     &marker_array);
  occupancy_marker_pub_->publish(marker_array);
}

void TsdfServer::publishSlices() {
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromTsdfLayerSlice(tsdf_map_->getTsdfLayer(), 2,
                                             slice_level_, &pointcloud);

  pointcloud.header.frame_id = world_frame_;

  sensor_msgs::msg::PointCloud2 pointcloud_message;
  pcl::toROSMsg(pointcloud, pointcloud_message);

  tsdf_slice_pub_->publish(pointcloud_message);
}

void TsdfServer::publishMap(bool reset_remote_map) {
  if (!publish_tsdf_map_) {
    return;
  }
  int subscribers = tsdf_map_pub_->get_subscription_count();
  if (subscribers > 0) {
    if (num_subscribers_tsdf_map_ < subscribers) {
      // Always reset the remote map and send all when a new subscriber
      // subscribes. A bit of overhead for other subscribers, but better than
      // inconsistent map states.
      reset_remote_map = true;
    }
    const bool only_updated = !reset_remote_map;
    timing::Timer publish_map_timer("map/publish_tsdf");
    voxblox_msgs::msg::Layer layer_msg;
    serializeLayerAsMsg<TsdfVoxel>(tsdf_map_->getTsdfLayer(), only_updated,
                                   &layer_msg);
    if (reset_remote_map) {
      layer_msg.action = static_cast<uint8_t>(MapDerializationAction::kReset);
    }
    tsdf_map_pub_->publish(layer_msg);
    publish_map_timer.Stop();
  }
  num_subscribers_tsdf_map_ = subscribers;
}

void TsdfServer::publishPointclouds() {
  // Combined function to publish all possible pointcloud messages -- surface
  // pointclouds, updated points, and occupied points.
  publishAllUpdatedTsdfVoxels();
  publishTsdfSurfacePoints();
  publishTsdfOccupiedNodes();
  if (publish_slices_) {
    publishSlices();
  }
}

void TsdfServer::updateMesh() {
  if (verbose_) {
    RCLCPP_INFO(node_->get_logger(), "Updating mesh.");
  }

  timing::Timer generate_mesh_timer("mesh/update");
  constexpr bool only_mesh_updated_blocks = true;
  constexpr bool clear_updated_flag = true;
  mesh_integrator_->generateMesh(only_mesh_updated_blocks, clear_updated_flag);
  generate_mesh_timer.Stop();

  timing::Timer publish_mesh_timer("mesh/publish");

  voxblox_msgs::msg::Mesh mesh_msg;
  generateVoxbloxMeshMsg(mesh_layer_, color_mode_, &mesh_msg);
  mesh_msg.header.frame_id = world_frame_;
  mesh_pub_->publish(mesh_msg);

  if (cache_mesh_) {
    cached_mesh_msg_ = mesh_msg;
  }

  publish_mesh_timer.Stop();

  if (publish_pointclouds_ && !publish_pointclouds_on_update_) {
    publishPointclouds();
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
  voxblox_msgs::msg::Mesh mesh_msg;
  generateVoxbloxMeshMsg(mesh_layer_, color_mode_, &mesh_msg);
  mesh_msg.header.frame_id = world_frame_;
  mesh_pub_->publish(mesh_msg);

  publish_mesh_timer.Stop();

  if (!mesh_filename_.empty()) {
    timing::Timer output_mesh_timer("mesh/output");
    const bool success = outputMeshLayerAsPly(mesh_filename_, *mesh_layer_);
    output_mesh_timer.Stop();
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "Output file as PLY: %s",
                  mesh_filename_.c_str());
    } else {
      RCLCPP_INFO(node_->get_logger(), "Failed to output mesh as PLY: %s",
                  mesh_filename_.c_str());
    }
  }

  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "Mesh Timings: " << std::endl
                                      << timing::Timing::Print());
  return true;
}

bool TsdfServer::saveMap(const std::string& file_path) {
  // Inheriting classes should add saving other layers to this function.
  return io::SaveLayer(tsdf_map_->getTsdfLayer(), file_path);
}

bool TsdfServer::loadMap(const std::string& file_path) {
  // Inheriting classes should add other layers to load, as this will only
  // load
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

void TsdfServer::clearMapCallback(
    std_srvs::srv::Empty::Request::SharedPtr /*request*/,
    std_srvs::srv::Empty::Response::SharedPtr /*response*/) {  // NOLINT
  clear();
}

void TsdfServer::generateMeshCallback(
    std_srvs::srv::Empty::Request::SharedPtr /*request*/,
    std_srvs::srv::Empty::Response::SharedPtr /*response*/) {  // NOLINT
  generateMesh();
}

void TsdfServer::saveMapCallback(
    voxblox_msgs::srv::FilePath::Request::SharedPtr request,
    voxblox_msgs::srv::FilePath::Response::SharedPtr /*response*/) {  // NOLINT
  saveMap(request->file_path);
}

void TsdfServer::loadMapCallback(
    voxblox_msgs::srv::FilePath::Request::SharedPtr request,
    voxblox_msgs::srv::FilePath::Response::SharedPtr /*response*/) {  // NOLINT
  loadMap(request->file_path);
}

void TsdfServer::publishPointcloudsCallback(
    std_srvs::srv::Empty::Request::SharedPtr /*request*/,
    std_srvs::srv::Empty::Response::SharedPtr /*response*/) {  // NOLINT
  publishPointclouds();
}

void TsdfServer::publishTsdfMapCallback(
    std_srvs::srv::Empty::Request::SharedPtr /*request*/,
    std_srvs::srv::Empty::Response::SharedPtr /*response*/) {  // NOLINT
  publishMap();
}

void TsdfServer::updateMeshEvent() { updateMesh(); }

void TsdfServer::publishMapEvent() { publishMap(); }

void TsdfServer::clear() {
  tsdf_map_->getTsdfLayerPtr()->removeAllBlocks();
  mesh_layer_->clear();

  // Publish a message to reset the map to all subscribers.
  if (publish_tsdf_map_) {
    constexpr bool kResetRemoteMap = true;
    publishMap(kResetRemoteMap);
  }
}

void TsdfServer::tsdfMapCallback(
    const voxblox_msgs::msg::Layer::SharedPtr layer_msg) {
  timing::Timer receive_map_timer("map/receive_tsdf");

  bool success =
      deserializeMsgToLayer<TsdfVoxel>(layer_msg, tsdf_map_->getTsdfLayerPtr());

  if (!success) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 10,
                          "Got an invalid TSDF map message!");
  } else {
    RCLCPP_INFO_ONCE(node_->get_logger(), "Got an TSDF map from ROS topic!");
    if (publish_pointclouds_on_update_) {
      publishPointclouds();
    }
  }
}

}  // namespace voxblox

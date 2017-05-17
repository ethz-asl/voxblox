#include "voxblox_ros/tsdf_server.h"

namespace voxblox {

TsdfServer::TsdfServer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(true),
      world_frame_("world"),
      slice_level_(0.5),
      transformer_(nh, nh_private) {
  // Before subscribing, determine minimum time between messages.
  // 0 by default.
  double min_time_between_msgs_sec = 0.0;
  nh_private_.param("min_time_between_msgs_sec", min_time_between_msgs_sec,
                    min_time_between_msgs_sec);
  min_time_between_msgs_.fromSec(min_time_between_msgs_sec);

  nh_private_.param("slice_level", slice_level_, slice_level_);
  nh_private_.param("world_frame", world_frame_, world_frame_);

  // Advertise topics.
  mesh_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("mesh", 1, true);
  surface_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
          "surface_pointcloud", 1, true);
  tsdf_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >("tsdf_pointcloud",
                                                              1, true);

  tsdf_slice_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "tsdf_slice", 1, true);

  int pointcloud_queue_size = 1;
  nh_private_.param("pointcloud_queue_size", pointcloud_queue_size,
                    pointcloud_queue_size);
  pointcloud_sub_ = nh_.subscribe("pointcloud", pointcloud_queue_size,
                                  &TsdfServer::insertPointcloud, this);

  nh_private_.param("verbose", verbose_, verbose_);
  std::string method("merged");
  nh_private_.param("method", method, method);
  if (method.compare("simple") == 0) {
    method_ = kSimple;
  } else if (method.compare("merged") == 0) {
    method_ = kMerged;
  } else if (method.compare("merged_discard") == 0) {
    method_ = kMergedDiscard;
  } else {
    method_ = kSimple;
  }

  // Determine map parameters.
  TsdfMap::Config config;
  // Workaround for OS X on mac mini not having specializations for float
  // for some reason.
  double voxel_size = config.tsdf_voxel_size;
  int voxels_per_side = config.tsdf_voxels_per_side;
  nh_private_.param("tsdf_voxel_size", voxel_size, voxel_size);
  nh_private_.param("tsdf_voxels_per_side", voxels_per_side, voxels_per_side);
  config.tsdf_voxel_size = static_cast<FloatingPoint>(voxel_size);
  config.tsdf_voxels_per_side = voxels_per_side;
  tsdf_map_.reset(new TsdfMap(config));

  // Determine integrator parameters.
  TsdfIntegrator::Config integrator_config;
  integrator_config.voxel_carving_enabled = true;
  // Used to be * 4 according to Marius's experience, now * 2.
  // This should be made bigger again if behind-surface weighting is improved.
  integrator_config.default_truncation_distance = config.tsdf_voxel_size * 2;

  double truncation_distance = integrator_config.default_truncation_distance;
  double max_weight = integrator_config.max_weight;
  nh_private_.param("voxel_carving_enabled",
                    integrator_config.voxel_carving_enabled,
                    integrator_config.voxel_carving_enabled);
  nh_private_.param("truncation_distance", truncation_distance,
                    truncation_distance);
  nh_private_.param("max_ray_length_m", integrator_config.max_ray_length_m,
                    integrator_config.max_ray_length_m);
  nh_private_.param("min_ray_length_m", integrator_config.min_ray_length_m,
                    integrator_config.min_ray_length_m);
  nh_private_.param("max_weight", max_weight, max_weight);
  nh_private_.param("use_const_weight", integrator_config.use_const_weight,
                    integrator_config.use_const_weight);
  nh_private_.param("allow_clear", integrator_config.allow_clear,
                    integrator_config.allow_clear);
  integrator_config.default_truncation_distance =
      static_cast<float>(truncation_distance);
  integrator_config.max_weight = static_cast<float>(max_weight);

  tsdf_integrator_.reset(
      new TsdfIntegrator(integrator_config, tsdf_map_->getTsdfLayerPtr()));

  // Mesh settings.
  nh_private_.param("mesh_filename", mesh_filename_, mesh_filename_);
  std::string color_mode("colors");
  nh_private_.param("color_mode", color_mode, color_mode);
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

  MeshIntegrator::Config mesh_config;
  nh_private_.param("mesh_min_weight", mesh_config.min_weight,
                    mesh_config.min_weight);

  mesh_layer_.reset(new MeshLayer(tsdf_map_->block_size()));

  mesh_integrator_.reset(new MeshIntegrator(
      mesh_config, tsdf_map_->getTsdfLayerPtr(), mesh_layer_.get()));

  // Advertise services.
  generate_mesh_srv_ = nh_private_.advertiseService(
      "generate_mesh", &TsdfServer::generateMeshCallback, this);
  save_map_srv_ = nh_private_.advertiseService(
      "save_map", &TsdfServer::saveMapCallback, this);
  load_map_srv_ = nh_private_.advertiseService(
      "load_map", &TsdfServer::loadMapCallback, this);

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

void TsdfServer::insertPointcloud(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg) {
  // Figure out if we should insert this.
  if (pointcloud_msg->header.stamp - last_msg_time_ < min_time_between_msgs_) {
    return;
  }
  last_msg_time_ = pointcloud_msg->header.stamp;

  // Look up transform from sensor frame to world frame.
  Transformation T_G_C;
  if (transformer_.lookupTransform(pointcloud_msg->header.frame_id,
                                   world_frame_, pointcloud_msg->header.stamp,
                                   &T_G_C)) {
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

    // Filter out NaNs. :|
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(pointcloud_pcl, pointcloud_pcl, indices);

    Pointcloud points_C;
    Colors colors;
    points_C.reserve(pointcloud_pcl.size());
    colors.reserve(pointcloud_pcl.size());
    for (size_t i = 0; i < pointcloud_pcl.points.size(); ++i) {
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
    if (method_ == Method::kMerged) {
      bool discard = false;
      tsdf_integrator_->integratePointCloudMerged(T_G_C, points_C, colors,
                                                  discard);
    } else if (method_ == Method::kMergedDiscard) {
      bool discard = true;
      tsdf_integrator_->integratePointCloudMerged(T_G_C, points_C, colors,
                                                  discard);
    } else {
      tsdf_integrator_->integratePointCloud(T_G_C, points_C, colors);
    }
    ros::WallTime end = ros::WallTime::now();
    if (verbose_) {
      ROS_INFO("Finished integrating in %f seconds, have %lu blocks.",
               (end - start).toSec(),
               tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks());
    }

    publishAllUpdatedTsdfVoxels();
    publishTsdfSurfacePoints();
    publishTsdfOccupiedNodes();
    publishSlices();

    // Callback for inheriting classes.
    newPoseCallback(T_G_C);

    if (verbose_) {
      ROS_INFO_STREAM("Timings: " << std::endl << timing::Timing::Print());
    }
  }
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

bool TsdfServer::generateMeshCallback(
    std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& response) {  // NOLINT
  timing::Timer generate_mesh_timer("mesh/generate");
  const bool clear_mesh = true;
  if (clear_mesh) {
    mesh_integrator_->generateWholeMesh();
  } else {
    const bool clear_updated_flag = true;
    mesh_integrator_->generateMeshForUpdatedBlocks(clear_updated_flag);
  }
  generate_mesh_timer.Stop();

  timing::Timer publish_mesh_timer("mesh/publish");
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(1);
  fillMarkerWithMesh(mesh_layer_, color_mode_, &marker_array.markers[0]);
  marker_array.markers[0].header.frame_id = world_frame_;
  mesh_pub_.publish(marker_array);
  publish_mesh_timer.Stop();

  if (!mesh_filename_.empty()) {
    timing::Timer output_mesh_timer("mesh/output");
    bool success = outputMeshLayerAsPly(mesh_filename_, *mesh_layer_);
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

bool TsdfServer::saveMapCallback(
    voxblox_msgs::FilePath::Request& request,
    voxblox_msgs::FilePath::Response& response) {  // NOLINT
  // Will only save TSDF layer for now.
  return io::SaveLayer(tsdf_map_->getTsdfLayer(), request.file_path);
}

bool TsdfServer::loadMapCallback(
    voxblox_msgs::FilePath::Request& request,
    voxblox_msgs::FilePath::Response& response) {  // NOLINT
  // Will only load TSDF layer for now.
  return io::LoadBlocksFromFile(
      request.file_path, Layer<TsdfVoxel>::BlockMergingStrategy::kReplace,
      tsdf_map_->getTsdfLayerPtr());
}

void TsdfServer::updateMeshEvent(const ros::TimerEvent& event) {
  if (verbose_) {
    ROS_INFO("Updating mesh.");
  }

  timing::Timer generate_mesh_timer("mesh/update");
  const bool clear_updated_flag = true;
  mesh_integrator_->generateMeshForUpdatedBlocks(clear_updated_flag);
  generate_mesh_timer.Stop();

  // TODO(helenol): also think about how to update markers incrementally?
  timing::Timer publish_mesh_timer("mesh/publish");
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(1);
  fillMarkerWithMesh(mesh_layer_, color_mode_, &marker_array.markers[0]);
  marker_array.markers[0].header.frame_id = world_frame_;
  mesh_pub_.publish(marker_array);
  publish_mesh_timer.Stop();
}

}  // namespace voxblox

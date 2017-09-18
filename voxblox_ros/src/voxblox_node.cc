#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <minkindr_conversions/kindr_xml.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <voxblox/core/esdf_map.h>
#include <voxblox/core/occupancy_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/occupancy_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>

#include <voxblox_msgs/FilePath.h>
#include "voxblox_ros/mesh_pcl.h"
#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"

namespace voxblox {

class VoxbloxNode {
 public:
  VoxbloxNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  void processPointCloudMessageAndInsert(
      const sensor_msgs::PointCloud2::Ptr& pointcloud_msg,
      const bool is_freespace_pointcloud = false);

  void insertPointcloudWithTf(const sensor_msgs::PointCloud2::Ptr& pointcloud);

  void insertFreespacePointcloudWithTf(
      const sensor_msgs::PointCloud2::Ptr& freespace_pointcloud);

  bool lookupTransform(const std::string& from_frame,
                       const std::string& to_frame, const ros::Time& timestamp,
                       Transformation* transform);
  bool lookupTransformTf(const std::string& from_frame,
                         const std::string& to_frame,
                         const ros::Time& timestamp, Transformation* transform);
  bool lookupTransformQueue(const std::string& from_frame,
                            const std::string& to_frame,
                            const ros::Time& timestamp,
                            Transformation* transform);

  void publishAllUpdatedTsdfVoxels();
  void publishAllUpdatedEsdfVoxels();
  void publishTsdfSurfacePoints();
  void publishTsdfOccupiedNodes();
  void publishOccupancy();
  void publishSlices();

  void transformCallback(const geometry_msgs::TransformStamped& transform_msg);

  bool generateMeshCallback(std_srvs::Empty::Request& request,       // NOLINT
                            std_srvs::Empty::Response& response);    // NOLINT
  bool generateEsdfCallback(std_srvs::Empty::Request& request,       // NOLINT
                            std_srvs::Empty::Response& response);    // NOLINT
  bool saveMapCallback(voxblox_msgs::FilePath::Request& request,     // NOLINT
                       voxblox_msgs::FilePath::Response& response);  // NOLINT
  bool loadMapCallback(voxblox_msgs::FilePath::Request& request,     // NOLINT
                       voxblox_msgs::FilePath::Response& response);  // NOLINT

  void updateMeshEvent(const ros::TimerEvent& e);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  bool verbose_;

  // This is a debug option, more or less...
  bool color_ptcloud_by_weight_;

  // Which maps to generate.
  bool generate_esdf_;
  bool generate_occupancy_;

  // What output information to publish
  bool publish_tsdf_info_;
  bool publish_slices_;

  bool output_mesh_as_pointcloud_;
  bool output_mesh_as_pcl_mesh_;

  // Global/map coordinate frame. Will always look up TF transforms to this
  // frame.
  std::string world_frame_;
  // If set, overwrite sensor frame with this value. If empty, unused.
  std::string sensor_frame_;
  // Whether to use TF transform resolution (true) or fixed transforms from
  // parameters and transform topics (false).
  bool use_tf_transforms_;
  int64_t timestamp_tolerance_ns_;
  // B is the body frame of the robot, C is the camera/sensor frame creating
  // the pointclouds, and D is the 'dynamic' frame; i.e., incoming messages
  // are assumed to be T_G_D.
  Transformation T_B_C_;
  Transformation T_B_D_;

  // Pointcloud visualization settings.
  double slice_level_;

  // If the system should subscribe to a pointcloud giving points in freespace
  bool use_freespace_pointcloud_;

  // Mesh output settings. Mesh is only written to file if mesh_filename_ is
  // not empty.
  std::string mesh_filename_;
  // How to color the mesh.
  ColorMode color_mode_;

  // Keep track of these for throttling.
  ros::Duration min_time_between_msgs_;

  // To be replaced (at least optionally) with odometry + static transform
  // from IMU to visual frame.
  tf::TransformListener tf_listener_;

  // Data subscribers.
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber freespace_pointcloud_sub_;

  // Only used if use_tf_transforms_ set to false.
  ros::Subscriber transform_sub_;

  // Publish markers for visualization.
  ros::Publisher mesh_pub_;
  ros::Publisher mesh_pointcloud_pub_;
  ros::Publisher mesh_pcl_mesh_pub_;
  ros::Publisher tsdf_pointcloud_pub_;
  ros::Publisher esdf_pointcloud_pub_;
  ros::Publisher surface_pointcloud_pub_;
  ros::Publisher occupancy_marker_pub_;
  ros::Publisher occupancy_layer_pub_;
  ros::Publisher tsdf_slice_pub_;
  ros::Publisher esdf_slice_pub_;

  // Services.
  ros::ServiceServer generate_mesh_srv_;
  ros::ServiceServer generate_esdf_srv_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer load_map_srv_;

  // Timers.
  ros::Timer update_mesh_timer_;

  // Maps and integrators.
  std::shared_ptr<TsdfMap> tsdf_map_;
  std::shared_ptr<TsdfIntegratorBase> tsdf_integrator_;
  // ESDF maps (optional).
  std::shared_ptr<EsdfMap> esdf_map_;
  std::shared_ptr<EsdfIntegrator> esdf_integrator_;
  // Occupancy maps (optional).
  std::shared_ptr<OccupancyMap> occupancy_map_;
  std::shared_ptr<OccupancyIntegrator> occupancy_integrator_;
  // Mesh accessories.
  std::shared_ptr<MeshLayer> mesh_layer_;
  std::shared_ptr<MeshIntegrator<TsdfVoxel>> mesh_integrator_;

  // Transform queue, used only when use_tf_transforms is false.
  AlignedDeque<geometry_msgs::TransformStamped> transform_queue_;
};

VoxbloxNode::VoxbloxNode(const ros::NodeHandle& nh,
                         const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(true),
      color_ptcloud_by_weight_(false),
      generate_esdf_(false),
      generate_occupancy_(false),
      publish_tsdf_info_(false),
      publish_slices_(false),
      output_mesh_as_pointcloud_(false),
      output_mesh_as_pcl_mesh_(false),
      world_frame_("world"),
      sensor_frame_(""),
      use_tf_transforms_(true),
      // 10 ms here:
      timestamp_tolerance_ns_(10000000),
      slice_level_(0.5),
      use_freespace_pointcloud_(false) {
  // Before subscribing, determine minimum time between messages.
  // 0 by default.
  double min_time_between_msgs_sec = 0.0;
  nh_private_.param("min_time_between_msgs_sec", min_time_between_msgs_sec,
                    min_time_between_msgs_sec);
  min_time_between_msgs_.fromSec(min_time_between_msgs_sec);

  // Determine which parts to generate.
  nh_private_.param("generate_esdf", generate_esdf_, generate_esdf_);
  nh_private_.param("output_mesh_as_pointcloud", output_mesh_as_pointcloud_,
                    output_mesh_as_pointcloud_);
  nh_private_.param("output_mesh_as_pcl_mesh", output_mesh_as_pcl_mesh_,
                    output_mesh_as_pcl_mesh_);
  nh_private_.param("slice_level", slice_level_, slice_level_);
  nh_private_.param("world_frame", world_frame_, world_frame_);
  nh_private_.param("sensor_frame", sensor_frame_, sensor_frame_);
  nh_private_.param("publish_tsdf_info", publish_tsdf_info_,
                    publish_tsdf_info_);
  nh_private_.param("publish_slices", publish_slices_, publish_slices_);

  // Advertise topics.
  mesh_pub_ = nh_private_.advertise<voxblox_msgs::Mesh>("mesh", 1, true);

  if (publish_tsdf_info_) {
    surface_pointcloud_pub_ =
        nh_private_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
            "surface_pointcloud", 1, true);
    tsdf_pointcloud_pub_ =
        nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI>>(
            "tsdf_pointcloud", 1, true);
    occupancy_marker_pub_ =
        nh_private_.advertise<visualization_msgs::MarkerArray>("occupied_nodes",
                                                               1, true);
  }

  if (output_mesh_as_pointcloud_) {
    mesh_pointcloud_pub_ =
        nh_private_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
            "mesh_pointcloud", 1, true);
  }

  if (output_mesh_as_pcl_mesh_) {
    mesh_pcl_mesh_pub_ =
        nh_private_.advertise<pcl_msgs::PolygonMesh>("pcl_mesh", 1, true);
  }

  if (publish_slices_) {
    tsdf_slice_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI>>(
        "tsdf_slice", 1, true);

    if (generate_esdf_) {
      esdf_slice_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI>>(
          "esdf_slice", 1, true);
    }
  }

  if (generate_esdf_) {
    esdf_pointcloud_pub_ =
        nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI>>(
            "esdf_pointcloud", 1, true);
  }

  if (generate_occupancy_) {
    occupancy_layer_pub_ =
        nh_private_.advertise<visualization_msgs::MarkerArray>(
            "occupancy_layer", 1, true);
  }

  int pointcloud_queue_size = 1;
  nh_private_.param("pointcloud_queue_size", pointcloud_queue_size,
                    pointcloud_queue_size);

  pointcloud_sub_ = nh_.subscribe("pointcloud", pointcloud_queue_size,
                                  &VoxbloxNode::insertPointcloudWithTf, this);

  nh_private_.param("use_freespace_pointcloud", use_freespace_pointcloud_,
                    use_freespace_pointcloud_);
  if (use_freespace_pointcloud_) {
    // points that are not inside an object, but may also not be on a surface.
    // These will only be used to mark freespace beyond the truncation distance.
    freespace_pointcloud_sub_ =
        nh_.subscribe("freespace_pointcloud", pointcloud_queue_size,
                      &VoxbloxNode::insertFreespacePointcloudWithTf, this);
  }

  nh_private_.param("verbose", verbose_, verbose_);
  nh_private_.param("color_ptcloud_by_weight", color_ptcloud_by_weight_,
                    color_ptcloud_by_weight_);

  // Determine map parameters.
  TsdfMap::Config config;

  // Workaround for OS X on mac mini not having specializations for float
  // for some reason.
  double voxel_size = config.tsdf_voxel_size;
  int voxels_per_side = config.tsdf_voxels_per_side;
  nh_private_.param("tsdf_voxel_size", voxel_size, voxel_size);
  nh_private_.param("tsdf_voxels_per_side", voxels_per_side, voxels_per_side);
  if (!isPowerOfTwo(voxels_per_side)) {
    ROS_ERROR("voxels_per_side must be a power of 2, setting to default value");
    voxels_per_side = config.tsdf_voxels_per_side;
  }
  config.tsdf_voxel_size = static_cast<FloatingPoint>(voxel_size);
  config.tsdf_voxels_per_side = voxels_per_side;
  tsdf_map_.reset(new TsdfMap(config));

  // Determine integrator parameters.
  TsdfIntegratorBase::Config integrator_config;
  integrator_config.voxel_carving_enabled = true;
  // Used to be * 4 according to Marius's experience, was changed to *2
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
  nh_private_.param("start_voxel_subsampling_factor",
                    integrator_config.start_voxel_subsampling_factor,
                    integrator_config.start_voxel_subsampling_factor);
  nh_private_.param("max_consecutive_ray_collisions",
                    integrator_config.max_consecutive_ray_collisions,
                    integrator_config.max_consecutive_ray_collisions);
  nh_private_.param("clear_checks_every_n_frames",
                    integrator_config.clear_checks_every_n_frames,
                    integrator_config.clear_checks_every_n_frames);
  nh_private_.param("max_integration_time_s",
                    integrator_config.max_integration_time_s,
                    integrator_config.max_integration_time_s);
  integrator_config.default_truncation_distance =
      static_cast<float>(truncation_distance);
  integrator_config.max_weight = static_cast<float>(max_weight);

  std::string method("merged");
  nh_private_.param("method", method, method);
  if (method.compare("simple") == 0) {
    tsdf_integrator_.reset(new SimpleTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else if (method.compare("merged") == 0) {
    integrator_config.enable_anti_grazing = false;
    tsdf_integrator_.reset(new MergedTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else if (method.compare("merged_discard") == 0) {
    integrator_config.enable_anti_grazing = true;
    tsdf_integrator_.reset(new MergedTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else if (method.compare("fast") == 0) {
    tsdf_integrator_.reset(new FastTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else {
    tsdf_integrator_.reset(new SimpleTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  }

  // ESDF settings.
  if (generate_esdf_) {
    EsdfMap::Config esdf_config;
    // TODO(helenol): add possibility for different ESDF map sizes.
    esdf_config.esdf_voxel_size = config.tsdf_voxel_size;
    esdf_config.esdf_voxels_per_side = config.tsdf_voxels_per_side;

    esdf_map_.reset(new EsdfMap(esdf_config));

    EsdfIntegrator::Config esdf_integrator_config;
    // Make sure that this is the same as the truncation distance OR SMALLER!
    esdf_integrator_config.min_distance_m =
        integrator_config.default_truncation_distance;
    nh_private_.param("esdf_max_distance_m",
                      esdf_integrator_config.max_distance_m,
                      esdf_integrator_config.max_distance_m);
    nh_private_.param("esdf_default_distance_m",
                      esdf_integrator_config.default_distance_m,
                      esdf_integrator_config.default_distance_m);
    nh_private_.param("esdf_min_diff_m", esdf_integrator_config.min_diff_m,
                      esdf_integrator_config.min_diff_m);

    esdf_integrator_.reset(new EsdfIntegrator(esdf_integrator_config,
                                              tsdf_map_->getTsdfLayerPtr(),
                                              esdf_map_->getEsdfLayerPtr()));
  }

  // Occupancy settings.
  if (generate_occupancy_) {
    OccupancyMap::Config occupancy_config;
    // TODO(helenol): add possibility for different ESDF map sizes.
    occupancy_config.occupancy_voxel_size = config.tsdf_voxel_size;
    occupancy_config.occupancy_voxels_per_side = config.tsdf_voxels_per_side;
    occupancy_map_.reset(new OccupancyMap(occupancy_config));

    OccupancyIntegrator::Config occupancy_integrator_config;
    occupancy_integrator_.reset(new OccupancyIntegrator(
        occupancy_integrator_config, occupancy_map_->getOccupancyLayerPtr()));
  }

  // Mesh settings.
  nh_private_.param("mesh_filename", mesh_filename_, mesh_filename_);
  std::string color_mode("color");
  nh_private_.param("color_mode", color_mode, color_mode);
  if (color_mode == "color" || color_mode == "colors") {
    color_mode_ = ColorMode::kColor;
  } else if (color_mode == "height") {
    color_mode_ = ColorMode::kHeight;
  } else if (color_mode == "normals") {
    color_mode_ = ColorMode::kNormals;
  } else if (color_mode == "lambert") {
    color_mode_ = ColorMode::kLambert;
  } else {  // Default case is gray.
    color_mode_ = ColorMode::kGray;
  }

  MeshIntegrator<TsdfVoxel>::Config mesh_config;
  nh_private_.param("mesh_min_weight", mesh_config.min_weight,
                    mesh_config.min_weight);

  mesh_layer_.reset(new MeshLayer(tsdf_map_->block_size()));

  mesh_integrator_.reset(new MeshIntegrator<TsdfVoxel>(
      mesh_config, tsdf_map_->getTsdfLayerPtr(), mesh_layer_.get()));

  // Advertise services.
  generate_mesh_srv_ = nh_private_.advertiseService(
      "generate_mesh", &VoxbloxNode::generateMeshCallback, this);
  if (generate_esdf_) {
    generate_esdf_srv_ = nh_private_.advertiseService(
        "generate_esdf", &VoxbloxNode::generateEsdfCallback, this);
  }
  save_map_srv_ = nh_private_.advertiseService(
      "save_map", &VoxbloxNode::saveMapCallback, this);
  load_map_srv_ = nh_private_.advertiseService(
      "load_map", &VoxbloxNode::loadMapCallback, this);

  // If set, use a timer to progressively integrate the mesh.
  double update_mesh_every_n_sec = 0.0;
  nh_private_.param("update_mesh_every_n_sec", update_mesh_every_n_sec,
                    update_mesh_every_n_sec);

  if (update_mesh_every_n_sec > 0.0) {
    update_mesh_timer_ =
        nh_private_.createTimer(ros::Duration(update_mesh_every_n_sec),
                                &VoxbloxNode::updateMeshEvent, this);
  }

  // Transform settings.
  nh_private_.param("use_tf_transforms", use_tf_transforms_,
                    use_tf_transforms_);
  // If we use topic transforms, we have 2 parts: a dynamic transform from a
  // topic and a static transform from parameters.
  // Static transform should be T_G_D (where D is whatever sensor the
  // dynamic coordinate frame is in) and the static should be T_D_C (where
  // C is the sensor frame that produces the depth data). It is possible to
  // specific T_C_D and set invert_static_tranform to true.
  if (!use_tf_transforms_) {
    transform_sub_ =
        nh_.subscribe("transform", 40, &VoxbloxNode::transformCallback, this);
    // Retrieve T_D_C from params.
    XmlRpc::XmlRpcValue T_B_D_xml;
    // TODO(helenol): split out into a function to avoid duplication.
    if (nh_private_.getParam("T_B_D", T_B_D_xml)) {
      kindr::minimal::xmlRpcToKindr(T_B_D_xml, &T_B_D_);

      // See if we need to invert it.
      bool invert_static_tranform = false;
      nh_private_.param("invert_T_B_D", invert_static_tranform,
                        invert_static_tranform);
      if (invert_static_tranform) {
        T_B_D_ = T_B_D_.inverse();
      }
    }
    XmlRpc::XmlRpcValue T_B_C_xml;
    if (nh_private_.getParam("T_B_C", T_B_C_xml)) {
      kindr::minimal::xmlRpcToKindr(T_B_C_xml, &T_B_C_);

      // See if we need to invert it.
      bool invert_static_tranform = false;
      nh_private_.param("invert_T_B_C", invert_static_tranform,
                        invert_static_tranform);
      if (invert_static_tranform) {
        T_B_C_ = T_B_C_.inverse();
      }
    }

    ROS_INFO_STREAM("Static transforms loaded from file.\nT_B_D:\n"
                    << T_B_D_ << "\nT_B_C:" << T_B_C_);
  }

  ros::spinOnce();
}

void VoxbloxNode::processPointCloudMessageAndInsert(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg,
    const bool is_freespace_pointcloud) {
  // Look up transform from sensor frame to world frame.
  Transformation T_G_C;
  if (lookupTransform(pointcloud_msg->header.frame_id, world_frame_,
                      pointcloud_msg->header.stamp, &T_G_C)) {
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

    tsdf_integrator_->integratePointCloud(T_G_C, points_C, colors,
                                          is_freespace_pointcloud);

    if (generate_occupancy_ && !is_freespace_pointcloud) {
      occupancy_integrator_->integratePointCloud(T_G_C, points_C);
    }
    ros::WallTime end = ros::WallTime::now();
    if (verbose_) {
      ROS_INFO("Finished integrating in %f seconds, have %lu blocks.",
               (end - start).toSec(),
               tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks());
      if (generate_occupancy_) {
        ROS_INFO("Occupancy: %lu blocks.",
                 occupancy_map_->getOccupancyLayerPtr()
                     ->getNumberOfAllocatedBlocks());
      }
    }
  }
}

void VoxbloxNode::insertPointcloudWithTf(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg) {
  // Figure out if we should insert this.
  static ros::Time last_msg_time;
  if (pointcloud_msg->header.stamp - last_msg_time < min_time_between_msgs_) {
    return;
  }
  last_msg_time = pointcloud_msg->header.stamp;

  constexpr bool is_freespace_pointcloud = false;
  processPointCloudMessageAndInsert(pointcloud_msg, is_freespace_pointcloud);

  if (publish_tsdf_info_) {
    publishAllUpdatedTsdfVoxels();
    publishTsdfSurfacePoints();
    publishTsdfOccupiedNodes();
  }
  if (generate_occupancy_) {
    publishOccupancy();
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

void VoxbloxNode::insertFreespacePointcloudWithTf(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg) {
  // Figure out if we should insert this.
  static ros::Time last_msg_time;
  if (pointcloud_msg->header.stamp - last_msg_time < min_time_between_msgs_) {
    return;
  }
  last_msg_time = pointcloud_msg->header.stamp;

  constexpr bool is_freespace_pointcloud = true;
  processPointCloudMessageAndInsert(pointcloud_msg, is_freespace_pointcloud);
}

void VoxbloxNode::transformCallback(
    const geometry_msgs::TransformStamped& transform_msg) {
  transform_queue_.push_back(transform_msg);
}

void VoxbloxNode::publishAllUpdatedTsdfVoxels() {
  DCHECK(tsdf_map_) << "TSDF map not allocated.";
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromTsdfLayer(tsdf_map_->getTsdfLayer(), &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  tsdf_pointcloud_pub_.publish(pointcloud);
}

void VoxbloxNode::publishAllUpdatedEsdfVoxels() {
  DCHECK(esdf_map_) << "ESDF map not allocated.";

  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(), &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  esdf_pointcloud_pub_.publish(pointcloud);
}

void VoxbloxNode::publishTsdfSurfacePoints() {
  DCHECK(tsdf_map_) << "TSDF map not allocated.";

  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  const float surface_distance_thresh =
      tsdf_map_->getTsdfLayer().voxel_size() * 0.75;
  createSurfacePointcloudFromTsdfLayer(tsdf_map_->getTsdfLayer(),
                                       surface_distance_thresh, &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  surface_pointcloud_pub_.publish(pointcloud);
}

void VoxbloxNode::publishTsdfOccupiedNodes() {
  DCHECK(tsdf_map_) << "TSDF map not allocated.";

  // Create a pointcloud with distance = intensity.
  visualization_msgs::MarkerArray marker_array;
  createOccupancyBlocksFromTsdfLayer(tsdf_map_->getTsdfLayer(), world_frame_,
                                     &marker_array);
  occupancy_marker_pub_.publish(marker_array);
}

void VoxbloxNode::publishOccupancy() {
  DCHECK(occupancy_map_) << "Occupancy map not allocated.";

  // Create a pointcloud with distance = intensity.
  visualization_msgs::MarkerArray marker_array;
  createOccupancyBlocksFromOccupancyLayer(
      *occupancy_map_->getOccupancyLayerPtr(), world_frame_, &marker_array);
  occupancy_layer_pub_.publish(marker_array);
}

void VoxbloxNode::publishSlices() {
  if (tsdf_map_) {
    pcl::PointCloud<pcl::PointXYZI> pointcloud;

    createDistancePointcloudFromTsdfLayerSlice(tsdf_map_->getTsdfLayer(), 2,
                                               slice_level_, &pointcloud);

    pointcloud.header.frame_id = world_frame_;
    tsdf_slice_pub_.publish(pointcloud);
  }
  if (esdf_map_) {
    pcl::PointCloud<pcl::PointXYZI> pointcloud;

    createDistancePointcloudFromEsdfLayerSlice(esdf_map_->getEsdfLayer(), 2,
                                               slice_level_, &pointcloud);

    pointcloud.header.frame_id = world_frame_;
    esdf_slice_pub_.publish(pointcloud);
  }
}

bool VoxbloxNode::lookupTransform(const std::string& from_frame,
                                  const std::string& to_frame,
                                  const ros::Time& timestamp,
                                  Transformation* transform) {
  if (use_tf_transforms_) {
    return lookupTransformTf(from_frame, to_frame, timestamp, transform);
  } else {
    return lookupTransformQueue(from_frame, to_frame, timestamp, transform);
  }
}

// Stolen from octomap_manager
bool VoxbloxNode::lookupTransformTf(const std::string& from_frame,
                                    const std::string& to_frame,
                                    const ros::Time& timestamp,
                                    Transformation* transform) {
  tf::StampedTransform tf_transform;
  ros::Time time_to_lookup = timestamp;

  // Allow overwriting the TF frame for the sensor.
  std::string from_frame_modified = from_frame;
  if (!sensor_frame_.empty()) {
    from_frame_modified = sensor_frame_;
  }

  // If this transform isn't possible at the time, then try to just look up
  // the latest (this is to work with bag files and static transform
  // publisher, etc).
  if (!tf_listener_.canTransform(to_frame, from_frame_modified,
                                 time_to_lookup)) {
    time_to_lookup = ros::Time(0);
    ROS_WARN("Using latest TF transform instead of timestamp match.");
  }

  try {
    tf_listener_.lookupTransform(to_frame, from_frame_modified, time_to_lookup,
                                 tf_transform);
  } catch (tf::TransformException& ex) {  // NOLINT
    ROS_ERROR_STREAM(
        "Error getting TF transform from sensor data: " << ex.what());
    return false;
  }

  tf::transformTFToKindr(tf_transform, transform);
  return true;
}

bool VoxbloxNode::lookupTransformQueue(const std::string& from_frame,
                                       const std::string& to_frame,
                                       const ros::Time& timestamp,
                                       Transformation* transform) {
  // Try to match the transforms in the queue.
  bool match_found = false;
  AlignedDeque<geometry_msgs::TransformStamped>::iterator it =
      transform_queue_.begin();
  for (; it != transform_queue_.end(); ++it) {
    // If the current transform is newer than the requested timestamp, we need
    // to break.
    if (it->header.stamp > timestamp) {
      if ((it->header.stamp - timestamp).toNSec() < timestamp_tolerance_ns_) {
        match_found = true;
      }
      break;
    }

    if ((timestamp - it->header.stamp).toNSec() < timestamp_tolerance_ns_) {
      match_found = true;
      break;
    }
  }

  if (match_found) {
    Transformation T_G_D;
    tf::transformMsgToKindr(it->transform, &T_G_D);

    // If we have a static transform, apply it too.
    // Transform should actually be T_G_C. So need to take it through the full
    // chain.
    *transform = T_G_D * T_B_D_.inverse() * T_B_C_;

    // And also clear the queue up to this point. This leaves the current
    // message in place.
    transform_queue_.erase(transform_queue_.begin(), it);
  } else {
    ROS_WARN_STREAM_THROTTLE(
        30, "No match found for transform timestamp: " << timestamp);
    if (!transform_queue_.empty()) {
      ROS_WARN_STREAM_THROTTLE(
          30,
          "Queue front: " << transform_queue_.front().header.stamp
                          << " back: " << transform_queue_.back().header.stamp);
    }
  }
  return match_found;
}

bool VoxbloxNode::generateMeshCallback(
    std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& response) {  // NOLINT
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

  if (output_mesh_as_pointcloud_) {
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
    fillPointcloudWithMesh(mesh_layer_, color_mode_, &pointcloud);
    pointcloud.header.frame_id = world_frame_;
    mesh_pointcloud_pub_.publish(pointcloud);
  }
  publish_mesh_timer.Stop();

  if (output_mesh_as_pcl_mesh_) {
    pcl::PolygonMesh polygon_mesh;
    toPCLPolygonMesh(*mesh_layer_, world_frame_, &polygon_mesh);
    pcl_msgs::PolygonMesh mesh_msg;
    pcl_conversions::fromPCL(polygon_mesh, mesh_msg);
    mesh_msg.header.stamp = ros::Time::now();
    mesh_pcl_mesh_pub_.publish(mesh_msg);
  }

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

bool VoxbloxNode::generateEsdfCallback(
    std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& response) {  // NOLINT
  if (!generate_esdf_) {
    return false;
  }
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

bool VoxbloxNode::saveMapCallback(
    voxblox_msgs::FilePath::Request& request,
    voxblox_msgs::FilePath::Response& response) {  // NOLINT
  // Will only save TSDF layer for now.
  return io::SaveLayer(tsdf_map_->getTsdfLayer(), request.file_path);
}

bool VoxbloxNode::loadMapCallback(
    voxblox_msgs::FilePath::Request& request,
    voxblox_msgs::FilePath::Response& response) {  // NOLINT
  // Will only load TSDF layer for now.
  return io::LoadBlocksFromFile(
      request.file_path, Layer<TsdfVoxel>::BlockMergingStrategy::kReplace,
      tsdf_map_->getTsdfLayerPtr());
}

void VoxbloxNode::updateMeshEvent(const ros::TimerEvent& e) {
  if (verbose_) {
    ROS_INFO("Updating mesh.");
  }
  // TODO(helenol): also update the ESDF layer each time you update the mesh.
  if (generate_esdf_) {
    const bool clear_updated_flag_esdf = false;
    esdf_integrator_->updateFromTsdfLayer(clear_updated_flag_esdf);
    publishAllUpdatedEsdfVoxels();
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

  if (output_mesh_as_pointcloud_) {
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
    fillPointcloudWithMesh(mesh_layer_, color_mode_, &pointcloud);
    pointcloud.header.frame_id = world_frame_;
    mesh_pointcloud_pub_.publish(pointcloud);
  }

  publish_mesh_timer.Stop();
}

}  // namespace voxblox

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox_node");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  voxblox::VoxbloxNode node(nh, nh_private);

  ros::spin();
  return 0;
}

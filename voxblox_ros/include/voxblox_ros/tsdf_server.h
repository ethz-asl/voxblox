#ifndef VOXBLOX_ROS_TSDF_SERVER_H_
#define VOXBLOX_ROS_TSDF_SERVER_H_

#include <memory>
#include <queue>
#include <string>

#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <voxblox/alignment/icp.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/utils/color_maps.h>
#include <voxblox_msgs/msg/mesh.hpp>
#include <voxblox_msgs/srv/file_path.hpp>

#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"
#include "voxblox_ros/transformer.h"

namespace voxblox {

constexpr float kDefaultMaxIntensity = 100.0;

class TsdfServer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TsdfServer(rclcpp::Node::SharedPtr node);
  virtual ~TsdfServer() {}

  void getServerConfigFromRosParam();

  void insertPointcloud(
      const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud);

  void insertFreespacePointcloud(
      const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud);

  virtual void processPointCloudMessageAndInsert(
      const sensor_msgs::msg::PointCloud2::SharedPtr& pointcloud_msg,
      const Transformation& T_G_C, const bool is_freespace_pointcloud);

  void integratePointcloud(const Transformation& T_G_C,
                           const Pointcloud& ptcloud_C, const Colors& colors,
                           const bool is_freespace_pointcloud = false);

  virtual void newPoseCallback(const Transformation& /*new_pose*/) {
    // Do nothing.
  }

  void publishAllUpdatedTsdfVoxels();
  void publishTsdfSurfacePoints();
  void publishTsdfOccupiedNodes();

  virtual void publishSlices();
  /// Incremental update.
  virtual void updateMesh();
  /// Batch update.
  virtual bool generateMesh();
  // Publishes all available pointclouds.
  virtual void publishPointclouds();
  // Publishes the complete map
  virtual void publishMap(bool reset_remote_map = false);
  virtual bool saveMap(const std::string& file_path);
  virtual bool loadMap(const std::string& file_path);

  void clearMapCallback(
      std_srvs::srv::Empty::Request::SharedPtr request,     // NOLINT
      std_srvs::srv::Empty::Response::SharedPtr response);  // NOLINT
  void saveMapCallback(
      voxblox_msgs::srv::FilePath::Request::SharedPtr request,     // NOLINT
      voxblox_msgs::srv::FilePath::Response::SharedPtr response);  // NOLINT
  void loadMapCallback(
      voxblox_msgs::srv::FilePath::Request::SharedPtr request,     // NOLINT
      voxblox_msgs::srv::FilePath::Response::SharedPtr response);  // NOLINT
  void generateMeshCallback(
      std_srvs::srv::Empty::Request::SharedPtr request,     // NOLINT
      std_srvs::srv::Empty::Response::SharedPtr response);  // NOLINT
  void publishPointcloudsCallback(
      std_srvs::srv::Empty::Request::SharedPtr request,     // NOLINT
      std_srvs::srv::Empty::Response::SharedPtr response);  // NOLINT
  void publishTsdfMapCallback(
      std_srvs::srv::Empty::Request::SharedPtr request,     // NOLINT
      std_srvs::srv::Empty::Response::SharedPtr response);  // NOLINT

  void updateMeshEvent();
  void publishMapEvent();

  std::shared_ptr<TsdfMap> getTsdfMapPtr() { return tsdf_map_; }
  std::shared_ptr<const TsdfMap> getTsdfMapPtr() const { return tsdf_map_; }

  /// Accessors for setting and getting parameters.
  double getSliceLevel() const { return slice_level_; }
  void setSliceLevel(double slice_level) { slice_level_ = slice_level; }

  bool setPublishSlices() const { return publish_slices_; }
  void setPublishSlices(const bool publish_slices) {
    publish_slices_ = publish_slices;
  }

  void setWorldFrame(const std::string& world_frame) {
    world_frame_ = world_frame;
  }
  std::string getWorldFrame() const { return world_frame_; }

  /// CLEARS THE ENTIRE MAP!
  virtual void clear();

  /// Overwrites the layer with what's coming from the topic!
  void tsdfMapCallback(const voxblox_msgs::msg::Layer::SharedPtr layer_msg);

  inline MeshIntegratorConfig getMeshIntegratorConfigFromRosParam() {
    MeshIntegratorConfig mesh_integrator_config;
    mesh_integrator_config.min_weight = node_->declare_parameter(
        "mesh_min_weight", mesh_integrator_config.min_weight);
    mesh_integrator_config.use_color = node_->declare_parameter(
        "mesh_use_color", mesh_integrator_config.use_color);
    return mesh_integrator_config;
  }

  inline TsdfIntegratorBase::Config getTsdfIntegratorConfigFromRosParam(
      const voxblox::TsdfMap::Config& tsdf_config) {
    TsdfIntegratorBase::Config integrator_config;

    integrator_config.voxel_carving_enabled = true;
    integrator_config.default_truncation_distance =
        tsdf_config.tsdf_voxel_size * 4;

    integrator_config.voxel_carving_enabled = node_->declare_parameter(
        "voxel_carving_enabled", integrator_config.voxel_carving_enabled);
    integrator_config.max_ray_length_m = node_->declare_parameter(
        "max_ray_length_m", integrator_config.max_ray_length_m);
    integrator_config.min_ray_length_m = node_->declare_parameter(
        "min_ray_length_m", integrator_config.min_ray_length_m);
    integrator_config.use_const_weight = node_->declare_parameter(
        "use_const_weight", integrator_config.use_const_weight);
    integrator_config.use_weight_dropoff = node_->declare_parameter(
        "use_weight_dropoff", integrator_config.use_weight_dropoff);
    integrator_config.allow_clear =
        node_->declare_parameter("allow_clear", integrator_config.allow_clear);
    integrator_config.start_voxel_subsampling_factor = node_->declare_parameter(
        "start_voxel_subsampling_factor",
        integrator_config.start_voxel_subsampling_factor);
    integrator_config.max_consecutive_ray_collisions = node_->declare_parameter(
        "max_consecutive_ray_collisions",
        integrator_config.max_consecutive_ray_collisions);
    integrator_config.clear_checks_every_n_frames =
        node_->declare_parameter("clear_checks_every_n_frames",
                                 integrator_config.clear_checks_every_n_frames);
    integrator_config.max_integration_time_s = node_->declare_parameter(
        "max_integration_time_s", integrator_config.max_integration_time_s);
    integrator_config.enable_anti_grazing = node_->declare_parameter(
        "anti_grazing", integrator_config.enable_anti_grazing);
    integrator_config.use_sparsity_compensation_factor =
        node_->declare_parameter(
            "use_sparsity_compensation_factor",
            integrator_config.use_sparsity_compensation_factor);
    integrator_config.sparsity_compensation_factor = node_->declare_parameter(
        "sparsity_compensation_factor",
        integrator_config.sparsity_compensation_factor);
    integrator_config.integration_order_mode = node_->declare_parameter(
        "integration_order_mode", integrator_config.integration_order_mode);

    double truncation_distance = integrator_config.default_truncation_distance;
    truncation_distance =
        node_->declare_parameter("truncation_distance", truncation_distance);
    integrator_config.default_truncation_distance =
        static_cast<float>(truncation_distance);

    double max_weight = integrator_config.max_weight;
    max_weight = node_->declare_parameter("max_weight", max_weight);
    integrator_config.max_weight = static_cast<float>(max_weight);

    return integrator_config;
  }

  inline ICP::Config getICPConfigFromRosParam() {
    ICP::Config icp_config;

    icp_config.min_match_ratio = node_->declare_parameter(
        "icp_min_match_ratio", icp_config.min_match_ratio);
    icp_config.subsample_keep_ratio = node_->declare_parameter(
        "icp_subsample_keep_ratio", icp_config.subsample_keep_ratio);
    icp_config.mini_batch_size = node_->declare_parameter(
        "icp_mini_batch_size", icp_config.mini_batch_size);
    icp_config.refine_roll_pitch = node_->declare_parameter(
        "icp_refine_roll_pitch", icp_config.refine_roll_pitch);
    icp_config.inital_translation_weighting =
        node_->declare_parameter("icp_inital_translation_weighting",
                                 icp_config.inital_translation_weighting);
    icp_config.inital_rotation_weighting = node_->declare_parameter(
        "icp_inital_rotation_weighting", icp_config.inital_rotation_weighting);

    return icp_config;
  }

  inline TsdfMap::Config getTsdfMapConfigFromRosParam() {
    TsdfMap::Config tsdf_config;

    /**
     * Workaround for OS X on mac mini not having specializations for float
     * for some reason.
     */
    double voxel_size = tsdf_config.tsdf_voxel_size;
    int voxels_per_side = tsdf_config.tsdf_voxels_per_side;
    voxel_size = node_->declare_parameter("tsdf_voxel_size", voxel_size);
    voxels_per_side =
        node_->declare_parameter("tsdf_voxels_per_side", voxels_per_side);
    if (!isPowerOfTwo(voxels_per_side)) {
      RCLCPP_ERROR(
          node_->get_logger(),
          "voxels_per_side must be a power of 2, setting to default value");
      voxels_per_side = tsdf_config.tsdf_voxels_per_side;
    }

    tsdf_config.tsdf_voxel_size = static_cast<FloatingPoint>(voxel_size);
    tsdf_config.tsdf_voxels_per_side = voxels_per_side;

    return tsdf_config;
  }

 protected:
  /**
   * Gets the next pointcloud that has an available transform to process from
   * the queue.
   */
  bool getNextPointcloudFromQueue(
      std::queue<sensor_msgs::msg::PointCloud2::SharedPtr>* queue,
      sensor_msgs::msg::PointCloud2::SharedPtr* pointcloud_msg,
      Transformation* T_G_C);

  /// Data subscribers.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      freespace_pointcloud_sub_;

  /// Publish markers for visualization.
  rclcpp::Publisher<voxblox_msgs::msg::Mesh>::SharedPtr mesh_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      tsdf_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      surface_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tsdf_slice_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      occupancy_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr
      icp_transform_pub_;

  /// Publish the complete map for other nodes to consume.
  rclcpp::Publisher<voxblox_msgs::msg::Layer>::SharedPtr tsdf_map_pub_;

  /// Subscriber to subscribe to another node generating the map.
  rclcpp::Subscription<voxblox_msgs::msg::Layer>::SharedPtr tsdf_map_sub_;

  // Services.
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr generate_mesh_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_map_srv_;
  rclcpp::Service<voxblox_msgs::srv::FilePath>::SharedPtr save_map_srv_;
  rclcpp::Service<voxblox_msgs::srv::FilePath>::SharedPtr load_map_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr publish_pointclouds_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr publish_tsdf_map_srv_;

  /// Tools for broadcasting TFs.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Timers.
  rclcpp::TimerBase::SharedPtr update_mesh_timer_;
  rclcpp::TimerBase::SharedPtr publish_map_timer_;

  rclcpp::Node::SharedPtr node_;

  /**
   * Transformer object to keep track of either TF transforms or messages from
   * a transform topic.
   */
  Transformer transformer_;

  bool verbose_;

  /**
   * Global/map coordinate frame. Will always look up TF transforms to this
   * frame.
   */
  std::string world_frame_;
  /**
   * Name of the ICP corrected frame. Publishes TF and transform topic to this
   * if ICP on.
   */
  std::string icp_corrected_frame_;
  /// Name of the pose in the ICP correct Frame.
  std::string pose_corrected_frame_;

  /// Delete blocks that are far from the system to help manage memory
  double max_block_distance_from_body_;

  /// Pointcloud visualization settings.
  double slice_level_;

  /// If the system should subscribe to a pointcloud giving points in freespace
  bool use_freespace_pointcloud_;

  /**
   * Mesh output settings. Mesh is only written to file if mesh_filename_ is
   * not empty.
   */
  std::string mesh_filename_;
  /// How to color the mesh.
  ColorMode color_mode_;

  /// Colormap to use for intensity pointclouds.
  std::shared_ptr<ColorMap> color_map_;

  /// Will throttle to this message rate.
  double min_time_between_msgs_sec_;
  rclcpp::Duration min_time_between_msgs_ =
      rclcpp::Duration::from_seconds(min_time_between_msgs_sec_);

  /// What output information to publish
  bool publish_pointclouds_on_update_;
  bool publish_slices_;
  bool publish_pointclouds_;
  bool publish_tsdf_map_;

  /// Whether to save the latest mesh message sent (for inheriting classes).
  bool cache_mesh_;

  /**
   *Whether to enable ICP corrections. Every pointcloud coming in will attempt
   * to be matched up to the existing structure using ICP. Requires the initial
   * guess from odometry to already be very good.
   */
  bool enable_icp_;
  /**
   * If using ICP corrections, whether to store accumulate the corrected
   * transform. If this is set to false, the transform will reset every
   * iteration.
   */
  bool accumulate_icp_corrections_;

  /// Subscriber settings.
  int pointcloud_queue_size_;
  int num_subscribers_tsdf_map_;

  // Maps and integrators.
  std::shared_ptr<TsdfMap> tsdf_map_;
  std::unique_ptr<TsdfIntegratorBase> tsdf_integrator_;

  /// ICP matcher
  std::shared_ptr<ICP> icp_;

  // Mesh accessories.
  std::shared_ptr<MeshLayer> mesh_layer_;
  std::unique_ptr<MeshIntegrator<TsdfVoxel>> mesh_integrator_;
  /// Optionally cached mesh message.
  voxblox_msgs::msg::Mesh cached_mesh_msg_;

  /**
   * Queue of incoming pointclouds, in case the transforms can't be immediately
   * resolved.
   */
  std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointcloud_queue_;
  std::queue<sensor_msgs::msg::PointCloud2::SharedPtr>
      freespace_pointcloud_queue_;

  // Last message times for throttling input.
  rclcpp::Time last_msg_time_ptcloud_;
  rclcpp::Time last_msg_time_freespace_ptcloud_;

  /// Current transform corrections from ICP.
  Transformation icp_corrected_transform_;

  TsdfIntegratorBase::Config
      tsdf_integrator_config;  // TODO: make these const and init in ctor header
  TsdfMap::Config
      tsdf_config;  // TODO: make these const and init in ctor header
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_TSDF_SERVER_H_

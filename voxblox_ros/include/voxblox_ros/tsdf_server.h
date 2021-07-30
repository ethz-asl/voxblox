#ifndef VOXBLOX_ROS_TSDF_SERVER_H_
#define VOXBLOX_ROS_TSDF_SERVER_H_

#include <deque>
#include <memory>
#include <queue>
#include <string>

#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <voxblox/alignment/icp.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/utils/color_maps.h>
#include <voxblox_msgs/FilePath.h>
#include <voxblox_msgs/Mesh.h>

#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"
#include "voxblox_ros/threshold.h"
#include "voxblox_ros/transformer.h"

namespace voxblox {

constexpr float kDefaultMaxIntensity = 100.0;

class TsdfServer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TsdfServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  TsdfServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
             const TsdfMap::Config& config,
             const TsdfIntegratorBase::Config& integrator_config,
             const MeshIntegratorConfig& mesh_config);
  virtual ~TsdfServer() {}

  void getServerConfigFromRosParam(const ros::NodeHandle& nh_private);

  void insertPointcloud(const sensor_msgs::PointCloud2::Ptr& pointcloud);

  void insertFreespacePointcloud(
      const sensor_msgs::PointCloud2::Ptr& pointcloud);

  virtual void processPointCloudMessageAndInsert(
      const sensor_msgs::PointCloud2::Ptr& pointcloud_msg,
      const Transformation& T_G_C, const bool is_freespace_pointcloud);

  void integratePointcloud(const ros::Time& timestamp,
                           const Transformation& T_G_C,
                           std::shared_ptr<const Pointcloud> ptcloud_C,
                           std::shared_ptr<const Colors> colors,
                           const bool is_freespace_pointcloud = false);

  // Note(schmluk): Provide the legacy interface for voxblox-specific packages.
  void integratePointcloud(const Transformation& T_G_C,
                           const Pointcloud& ptcloud_C, const Colors& colors,
                           const bool is_freespace_pointcloud = false);

  void servicePointcloudDeintegrationQueue();

  virtual void newPoseCallback(const Transformation& T_G_C) {
    if (slice_level_follow_robot_) {
      slice_level_ = T_G_C.getPosition().z();
    }
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
  virtual void publishSubmap();
  virtual bool saveMap(const std::string& file_path);
  virtual bool loadMap(const std::string& file_path);

  bool clearMapCallback(std_srvs::Empty::Request& request,           // NOLINT
                        std_srvs::Empty::Response& response);        // NOLINT
  bool saveMapCallback(voxblox_msgs::FilePath::Request& request,     // NOLINT
                       voxblox_msgs::FilePath::Response& response);  // NOLINT
  bool loadMapCallback(voxblox_msgs::FilePath::Request& request,     // NOLINT
                       voxblox_msgs::FilePath::Response& response);  // NOLINT
  bool generateMeshCallback(std_srvs::Empty::Request& request,       // NOLINT
                            std_srvs::Empty::Response& response);    // NOLINT
  bool publishPointcloudsCallback(
      std_srvs::Empty::Request& request,                             // NOLINT
      std_srvs::Empty::Response& response);                          // NOLINT
  bool publishTsdfMapCallback(std_srvs::Empty::Request& request,     // NOLINT
                              std_srvs::Empty::Response& response);  // NOLINT

  void updateMeshEvent(const ros::TimerEvent& event);
  void publishMapEvent(const ros::TimerEvent& event);

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
  void tsdfMapCallback(const voxblox_msgs::Layer& layer_msg);

 protected:
  /**
   * Gets the next pointcloud that has an available transform to process from
   * the queue.
   */
  bool getNextPointcloudFromQueue(
      std::queue<sensor_msgs::PointCloud2::Ptr>* queue,
      sensor_msgs::PointCloud2::Ptr* pointcloud_msg, Transformation* T_G_C);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  /// Data subscribers.
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber freespace_pointcloud_sub_;

  /// Publish markers for visualization.
  ros::Publisher mesh_pub_;
  ros::Publisher tsdf_pointcloud_pub_;
  ros::Publisher surface_pointcloud_pub_;
  ros::Publisher tsdf_slice_pub_;
  ros::Publisher occupancy_marker_pub_;
  ros::Publisher icp_transform_pub_;
  ros::Publisher reprojected_pointcloud_pub_;

  /// Publish the complete map for other nodes to consume.
  ros::Publisher tsdf_map_pub_;
  ros::Publisher submap_pub_;
  ros::Publisher new_submap_notification_pub_;

  /// Subscriber to subscribe to another node generating the map.
  ros::Subscriber tsdf_map_sub_;

  // Services.
  ros::ServiceServer generate_mesh_srv_;
  ros::ServiceServer clear_map_srv_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer load_map_srv_;
  ros::ServiceServer publish_pointclouds_srv_;
  ros::ServiceServer publish_tsdf_map_srv_;

  /// Tools for broadcasting TFs.
  tf::TransformBroadcaster tf_broadcaster_;

  // Timers.
  ros::Timer update_mesh_timer_;
  ros::Timer publish_map_timer_;
  ros::Timer publish_submap_timer_;

  bool verbose_;

  /**
   * Global/map coordinate frame. Will always look up TF transforms to this
   * frame.
   */
  std::string world_frame_;
  std::string robot_name_;
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
  bool slice_level_follow_robot_;

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
  ros::Duration min_time_between_msgs_;

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
  TsdfIntegratorBase::Ptr tsdf_integrator_;

  /// ICP matcher
  std::shared_ptr<ICP> icp_;

  // Mesh accessories.
  std::shared_ptr<MeshLayer> mesh_layer_;
  std::unique_ptr<MeshIntegrator<TsdfVoxel>> mesh_integrator_;
  /// Optionally cached mesh message.
  voxblox_msgs::Mesh cached_mesh_msg_;

  /**
   * Transformer object to keep track of either TF transforms or messages from
   * a transform topic.
   */
  Transformer transformer_;
  /**
   * Queue of incoming pointclouds, in case the transforms can't be immediately
   * resolved.
   */
  std::queue<sensor_msgs::PointCloud2::Ptr> pointcloud_queue_;
  std::queue<sensor_msgs::PointCloud2::Ptr> freespace_pointcloud_queue_;

  // Optionally publish the map as submaps for use in submap-based
  // SLAM and/or path planning systems such as cblox, voxgraph, or GLocal
  Threshold<FloatingPoint> submap_max_time_interval_;
  Threshold<FloatingPoint> submap_max_distance_travelled_;
  ros::Time last_published_submap_timestamp_;
  Point last_published_submap_position_;
  bool submappingEnabled() const {
    return submap_max_time_interval_.isSet() ||
           submap_max_distance_travelled_.isSet();
  }
  bool shouldCreateNewSubmap(const ros::Time& current_timestamp,
                             const Transformation& current_T_G_C);
  void createNewSubmap(const ros::Time& current_timestamp,
                       const Transformation& current_T_G_C);
  int published_submap_counter_;
  bool write_submaps_to_disk_;
  std::string submap_root_directory_;

  // Optionally build and incrementally update a sliding window local map by
  // not only integrating pointclouds but also removing them again once their
  // age and/or distance to the robot exceeds a given threshold.
  // NOTE: This can also be used to create submap collections with arbitrary
  //       amounts of overlap, while only having to integrating and
  //       deintegrating each pointcloud once.
  struct PointcloudDeintegrationPacket {
    const ros::Time timestamp;
    const Transformation T_G_C;
    std::shared_ptr<const Pointcloud> ptcloud_C;
    std::shared_ptr<const Colors> colors;
    const bool is_freespace_pointcloud;
  };
  std::deque<PointcloudDeintegrationPacket> pointcloud_deintegration_queue_;
  Threshold<size_t> pointcloud_deintegration_max_queue_length_;
  Threshold<FloatingPoint> pointcloud_deintegration_max_time_interval_;
  Threshold<FloatingPoint> pointcloud_deintegration_max_distance_travelled_;
  bool pointcloudDeintegrationEnabled() const {
    return (pointcloud_deintegration_max_queue_length_.isSet() ||
            pointcloud_deintegration_max_time_interval_.isSet() ||
            pointcloud_deintegration_max_distance_travelled_.isSet());
  }

  // Remove blocks that no longer contain any observed voxels
  const size_t num_voxels_per_block_;
  bool map_needs_pruning_;
  virtual void pruneMap();

  // Last message times for throttling input.
  ros::Time last_msg_time_ptcloud_;
  ros::Time last_msg_time_freespace_ptcloud_;

  /// Current transform corrections from ICP.
  Transformation icp_corrected_transform_;

  // File system helpers
  bool hasOnlyAsciiCharacters(const std::string& string_to_test);
  bool createPath(const std::string& path_to_create_input);
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_TSDF_SERVER_H_

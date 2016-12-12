#ifndef VOXBLOX_ROS_TSDF_SERVER_H_
#define VOXBLOX_ROS_TSDF_SERVER_H_

#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>

#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>

#include "voxblox_ros/transformer.h"
#include "voxblox_ros/FilePath.h"
#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"

namespace voxblox {

class TsdfServer {
 public:
  // Merging method for new pointclouds.
  enum Method { kSimple = 0, kMerged, kMergedDiscard };

  TsdfServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~TsdfServer() {}

  void insertPointcloud(const sensor_msgs::PointCloud2::Ptr& pointcloud);

  void publishAllUpdatedTsdfVoxels();
  void publishTsdfSurfacePoints();
  void publishTsdfOccupiedNodes();
  void publishSlices();

  bool saveMapCallback(voxblox_ros::FilePath::Request& request,     // NOLINT
                       voxblox_ros::FilePath::Response& response);  // NOLINT
  bool loadMapCallback(voxblox_ros::FilePath::Request& request,     // NOLINT
                       voxblox_ros::FilePath::Response& response);  // NOLINT

  void updateMeshEvent(const ros::TimerEvent& e);

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  bool verbose_;

    // Merging method for new pointclouds, chosen from enum above.
  Method method_;

  // Global/map coordinate frame. Will always look up TF transforms to this
  // frame.
  std::string world_frame_;

    // Pointcloud visualization settings.
  double slice_level_;

  // Mesh output settings. Mesh is only written to file if mesh_filename_ is
  // not empty.
  std::string mesh_filename_;
  // How to color the mesh.
  ColorMode color_mode_;

  // Keep track of these for throttling.
  ros::Duration min_time_between_msgs_;
  ros::Time last_msg_time_;

  // Data subscribers.
  ros::Subscriber pointcloud_sub_;

  // Publish markers for visualization.
  ros::Publisher mesh_pub_;
  ros::Publisher tsdf_pointcloud_pub_;
  ros::Publisher surface_pointcloud_pub_;
  ros::Publisher tsdf_slice_pub_;

  // Services.
  ros::ServiceServer generate_mesh_srv_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer load_map_srv_;

  // Timers.
  ros::Timer update_mesh_timer_;

  // Transformer object to keep track of either TF transforms or messages from
  // a transform topic.
  Transformer transformer_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_TSDF_SERVER_H_

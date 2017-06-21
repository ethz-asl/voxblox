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
#include <string>
#include <visualization_msgs/MarkerArray.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>

#include <voxblox_msgs/FilePath.h>
#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"
#include "voxblox_ros/transformer.h"

namespace voxblox {

class TsdfServer {
 public:
  // Merging method for new pointclouds.
  enum Method { kSimple = 0, kMerged, kMergedDiscard };

  TsdfServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~TsdfServer() {}

  virtual void insertPointcloud(
      const sensor_msgs::PointCloud2::Ptr& pointcloud);

  void integratePointcloud(const Transformation& T_G_C,
                           const Pointcloud& ptcloud_C, const Colors& colors);
  virtual void newPoseCallback(const Transformation& new_pose) {}

  void publishAllUpdatedTsdfVoxels();
  void publishTsdfSurfacePoints();
  void publishTsdfOccupiedNodes();

  virtual void publishSlices();
  virtual void updateMesh();    // Incremental update.
  virtual bool generateMesh();  // Batch update.

  bool saveMapCallback(voxblox_msgs::FilePath::Request& request,     // NOLINT
                       voxblox_msgs::FilePath::Response& response);  // NOLINT
  bool loadMapCallback(voxblox_msgs::FilePath::Request& request,     // NOLINT
                       voxblox_msgs::FilePath::Response& response);  // NOLINT
  bool generateMeshCallback(std_srvs::Empty::Request& request,       // NOLINT
                            std_srvs::Empty::Response& response);    // NOLINT

  void updateMeshEvent(const ros::TimerEvent& event);

  std::shared_ptr<TsdfMap> getTsdfMapPtr() { return tsdf_map_; }

  // Accessors for setting and getting parameters.
  double getSliceLevel() const { return slice_level_; }
  void setSliceLevel(double slice_level) { slice_level_ = slice_level; }

  // CLEARS THE ENTIRE MAP!
  virtual void clear();

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
  ros::Publisher occupancy_marker_pub_;

  // Services.
  ros::ServiceServer generate_mesh_srv_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer load_map_srv_;

  // Timers.
  ros::Timer update_mesh_timer_;

  // Maps and integrators.
  std::shared_ptr<TsdfMap> tsdf_map_;
  std::unique_ptr<TsdfIntegrator> tsdf_integrator_;

  // Mesh accessories.
  std::shared_ptr<MeshLayer> mesh_layer_;
  std::unique_ptr<MeshIntegrator> mesh_integrator_;

  // Transformer object to keep track of either TF transforms or messages from
  // a transform topic.
  Transformer transformer_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_TSDF_SERVER_H_

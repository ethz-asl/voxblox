#ifndef VOXBLOX_ROS_SIMPLE_TSDF_VISUALIZER_H_
#define VOXBLOX_ROS_SIMPLE_TSDF_VISUALIZER_H_

#include <string>

#include <ros/ros.h>
#include <voxblox/core/layer.h>

#include "voxblox_ros/mesh_vis.h"

namespace voxblox {

class SimpleTsdfVisualizer {
 public:
  explicit SimpleTsdfVisualizer(const ros::NodeHandle &nh_private);

  void run(const Layer<TsdfVoxel> &tsdf_layer);

 private:
  ros::NodeHandle nh_private_;

  ros::Publisher surface_pointcloud_pub_;
  ros::Publisher tsdf_pointcloud_pub_;
  ros::Publisher mesh_pub_;
  ros::Publisher mesh_pointcloud_pub_;
  ros::Publisher mesh_pcl_mesh_pub_;

  // Settings
  double tsdf_surface_distance_threshold_factor_;
  std::string tsdf_world_frame_;
  ColorMode tsdf_mesh_color_mode_;
  std::string tsdf_voxel_ply_output_path_;
  std::string tsdf_mesh_output_path_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_SIMPLE_TSDF_VISUALIZER_H_

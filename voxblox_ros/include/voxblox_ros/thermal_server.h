#ifndef VOXBLOX_ROS_THERMAL_SERVER_H_
#define VOXBLOX_ROS_THERMAL_SERVER_H_

#include <memory>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <voxblox/core/voxel.h>
#include <voxblox/integrator/thermal_integrator.h>

#include "voxblox_ros/tsdf_server.h"

namespace voxblox {

class ThermalServer : public TsdfServer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ThermalServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~ThermalServer() {}

  // virtual void updateMesh() {}
  virtual void publishPointclouds();

  void thermalImageCallback(const sensor_msgs::ImageConstPtr& image);

 protected:
  // Subscriber for thermal images.
  ros::Subscriber thermal_image_sub_;

  // Publish markers for visualization.
  ros::Publisher thermal_pointcloud_pub_;
  ros::Publisher thermal_mesh_pub_;

  // Parameters of the incoming UNDISTORTED thermal images.
  double focal_length_px_;

  // Thermal layer, integrator, and color maps, all related to storing
  // and visualizing thermal data.
  std::shared_ptr<Layer<ThermalVoxel>> thermal_layer_;
  std::unique_ptr<ThermalIntegrator> thermal_integrator_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_THERMAL_SERVER_H_

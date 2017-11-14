#ifndef VOXBLOX_ROS_ESDF_SERVER_H_
#define VOXBLOX_ROS_ESDF_SERVER_H_

#include <memory>

#include <voxblox/core/esdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox_msgs/Layer.h>

#include "voxblox_ros/tsdf_server.h"

namespace voxblox {

class EsdfServer : public TsdfServer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EsdfServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  EsdfServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
             const EsdfMap::Config& esdf_config,
             const EsdfIntegrator::Config& esdf_integrator_config,
             const TsdfMap::Config& tsdf_config,
             const TsdfIntegratorBase::Config& tsdf_integrator_config);
  virtual ~EsdfServer() {}

  void publishAllUpdatedEsdfVoxels();
  virtual void publishSlices();

  bool generateEsdfCallback(std_srvs::Empty::Request& request,     // NOLINT
                            std_srvs::Empty::Response& response);  // NOLINT

  virtual void updateMesh();
  virtual void newPoseCallback(const Transformation& T_G_C);

  // Call updateMesh if you want everything updated; call this specifically
  // if you don't want the mesh or visualization.
  void updateEsdf();

  void esdfMapCallback(const voxblox_msgs::Layer& layer_msg);

  std::shared_ptr<EsdfMap> getEsdfMapPtr() { return esdf_map_; }

  bool getClearSphere() const { return clear_sphere_for_planning_; }
  void setClearSphere(bool clear_sphere_for_planning) {
    clear_sphere_for_planning_ = clear_sphere_for_planning;
  }

  virtual void clear();

 protected:
  // Publish markers for visualization.
  ros::Publisher esdf_pointcloud_pub_;
  ros::Publisher esdf_slice_pub_;

  // Publish the complete map for other nodes to consume.
  ros::Publisher esdf_map_pub_;

  // Subscriber to subscribe to another node generating the map.
  ros::Subscriber esdf_map_sub_;

  // Services.
  ros::ServiceServer generate_esdf_srv_;

  bool clear_sphere_for_planning_;

  // ESDF maps.
  std::shared_ptr<EsdfMap> esdf_map_;
  std::unique_ptr<EsdfIntegrator> esdf_integrator_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_ESDF_SERVER_H_

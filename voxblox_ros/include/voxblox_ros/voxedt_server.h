#ifndef VOXBLOX_ROS_VOXFIELD_SERVER_H_
#define VOXBLOX_ROS_VOXFIELD_SERVER_H_

#include <memory>
#include <string>

#include <voxblox/core/esdf_map.h>
#include <voxblox/core/occupancy_map.h>
#include <voxblox/integrator/esdf_occ_fiesta_integrator.h>
#include <voxblox/integrator/esdf_occ_edt_integrator.h>
#include <voxblox/integrator/occupancy_tsdf_integrator.h>
#include <voxblox_msgs/Layer.h>

#include "voxblox_ros/tsdf_server.h"

namespace voxblox {

class VoxedtServer : public TsdfServer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VoxedtServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  VoxedtServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                 const EsdfMap::Config& esdf_config,
                 //const EsdfOccFiestaIntegrator::Config& esdf_integrator_config,
                 const EsdfOccEdtIntegrator::Config& esdf_integrator_config,
                 const TsdfMap::Config& tsdf_config,
                 const TsdfIntegratorBase::Config& tsdf_integrator_config,
                 const OccupancyMap::Config& occ_config,
                 const OccTsdfIntegrator::Config& occ_tsdf_integrator_config,
                 const MeshIntegratorConfig& mesh_config);
  virtual ~VoxedtServer() {}

  void publishAllUpdatedEsdfVoxels();
  virtual void publishSlices();
  void visualizeEsdfError();
  void publishTraversable();
  void publishOccupancyOccupiedNodes();

  virtual void publishPointclouds();
  virtual void publishMap(bool reset_remote_map = false);
  virtual bool saveTsdfMap(const std::string& file_path);
  virtual bool saveEsdfMap(const std::string& file_path);
  virtual bool saveOccMap(const std::string& file_path);
  virtual bool saveAllMap(const std::string& file_path);
  virtual bool loadMap(const std::string& file_path);

  /// Timer events
  void updateEsdfEvent(const ros::TimerEvent& event);

  void evalEsdfEvent(const ros::TimerEvent& event);

  /// Call this to update the ESDF based on latest state of the occupancy map,
  /// considering only the newly updated parts of the occupancy map (checked
  /// with the ESDF updated bit in Update::Status).
  void updateEsdfFromOcc();

  /// Update the ESDF all at once; clear the existing map.
  // void updateEsdfBatch(bool full_euclidean = false);

  /// Call this to update the Occupancy map based on latest state of the TSDF
  /// map
  void updateOccFromTsdf();

  void evalEsdfRefOcc();

  // Overwrites the layer with what's coming from the topic!
  void esdfMapCallback(const voxblox_msgs::Layer& layer_msg);

  inline std::shared_ptr<EsdfMap> getEsdfMapPtr() { return esdf_map_; }
  inline std::shared_ptr<const EsdfMap> getEsdfMapPtr() const {
    return esdf_map_;
  }

  bool saveEsdfMapCallback(
      voxblox_msgs::FilePath::Request& request,     // NOLINT
      voxblox_msgs::FilePath::Response& response);  // NOLINT

  bool saveOccMapCallback(
      voxblox_msgs::FilePath::Request& request,     // NOLINT
      voxblox_msgs::FilePath::Response& response);  // NOLINT

  bool saveAllMapCallback(
      voxblox_msgs::FilePath::Request& request,     // NOLINT
      voxblox_msgs::FilePath::Response& response);  // NOLINT

  bool getClearSphere() const { return clear_sphere_for_planning_; }
  void setClearSphere(bool clear_sphere_for_planning) {
    clear_sphere_for_planning_ = clear_sphere_for_planning;
  }

  float getEsdfMaxDistance() const;
  void setEsdfMaxDistance(float max_distance);

  float getTraversabilityRadius() const;
  void setTraversabilityRadius(float traversability_radius);

  /**
   * These are for enabling or disabling incremental update of the ESDF. Use
   * carefully.
   */
  void disableIncrementalUpdate() { incremental_update_ = false; }
  void enableIncrementalUpdate() { incremental_update_ = true; }

  virtual void clear();

 protected:
  /// Sets up publishing and subscribing. Should only be called from
  /// constructor.
  void setupRos();

  /// Publish markers for visualization.
  ros::Publisher esdf_pointcloud_pub_;
  ros::Publisher esdf_slice_pub_;
  ros::Publisher esdf_error_slice_pub_;
  ros::Publisher traversable_pub_;

  /// Publish the complete map for other nodes to consume.
  ros::Publisher esdf_map_pub_;

  /// Subscriber to subscribe to another node generating the map.
  ros::Subscriber esdf_map_sub_;

  /// Services.
  ros::ServiceServer generate_esdf_srv_;
  ros::ServiceServer save_esdf_map_srv_;
  ros::ServiceServer save_occ_map_srv_;
  ros::ServiceServer save_all_map_srv_;

  /// Timers.
  ros::Timer update_esdf_timer_;
  ros::Timer eval_esdf_timer_;

  bool clear_sphere_for_planning_;
  bool publish_esdf_map_;
  bool publish_traversable_;
  float traversability_radius_;
  bool incremental_update_;
  int num_subscribers_esdf_map_;
  bool esdf_ready_;

  // ESDF maps.
  std::shared_ptr<EsdfMap> esdf_map_;
  //std::unique_ptr<EsdfOccFiestaIntegrator> esdf_integrator_;
  std::unique_ptr<EsdfOccEdtIntegrator> esdf_integrator_;

  // Occupancy maps.
  std::shared_ptr<OccupancyMap> occupancy_map_;
  std::unique_ptr<OccTsdfIntegrator> occupancy_integrator_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_VOXFIELD_SERVER_H_

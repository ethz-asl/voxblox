#ifndef VOXBLOX_ROS_ESDF_SERVER_H_
#define VOXBLOX_ROS_ESDF_SERVER_H_

#include <memory>
#include <string>

#include <voxblox/core/esdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox_msgs/msg/layer.hpp>

#include "voxblox_ros/tsdf_server.h"

namespace voxblox {

class EsdfServer : public TsdfServer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EsdfServer();
  virtual ~EsdfServer() {}

  bool generateEsdfCallback(
      std_srvs::srv::Empty::Request::SharedPtr request,     // NOLINT
      std_srvs::srv::Empty::Response::SharedPtr response);  // NOLINT

  void publishAllUpdatedEsdfVoxels();
  virtual void publishSlices();
  void publishTraversable();

  virtual void publishPointclouds();
  virtual void newPoseCallback(const Transformation& T_G_C);
  virtual void publishMap(bool reset_remote_map = false);
  virtual bool saveMap(const std::string& file_path);
  virtual bool loadMap(const std::string& file_path);

  void updateEsdfEvent();

  /// Call this to update the ESDF based on latest state of the TSDF map,
  /// considering only the newly updated parts of the TSDF map (checked with
  /// the ESDF updated bit in Update::Status).
  void updateEsdf();
  /// Update the ESDF all at once; clear the existing map.
  void updateEsdfBatch(bool full_euclidean = false);

  // Overwrites the layer with what's coming from the topic!
  void esdfMapCallback(const voxblox_msgs::msg::Layer::SharedPtr layer_msg);

  inline std::shared_ptr<EsdfMap> getEsdfMapPtr() { return esdf_map_; }
  inline std::shared_ptr<const EsdfMap> getEsdfMapPtr() const {
    return esdf_map_;
  }

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

  inline EsdfMap::Config getEsdfMapConfigFromRosParam() {
    EsdfMap::Config esdf_config;
    esdf_config.esdf_voxel_size = this->tsdf_config.tsdf_voxel_size;
    esdf_config.esdf_voxels_per_side = this->tsdf_config.tsdf_voxels_per_side;
    return esdf_config;
  }

  inline EsdfIntegrator::Config getEsdfIntegratorConfigFromRosParam() {
    EsdfIntegrator::Config esdf_integrator_config;
    esdf_integrator_config.min_distance_m =
        this->tsdf_integrator_config.default_truncation_distance / 2.0;

    esdf_integrator_config.full_euclidean_distance =
        this->declare_parameter("esdf_euclidean_distance",
                                esdf_integrator_config.full_euclidean_distance);
    esdf_integrator_config.max_distance_m = this->declare_parameter(
        "esdf_max_distance_m", esdf_integrator_config.max_distance_m);
    esdf_integrator_config.min_distance_m = this->declare_parameter(
        "esdf_min_distance_m", esdf_integrator_config.min_distance_m);
    esdf_integrator_config.default_distance_m = this->declare_parameter(
        "esdf_default_distance_m", esdf_integrator_config.default_distance_m);
    esdf_integrator_config.min_diff_m = this->declare_parameter(
        "esdf_min_diff_m", esdf_integrator_config.min_diff_m);
    esdf_integrator_config.clear_sphere_radius = this->declare_parameter(
        "clear_sphere_radius", esdf_integrator_config.clear_sphere_radius);
    esdf_integrator_config.occupied_sphere_radius =
        this->declare_parameter("occupied_sphere_radius",
                                esdf_integrator_config.occupied_sphere_radius);
    esdf_integrator_config.add_occupied_crust = this->declare_parameter(
        "esdf_add_occupied_crust", esdf_integrator_config.add_occupied_crust);

    if (esdf_integrator_config.default_distance_m <
        esdf_integrator_config.max_distance_m) {
      esdf_integrator_config.default_distance_m =
          esdf_integrator_config.max_distance_m;
    }

    return esdf_integrator_config;
  }

 protected:
  /// Sets up publishing and subscribing. Should only be called from
  /// constructor.
  void setupRos();

  /// Publish markers for visualization.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      esdf_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_slice_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr traversable_pub_;

  /// Publish the complete map for other nodes to consume.
  rclcpp::Publisher<voxblox_msgs::msg::Layer>::SharedPtr esdf_map_pub_;

  /// Subscriber to subscribe to another node generating the map.
  rclcpp::Subscription<voxblox_msgs::msg::Layer>::SharedPtr esdf_map_sub_;

  /// Services.
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr generate_esdf_srv_;

  /// Timers.
  rclcpp::TimerBase::SharedPtr update_esdf_timer_;

  bool clear_sphere_for_planning_;
  bool publish_esdf_map_;
  bool publish_traversable_;
  float traversability_radius_;
  bool incremental_update_;
  int num_subscribers_esdf_map_;

  // ESDF maps.
  std::shared_ptr<EsdfMap> esdf_map_;
  std::unique_ptr<EsdfIntegrator> esdf_integrator_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_ESDF_SERVER_H_

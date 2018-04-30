#include "voxblox_ros/thermal_server.h"

namespace voxblox {

ThermalServer::ThermalServer(const ros::NodeHandle& nh,
                             const ros::NodeHandle& nh_private)
    : TsdfServer(nh, nh_private),
      focal_length_px_(391.5f),
      subsample_factor_(12) {
  cache_mesh_ = true;

  thermal_layer_.reset(
      new Layer<ThermalVoxel>(tsdf_map_->getTsdfLayer().voxel_size(),
                              tsdf_map_->getTsdfLayer().voxels_per_side()));
  thermal_integrator_.reset(
      new ThermalIntegrator(tsdf_map_->getTsdfLayer(), thermal_layer_.get()));

  // Get ROS params:
  nh_private_.param("thermal_focal_length", focal_length_px_, focal_length_px_);
  nh_private_.param("subsample_factor", subsample_factor_, subsample_factor_);

  float thermal_min_value = 10.0;
  float thermal_max_value = 40.0;
  nh_private_.param("thermal_min_value", thermal_min_value, thermal_min_value);
  nh_private_.param("thermal_max_value", thermal_max_value, thermal_max_value);

  FloatingPoint thermal_max_distance = thermal_integrator_->getMaxDistance();
  nh_private_.param("thermal_max_distance", thermal_max_distance,
                    thermal_max_distance);
  thermal_integrator_->setMaxDistance(thermal_max_distance);

  // Publishers for output.
  thermal_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
          "thermal_pointcloud", 1, true);
  thermal_mesh_pub_ =
      nh_private_.advertise<voxblox_msgs::Mesh>("thermal_mesh", 1, true);

  color_map_.reset(new IronbowColorMap());
  color_map_->setMinValue(thermal_min_value);
  color_map_->setMaxValue(thermal_max_value);

  // Set up subscriber.
  thermal_image_sub_ = nh_private_.subscribe(
      "thermal_image", 1, &ThermalServer::thermalImageCallback, this);
}

void ThermalServer::updateMesh() {
  TsdfServer::updateMesh();

  // Now recolor the mesh...
  timing::Timer publish_mesh_timer("thermal_mesh/publish");
  recolorVoxbloxMeshMsgByTemperature(*thermal_layer_, color_map_, &mesh_msg_);
  thermal_mesh_pub_.publish(mesh_msg_);
  publish_mesh_timer.Stop();
}

void ThermalServer::publishPointclouds() {
  // Create a pointcloud with temperature = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createTemperaturePointcloudFromThermalLayer(*thermal_layer_, &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  thermal_pointcloud_pub_.publish(pointcloud);

  TsdfServer::publishPointclouds();
}

void ThermalServer::thermalImageCallback(
    const sensor_msgs::ImageConstPtr& image) {
  CHECK(thermal_layer_);
  CHECK(thermal_integrator_);
  // Look up transform first...
  Transformation T_G_C;
  if (!transformer_.lookupTransform(image->header.frame_id, world_frame_,
                                    image->header.stamp, &T_G_C)) {
    ROS_WARN_THROTTLE(10, "Failed to look up thermal transform!");
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image);

  CHECK(cv_ptr);

  size_t num_pixels =
      cv_ptr->image.rows * cv_ptr->image.cols / subsample_factor_;

  float half_row = cv_ptr->image.rows / 2.0;
  float half_col = cv_ptr->image.cols / 2.0;

  // Pre-allocate the bearing vectors and temperatures.
  Pointcloud bearing_vectors;
  bearing_vectors.reserve(num_pixels + 1);
  std::vector<float> temperatures;
  temperatures.reserve(num_pixels + 1);

  size_t k = 0;
  size_t m = 0;
  for (int i = 0; i < cv_ptr->image.rows; i++) {
    const float* image_row = cv_ptr->image.ptr<float>(i);
    for (int j = 0; j < cv_ptr->image.cols; j++) {
      if (m % subsample_factor_ == 0) {
        bearing_vectors.push_back(
            T_G_C.getRotation().toImplementation() *
            Point(j - half_col, i - half_row, focal_length_px_).normalized());
        temperatures.push_back(image_row[j]);
        k++;
      }
      m++;
    }
  }

  // Put this into the integrator.
  thermal_integrator_->addThermalBearingVectors(T_G_C.getPosition(),
                                                bearing_vectors, temperatures);
}

}  // namespace voxblox

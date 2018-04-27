#include "voxblox_ros/thermal_server.h"

namespace voxblox {

ThermalServer::ThermalServer(const ros::NodeHandle& nh,
                             const ros::NodeHandle& nh_private)
    : TsdfServer(nh, nh_private), focal_length_px_(391.5f) {
  // Get ROS params:
  nh_private_.param("thermal_focal_length", focal_length_px_, focal_length_px_);

  // Publishers for output.
  thermal_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
          "thermal_pointcloud", 1, true);

  thermal_layer_.reset(
      new Layer<ThermalVoxel>(tsdf_map_->getTsdfLayer().voxel_size(),
                              tsdf_map_->getTsdfLayer().voxels_per_side()));
  thermal_integrator_.reset(
      new ThermalIntegrator(tsdf_map_->getTsdfLayer(), thermal_layer_.get()));
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
  // Look up transform first...
  Transformation T_G_C;
  transformer_.lookupTransform(image->header.frame_id, world_frame_,
                               image->header.stamp, &T_G_C);

  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image);

  size_t num_pixels = cv_ptr->image.rows * cv_ptr->image.cols;

  float half_row = cv_ptr->image.rows / 2.0;
  float half_col = cv_ptr->image.cols / 2.0;

  // Pre-allocate the bearing vectors and temperatures.
  Pointcloud bearing_vectors;
  bearing_vectors.resize(num_pixels);
  std::vector<float> temperatures;
  temperatures.resize(num_pixels);

  size_t k = 0;
  for (int i = 0; i < cv_ptr->image.rows; i++) {
    const float* image_row = cv_ptr->image.ptr<float>(i);
    for (int j = 0; j < cv_ptr->image.cols; j++) {
      bearing_vectors[k] =
          T_G_C.getRotation().toImplementation() *
          Point(j - half_col, i - half_row, focal_length_px_).normalized();
      temperatures[k] = image_row[j];
      k++;
    }
  }

  // Put this into the integrator.
  thermal_integrator_->addThermalBearingVectors(T_G_C.getPosition(),
                                                bearing_vectors, temperatures);
}

}  // namespace voxblox

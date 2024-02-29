#include "voxblox_ros/intensity_server.h"

namespace voxblox {

IntensityServer::IntensityServer()
    : TsdfServer(), focal_length_px_(400.0f), subsample_factor_(12) {
  cache_mesh_ = true;

  intensity_layer_.reset(
      new Layer<IntensityVoxel>(tsdf_map_->getTsdfLayer().voxel_size(),
                                tsdf_map_->getTsdfLayer().voxels_per_side()));
  intensity_integrator_.reset(new IntensityIntegrator(tsdf_map_->getTsdfLayer(),
                                                      intensity_layer_.get()));

  // Get ROS params:
  focal_length_px_ =
      this->declare_parameter("intensity_focal_length", focal_length_px_);
  subsample_factor_ =
      this->declare_parameter("subsample_factor", subsample_factor_);

  float intensity_min_value = 10.0f;
  float intensity_max_value = 40.0f;
  intensity_min_value =
      this->declare_parameter("intensity_min_value", intensity_min_value);
  intensity_max_value =
      this->declare_parameter("intensity_max_value", intensity_max_value);

  FloatingPoint intensity_max_distance =
      intensity_integrator_->getMaxDistance();
  intensity_max_distance =
      this->declare_parameter("intensity_max_distance", intensity_max_distance);
  intensity_integrator_->setMaxDistance(intensity_max_distance);

  // Publishers for output.
  intensity_pointcloud_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "intensity_pointcloud", 1);
  intensity_mesh_pub_ =
      this->create_publisher<voxblox_msgs::msg::Mesh>("intensity_mesh", 1);

  color_map_.reset(new IronbowColorMap());
  color_map_->setMinValue(intensity_min_value);
  color_map_->setMaxValue(intensity_max_value);

  // Set up subscriber.
  intensity_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "intensity_image", 1,
      std::bind(&IntensityServer::intensityImageCallback, this,
                std::placeholders::_1));
}

void IntensityServer::updateMesh() {
  TsdfServer::updateMesh();

  // Now recolor the mesh...
  timing::Timer publish_mesh_timer("intensity_mesh/publish");
  recolorVoxbloxMeshMsgByIntensity(*intensity_layer_, color_map_,
                                   &cached_mesh_msg_);
  intensity_mesh_pub_->publish(cached_mesh_msg_);
  publish_mesh_timer.Stop();
}

void IntensityServer::publishPointclouds() {
  // Create a pointcloud with temperature = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createIntensityPointcloudFromIntensityLayer(*intensity_layer_, &pointcloud);

  pointcloud.header.frame_id = world_frame_;

  sensor_msgs::msg::PointCloud2 pointcloud_message;
  pcl::toROSMsg(pointcloud, pointcloud_message);

  intensity_pointcloud_pub_->publish(pointcloud_message);

  TsdfServer::publishPointclouds();
}

void IntensityServer::intensityImageCallback(
    const sensor_msgs::msg::Image::SharedPtr image) {
  CHECK(intensity_layer_);
  CHECK(intensity_integrator_);
  CHECK(image);
  // Look up transform first...
  Transformation T_G_C;
  if (!transformer_.lookupTransform(image->header.frame_id, world_frame_,
                                    image->header.stamp, &T_G_C)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10,
                         "Failed to look up intensity transform!");
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image);

  CHECK(cv_ptr);

  const size_t num_pixels =
      cv_ptr->image.rows * cv_ptr->image.cols / subsample_factor_;

  float half_row = cv_ptr->image.rows / 2.0;
  float half_col = cv_ptr->image.cols / 2.0;

  // Pre-allocate the bearing vectors and intensities.
  Pointcloud bearing_vectors;
  bearing_vectors.reserve(num_pixels + 1);
  std::vector<float> intensities;
  intensities.reserve(num_pixels + 1);

  size_t k = 0;
  size_t m = 0;
  for (int i = 0; i < cv_ptr->image.rows; i++) {
    const float* image_row = cv_ptr->image.ptr<float>(i);
    for (int j = 0; j < cv_ptr->image.cols; j++) {
      if (m % subsample_factor_ == 0) {
        // Rotates the vector pointing from the camera center to the pixel
        // into the global frame, and normalizes it.
        bearing_vectors.push_back(
            T_G_C.getRotation().toImplementation() *
            Point(j - half_col, i - half_row, focal_length_px_).normalized());
        intensities.push_back(image_row[j]);
        k++;
      }
      m++;
    }
  }

  // Put this into the integrator.
  intensity_integrator_->addIntensityBearingVectors(
      T_G_C.getPosition(), bearing_vectors, intensities);
}

}  // namespace voxblox

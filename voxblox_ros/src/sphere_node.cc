#include <geometry_msgs/Point.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <voxblox/core/common.h>
#include <voxblox/simulation/sphere_simulator.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "sphere_simulator");

  ros::NodeHandle node_handle;
  ros::Publisher point_cloud_publisher;
  point_cloud_publisher =
      node_handle.advertise<sensor_msgs::PointCloud2>("sphere", 1);

  constexpr double kMean = 0;
  constexpr double kSigma = 0.05;
  constexpr int kNumPoints = 1000000;
  constexpr double kRadius = 5.0;

  LOG(INFO) << "***Sphere Simulator Params***";
  LOG(INFO) << "Mean: " << kMean;
  LOG(INFO) << "Sigma: " << kSigma;
  LOG(INFO) << "#Points: " << kNumPoints;
  LOG(INFO) << "Radius: " << kRadius;
  LOG(INFO) << "********************";

  voxblox::Pointcloud sphere_points;
  voxblox::sphere_sim::createSphere(kMean, kSigma, kRadius, kNumPoints,
                                    &sphere_points);

  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;

  uint32_t rgb = ((uint32_t)0 << 16 | (uint32_t)0 << 8 | (uint32_t)0);

  // Calculate coordinates.
  for (int it = 0; it < sphere_points.size(); ++it) {
    pcl::PointXYZRGB pcl_point;
    pcl_point.x = sphere_points[it].x();
    pcl_point.y = sphere_points[it].y();
    pcl_point.z = sphere_points[it].z();
    pcl_point.rgb = *reinterpret_cast<float*>(&rgb);
    pcl_cloud.points.push_back(pcl_point);
  }

  sensor_msgs::PointCloud2 pcl_message;
  if (!pcl_cloud.points.empty()) {
    pcl::toROSMsg(pcl_cloud, pcl_message);
    pcl_message.header.frame_id = "world";
    pcl_message.header.stamp = ros::Time::now();
    LOG(INFO) << "Publishing sphere with " << pcl_cloud.points.size()
              << " points";
  }

  ros::Rate ros_rate(1);
  while (ros::ok()) {
    if (!pcl_cloud.points.empty()) {
      point_cloud_publisher.publish(pcl_message);
    }

    ros::spinOnce();
    ros_rate.sleep();
  }

  return 0;
}

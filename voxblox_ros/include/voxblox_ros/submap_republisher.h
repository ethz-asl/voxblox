#ifndef VOXBLOX_ROS_SUBMAP_REPUBLISHER_H_
#define VOXBLOX_ROS_SUBMAP_REPUBLISHER_H_

#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/io/trajectory_io.h>

namespace voxblox {
class SubmapRepublisher {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SubmapRepublisher(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);

  void newSubmapNotificationCallback(const std_msgs::String& msg);

  void publishSubmap(const Layer<TsdfVoxel>& tsdf_layer,
                     const io::Trajectory& trajectory);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber new_submap_notification_sub_;
  ros::Publisher submap_pub_;
};
}  // namespace voxblox

#endif  // VOXBLOX_ROS_SUBMAP_REPUBLISHER_H_

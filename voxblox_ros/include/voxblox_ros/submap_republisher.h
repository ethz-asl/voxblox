#ifndef VOXBLOX_ROS_SUBMAP_REPUBLISHER_H_
#define VOXBLOX_ROS_SUBMAP_REPUBLISHER_H_

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <voxblox/Trajectory.pb.h>
#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>

namespace voxblox {
class SubmapRepublisher {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // TODO(victorr): Consider making a dedicated Trajectory class,
  //                with built-in proto (and possibly ROS msg) converters
  struct StampedPose {
    ros::Time timestamp;
    Transformation pose;
    static StampedPose fromProto(const StampedPoseProto& proto);
  };

  SubmapRepublisher(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);

  void newSubmapNotificationCallback(const std_msgs::String& msg);

  // TODO(victorr): Move the submap saving and loading methods to a single
  //                class, used by the TsdfServer and the SubmapRepublisher
  bool loadSubmap(const std::string& submap_folder_path);
  bool loadMap(const std::string& file_path);
  bool loadTrajectory(const std::string& file_path);

  void publishSubmap();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber new_submap_notification_sub_;
  ros::Publisher submap_pub_;

  std::string robot_name_;
  std::string world_frame_;
  std::shared_ptr<Layer<TsdfVoxel>> tsdf_layer_ptr_;
  std::vector<StampedPose> trajectory_;
};
}  // namespace voxblox

#endif  // VOXBLOX_ROS_SUBMAP_REPUBLISHER_H_

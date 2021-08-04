#include "voxblox_ros/submap_republisher.h"

#include <minkindr_conversions/kindr_msg.h>
#include <voxblox/io/submap_io.h>
#include <voxblox_msgs/Submap.h>

#include "voxblox_ros/conversions.h"

namespace voxblox {

SubmapRepublisher::SubmapRepublisher(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  new_submap_notification_sub_ =
      nh_.subscribe("new_submap_written_to_disk", 1000,
                    &SubmapRepublisher::newSubmapNotificationCallback, this);
  submap_pub_ =
      nh_private_.advertise<voxblox_msgs::Submap>("submap_out", 1, false);
}

void SubmapRepublisher::newSubmapNotificationCallback(
    const std_msgs::String& msg) {
  const std::string submap_folder_path = msg.data;

  // Load the submap, populate the msg and publish it
  Layer<TsdfVoxel>::Ptr tsdf_layer_ptr;
  io::Trajectory trajectory;
  if (io::LoadSubmap(submap_folder_path, &tsdf_layer_ptr, &trajectory)) {
    publishSubmap(*tsdf_layer_ptr, trajectory);
  } else {
    ROS_ERROR_STREAM("Failed to load submap from directory \""
                     << submap_folder_path << "\".");
  }
}

void SubmapRepublisher::publishSubmap(const Layer<TsdfVoxel>& tsdf_layer,
                                      const io::Trajectory& trajectory) {
  // Publish the submap if anyone is listening
  if (0 < submap_pub_.getNumSubscribers()) {
    voxblox_msgs::Submap submap_msg;
    submap_msg.robot_name = trajectory.robot_name;

    constexpr bool kSerializeOnlyUpdated = false;
    serializeLayerAsMsg<TsdfVoxel>(tsdf_layer, kSerializeOnlyUpdated,
                                   &submap_msg.layer);

    for (const io::StampedPose& stamped_pose : trajectory.stamped_poses) {
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.frame_id = trajectory.frame_id;
      pose_msg.header.stamp.fromNSec(stamped_pose.timestamp);
      tf::poseKindrToMsg(stamped_pose.pose.cast<double>(), &pose_msg.pose);
      submap_msg.trajectory.poses.emplace_back(pose_msg);
    }

    submap_pub_.publish(submap_msg);
  }
}

}  // namespace voxblox

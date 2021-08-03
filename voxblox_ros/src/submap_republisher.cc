#include "voxblox_ros/submap_republisher.h"

#include <minkindr_conversions/kindr_msg.h>
#include <voxblox/io/layer_io.h>
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
  if (loadSubmap(submap_folder_path)) {
    publishSubmap();
  } else {
    ROS_ERROR_STREAM("Failed to load submap from directory \""
                     << submap_folder_path << "\".");
  }
}

bool SubmapRepublisher::loadSubmap(const std::string& submap_folder_path) {
  // Load the TSDF
  const std::string volumetric_map_file_path =
      submap_folder_path + "/volumetric_map.tsdf";
  if (!loadMap(volumetric_map_file_path)) {
    ROS_ERROR_STREAM("Failed to load submap TSDF from file \""
                     << volumetric_map_file_path << "\".");
    return false;
  }

  // Load the trajectory
  const std::string trajectory_file_path =
      submap_folder_path + "/robot_trajectory.traj";
  if (!loadTrajectory(trajectory_file_path)) {
    ROS_ERROR_STREAM("Failed to load submap trajectory from file \""
                     << trajectory_file_path << "\".");
    return false;
  }

  return true;
}

bool SubmapRepublisher::loadMap(const std::string& file_path) {
  // TODO(victorr): Consider automatically reinterpolating all
  //                incoming submaps to a common voxel_size
  if (io::LoadLayer<TsdfVoxel>(file_path, &tsdf_layer_ptr_)) {
    return true;
  }
  return false;
}

bool SubmapRepublisher::loadTrajectory(const std::string& file_path) {
  // Open and check the file
  std::fstream proto_file;
  proto_file.open(file_path, std::fstream::in);
  if (!proto_file.is_open()) {
    ROS_ERROR_STREAM(
        "Could not open protobuf file to load trajectory: " << file_path);
    return false;
  }

  // Read trajectory from file
  TrajectoryProto trajectory_proto;
  if (trajectory_proto.ParseFromIstream(&proto_file)) {
    robot_name_ = trajectory_proto.robot_name();
    world_frame_ = trajectory_proto.frame_id();
    trajectory_.clear();
    for (const StampedPoseProto& stamped_pose_proto :
         trajectory_proto.stamped_poses()) {
      trajectory_.emplace_back(StampedPose::fromProto(stamped_pose_proto));
    }
  } else {
    return false;
  }

  return true;
}

void SubmapRepublisher::publishSubmap() {
  // Publish the submap if anyone is listening
  if (0 < submap_pub_.getNumSubscribers()) {
    voxblox_msgs::Submap submap_msg;
    submap_msg.robot_name = robot_name_;

    constexpr bool kSerializeOnlyUpdated = false;
    serializeLayerAsMsg<TsdfVoxel>(*tsdf_layer_ptr_, kSerializeOnlyUpdated,
                                   &submap_msg.layer);

    for (const StampedPose& stamped_pose : trajectory_) {
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.frame_id = world_frame_;
      pose_msg.header.stamp = stamped_pose.timestamp;
      tf::poseKindrToMsg(stamped_pose.pose.cast<double>(), &pose_msg.pose);
      submap_msg.trajectory.poses.emplace_back(pose_msg);
    }
    submap_pub_.publish(submap_msg);
  }
}

SubmapRepublisher::StampedPose SubmapRepublisher::StampedPose::fromProto(
    const StampedPoseProto& proto) {
  StampedPose stamped_pose;

  stamped_pose.timestamp.fromNSec(proto.timestamp());

  Point& position = stamped_pose.pose.getPosition();
  const PositionProto& position_proto = proto.pose().position();
  position = {position_proto.x(), position_proto.y(), position_proto.z()};

  Rotation& orientation = stamped_pose.pose.getRotation();
  const OrientationProto& orientation_proto = proto.pose().orientation();
  orientation.setValues(orientation_proto.w(), orientation_proto.x(),
                        orientation_proto.y(), orientation_proto.z());

  return stamped_pose;
}
}  // namespace voxblox

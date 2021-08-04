#include "voxblox/io/trajectory_io.h"

namespace voxblox {
namespace io {

StampedPose::StampedPose(const StampedPoseProto& proto) {
  timestamp = proto.timestamp();

  const PoseProto& pose_proto = proto.pose();

  const PositionProto& position_proto = pose_proto.position();
  Point& position = pose.getPosition();
  position = {position_proto.x(), position_proto.y(), position_proto.z()};

  const OrientationProto& orientation_proto = pose_proto.orientation();
  Rotation& orientation = pose.getRotation();
  orientation.setValues(orientation_proto.w(), orientation_proto.x(),
                        orientation_proto.y(), orientation_proto.z());
}

void StampedPose::getProto(StampedPoseProto* proto) const {
  CHECK_NOTNULL(proto);

  proto->set_timestamp(timestamp);

  PoseProto* pose_proto = proto->mutable_pose();

  const Point& position = pose.getPosition();
  PositionProto* position_proto = pose_proto->mutable_position();
  position_proto->set_x(position.x());
  position_proto->set_y(position.y());
  position_proto->set_z(position.z());

  const Transformation::Rotation& orientation = pose.getRotation();
  OrientationProto* orientation_proto = pose_proto->mutable_orientation();
  orientation_proto->set_w(orientation.w());
  orientation_proto->set_x(orientation.x());
  orientation_proto->set_y(orientation.y());
  orientation_proto->set_z(orientation.z());
}

Trajectory::Trajectory(const TrajectoryProto& proto) {
  robot_name = proto.robot_name();
  frame_id = proto.frame_id();
  for (const StampedPoseProto& stamped_pose_proto : proto.stamped_poses()) {
    stamped_poses.emplace_back(stamped_pose_proto);
  }
}

void Trajectory::getProto(TrajectoryProto* proto) const {
  CHECK_NOTNULL(proto);

  proto->set_robot_name(robot_name);
  proto->set_frame_id(frame_id);
  for (const StampedPose& stamped_pose : stamped_poses) {
    StampedPoseProto* new_stamped_pose = proto->add_stamped_poses();
    stamped_pose.getProto(new_stamped_pose);
  }
}

}  // namespace io
}  // namespace voxblox

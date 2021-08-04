#ifndef VOXBLOX_IO_TRAJECTORY_IO_H_
#define VOXBLOX_IO_TRAJECTORY_IO_H_

#include <string>
#include <utility>
#include <vector>

#include <glog/logging.h>

#include "voxblox/Trajectory.pb.h"
#include "voxblox/core/common.h"

namespace voxblox {
namespace io {

struct StampedPose {
  StampedPose() = default;
  StampedPose(const uint64_t _timestamp, const Transformation& _pose)
      : timestamp(_timestamp), pose(_pose) {}

  explicit StampedPose(const StampedPoseProto& proto);
  void getProto(StampedPoseProto* proto) const;

  uint64_t timestamp = 0u;
  Transformation pose;
};

struct Trajectory {
  Trajectory() = default;
  Trajectory(std::string _robot_name, std::string _frame_id,
             std::vector<StampedPose> _stamped_poses)
      : robot_name(std::move(_robot_name)),
        frame_id(std::move(_frame_id)),
        stamped_poses(std::move(_stamped_poses)) {}

  explicit Trajectory(const TrajectoryProto& proto);
  void getProto(TrajectoryProto* proto) const;

  std::string robot_name;
  std::string frame_id;
  std::vector<StampedPose> stamped_poses;
};

inline bool SaveTrajectory(const Trajectory& trajectory,
                           const std::string& file_path);

inline bool LoadTrajectory(const std::string& file_path,
                           Trajectory* trajectory_ptr);

}  // namespace io
}  // namespace voxblox

#include "voxblox/io/trajectory_io_inl.h"

#endif  // VOXBLOX_IO_TRAJECTORY_IO_H_

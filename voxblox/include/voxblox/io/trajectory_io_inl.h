#ifndef VOXBLOX_IO_TRAJECTORY_IO_INL_H_
#define VOXBLOX_IO_TRAJECTORY_IO_INL_H_

#include <fstream>
#include <string>

namespace voxblox {
namespace io {
bool SaveTrajectory(const Trajectory& trajectory,
                    const std::string& file_path) {
  // Create and open the file
  const std::ios_base::openmode file_flags =
      std::fstream::out | std::fstream::binary | std::fstream::trunc;
  std::ofstream file_stream(file_path, file_flags);
  if (!file_stream.is_open()) {
    LOG(WARNING) << "Could not open file '" << file_path
                 << "' to save the trajectory.";
    return false;
  }

  // Write trajectory to file
  TrajectoryProto trajectory_proto;
  trajectory.getProto(&trajectory_proto);

  if (!trajectory_proto.SerializeToOstream(&file_stream)) {
    LOG(ERROR) << "Could not serialize trajectory to protobuf file: "
               << file_path;
    return false;
  }

  return true;
}

bool LoadTrajectory(const std::string& file_path, Trajectory* trajectory_ptr) {
  CHECK_NOTNULL(trajectory_ptr);

  // Open and check the file
  std::fstream proto_file;
  proto_file.open(file_path, std::fstream::in);
  if (!proto_file.is_open()) {
    LOG(ERROR) << "Could not open protobuf file to load trajectory: "
               << file_path;
    return false;
  }

  // Read trajectory from file
  TrajectoryProto trajectory_proto;
  if (!trajectory_proto.ParseFromIstream(&proto_file)) {
    LOG(ERROR) << "Could not parse trajectory from protobuf file: "
               << file_path;
    return false;
  }
  *trajectory_ptr = Trajectory(trajectory_proto);

  return true;
}
}  // namespace io
}  // namespace voxblox

#endif  // VOXBLOX_IO_TRAJECTORY_IO_INL_H_

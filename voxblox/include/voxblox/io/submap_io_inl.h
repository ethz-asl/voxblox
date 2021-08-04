#ifndef VOXBLOX_IO_SUBMAP_IO_INL_H_
#define VOXBLOX_IO_SUBMAP_IO_INL_H_

#include <string>

namespace voxblox {
namespace io {

bool SaveSubmap(const std::string& submap_folder_path,
                const Layer<TsdfVoxel>& tsdf_layer,
                const Trajectory& trajectory) {
  // Save the TSDF
  const std::string volumetric_map_file_path =
      submap_folder_path + "/volumetric_map.tsdf";
  if (!io::SaveLayer(tsdf_layer, volumetric_map_file_path)) {
    LOG(ERROR) << "Failed to write submap TSDF to file \""
               << volumetric_map_file_path << "\".";
    return false;
  }

  // Save the trajectory
  const std::string trajectory_file_path =
      submap_folder_path + "/robot_trajectory.traj";
  if (!io::SaveTrajectory(trajectory, trajectory_file_path)) {
    LOG(ERROR) << "Failed to write submap trajectory to file \""
               << trajectory_file_path << "\".";
    return false;
  }

  return true;
}

bool LoadSubmap(const std::string& submap_folder_path,
                Layer<TsdfVoxel>::Ptr* tsdf_layer_ptr,
                io::Trajectory* trajectory_ptr) {
  CHECK_NOTNULL(tsdf_layer_ptr);
  CHECK_NOTNULL(trajectory_ptr);

  // Load the TSDF
  const std::string volumetric_map_file_path =
      submap_folder_path + "/volumetric_map.tsdf";
  // TODO(victorr): Consider automatically reinterpolating all
  //                incoming submaps to a common voxel_size
  if (!io::LoadLayer<TsdfVoxel>(volumetric_map_file_path, tsdf_layer_ptr)) {
    LOG(ERROR) << "Failed to load submap TSDF from file \""
               << volumetric_map_file_path << "\".";
    return false;
  }

  // Load the trajectory
  const std::string trajectory_file_path =
      submap_folder_path + "/robot_trajectory.traj";
  if (!io::LoadTrajectory(trajectory_file_path, trajectory_ptr)) {
    LOG(ERROR) << "Failed to load submap trajectory from file \""
               << trajectory_file_path << "\".";
    return false;
  }

  return true;
}

}  // namespace io
}  // namespace voxblox

#endif  // VOXBLOX_IO_SUBMAP_IO_INL_H_

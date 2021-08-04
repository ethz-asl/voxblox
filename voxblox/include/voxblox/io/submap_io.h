#ifndef VOXBLOX_IO_SUBMAP_IO_H_
#define VOXBLOX_IO_SUBMAP_IO_H_

#include <string>

#include "voxblox/core/common.h"
#include "voxblox/io/layer_io.h"
#include "voxblox/io/trajectory_io.h"

namespace voxblox {
namespace io {

inline bool SaveSubmap(const std::string& submap_folder_path,
                       const Layer<TsdfVoxel>& tsdf_layer,
                       const io::Trajectory& trajectory);

inline bool LoadSubmap(const std::string& submap_folder_path,
                       Layer<TsdfVoxel>::Ptr* tsdf_layer_ptr,
                       io::Trajectory* trajectory_ptr);

}  // namespace io
}  // namespace voxblox

#include "voxblox/io/submap_io_inl.h"

#endif  // VOXBLOX_IO_SUBMAP_IO_H_

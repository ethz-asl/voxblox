//
// Created by nico on 19.09.19.
//

#ifndef VOXBLOX_ROS_INCLUDE_VOXBLOX_ROS_RAY_CASTING_H_
#define VOXBLOX_ROS_INCLUDE_VOXBLOX_ROS_RAY_CASTING_H_

#include <unordered_set>
#include <vector>

#include <voxblox/core/common.h>
#include <voxblox/core/tsdf_map.h>
#include <Eigen/Eigen>

// An extension of std, requirement to support
// std::unordered_set<Eigen::Matrix<int64_t, 3, 1>>
namespace std {
// hash an Eigen::Matrix from
// https://github.com/ethz-asl/map_api/blob/master/map-api-common/include/map-api-common/eigen-hash.h
// based on boost::hash_combine
// http://www.boost.org/doc/libs/1_55_0/doc/html/hash/reference.html#boost.hash_combine
template <typename Scalar, int Rows, int Cols>
struct hash<Eigen::Matrix<Scalar, Rows, Cols>> {
  size_t operator()(const Eigen::Matrix<Scalar, Rows, Cols>& matrix) const {
    size_t seed = 0;
    for (Eigen::Index i = 0; i < matrix.size(); ++i) {
      Scalar elem = *(matrix.data() + i);
      seed ^=
          std::hash<Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

}  // namespace std

namespace voxblox {

inline void performRayCasting(
    const voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer,
    const Eigen::Vector3d& view_point,
    const std::vector<Eigen::Vector3d>& endpoints,
    const std::vector<Eigen::Vector3d>& startpoints,
    std::vector<Eigen::Vector3d>& map_endpoints) {
  const float voxel_size = tsdf_layer->voxel_size();
  const float voxel_size_inv = 1.0 / voxel_size;

  std::unordered_set<voxblox::GlobalIndex> raycast_occupied_set_;
  raycast_occupied_set_.reserve(endpoints.size());
  for (size_t i = 0; i < endpoints.size(); ++i) {
    voxblox::Point start_scaled;
    if (startpoints.size() == endpoints.size()) {
      start_scaled =
          startpoints[i].cast<voxblox::FloatingPoint>() * voxel_size_inv;
    } else {
      start_scaled = view_point.cast<voxblox::FloatingPoint>() * voxel_size_inv;
    }
    const voxblox::Point end_scaled =
        endpoints[i].cast<voxblox::FloatingPoint>() * voxel_size_inv;

    voxblox::LongIndexVector global_voxel_indices;
    voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);

    // Iterate over the ray.
    for (size_t k = 0; k < global_voxel_indices.size(); ++k) {
      const voxblox::GlobalIndex& global_index = global_voxel_indices[k];
      const voxblox::TsdfVoxel* voxel =
          tsdf_layer->getVoxelPtrByGlobalIndex(global_index);
      if (voxel == nullptr || voxel->weight < 1e-6) {
        continue;
      }
      if (voxel->distance <= 0.0) {
        raycast_occupied_set_.insert(global_index);
        break;
      }
      k += voxel->distance * voxel_size_inv;
    }
  }
  map_endpoints.reserve(raycast_occupied_set_.size());
  for (auto it = raycast_occupied_set_.begin();
       it != raycast_occupied_set_.end(); ++it) {
    map_endpoints.push_back(
        voxblox::getCenterPointFromGridIndex(*it, voxel_size).cast<double>());
  }
}
}  // namespace voxblox
#endif  // VOXBLOX_ROS_INCLUDE_VOXBLOX_ROS_RAY_CASTING_H_

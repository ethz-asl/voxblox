#ifndef VOXBLOX_ROS_INTENSITY_VIS_H_
#define VOXBLOX_ROS_INTENSITY_VIS_H_

#include <memory>

#include <voxblox/utils/color_maps.h>
#include <voxblox_msgs/Mesh.h>

#include "voxblox_ros/conversions.h"
#include "voxblox_ros/mesh_vis.h"

namespace voxblox {

inline void recolorVoxbloxMeshMsgByIntensity(
    const Layer<IntensityVoxel>& intensity_layer,
    const std::shared_ptr<ColorMap>& color_map, voxblox_msgs::Mesh* mesh_msg) {
  CHECK_NOTNULL(mesh_msg);
  CHECK(color_map);

  // Go over all the blocks in the mesh.
  for (voxblox_msgs::MeshBlock& mesh_block : mesh_msg->mesh_blocks) {
    // Go over all the triangles in the mesh.
    for (voxblox_msgs::Triangle& triangle : mesh_block.triangles) {
      // Look up triangles in the thermal layer.
      for (size_t local_vert_idx = 0u; local_vert_idx < 3; ++local_vert_idx) {
        const IntensityVoxel* voxel = intensity_layer.getVoxelPtrByCoordinates(
            Point(triangle.x[local_vert_idx], triangle.y[local_vert_idx],
                  triangle.z[local_vert_idx]));
        if (voxel != nullptr && voxel->weight > 0.0) {
          float intensity = voxel->intensity;
          Color new_color = color_map->colorLookup(intensity);
          triangle.r[local_vert_idx] = new_color.r;
          triangle.g[local_vert_idx] = new_color.g;
          triangle.b[local_vert_idx] = new_color.b;
          triangle.a[local_vert_idx] = new_color.a;
        }
      }
    }
  }
}

}  // namespace voxblox

#endif  // VOXBLOX_ROS_INTENSITY_VIS_H_

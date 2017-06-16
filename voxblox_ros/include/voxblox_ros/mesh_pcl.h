// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// Mesh output taken from open_chisel: github.com/personalrobotics/OpenChisel

#ifndef VOXBLOX_ROS_MESH_PCL_H_
#define VOXBLOX_ROS_MESH_PCL_H_

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <voxblox/core/common.h>
#include <voxblox/mesh/mesh_layer.h>

namespace voxblox {

inline void toPCLPolygonMesh(const MeshLayer& mesh_layer,
                             const std::string frame_id,
                             pcl::PolygonMesh* polygon_mesh_ptr) {
  // Constructing the vertices pointcloud
  pcl::PointCloud<pcl::PointXYZ> pointcloud;
  std::vector<pcl::Vertices> polygons;
  BlockIndexList mesh_indices;
  mesh_layer.getAllAllocatedMeshes(&mesh_indices);
  // Looping over the block indices and adding the pointcloud
  for (const BlockIndex& block_index : mesh_indices) {
    // Getting the mesh triangles in this block
    const Mesh& mesh = mesh_layer.getMeshByIndex(block_index);
    // Looping over vertices in this mesh
    for (const Point& point : mesh.vertices) {
      pointcloud.push_back(pcl::PointXYZ(static_cast<float>(point[0]),
                                         static_cast<float>(point[1]),
                                         static_cast<float>(point[2])));
    }
    // Adding the triangles
    VertexIndex last_biggest_index;
    if (!polygons.empty()) {
      last_biggest_index = polygons.back().vertices.back();
    } else {
      last_biggest_index = -1;
    }
    for (size_t start_idx = 0; start_idx < mesh.indices.size();
         start_idx += 3) {
      pcl::Vertices vertices;
      for (int vertex_idx = 0; vertex_idx < 3; vertex_idx++) {
        vertices.vertices.push_back(
            last_biggest_index + mesh.indices.at(start_idx + vertex_idx) + 1);
      }
      polygons.push_back(vertices);
    }
  }
  // Converting to the pointcloud binary
  pcl::PCLPointCloud2 pointcloud2;
  pcl::toPCLPointCloud2(pointcloud, pointcloud2);
  // Filling the mesh
  polygon_mesh_ptr->header.frame_id = frame_id;
  polygon_mesh_ptr->cloud = pointcloud2;
  polygon_mesh_ptr->polygons = polygons;
}

}  // namespace voxblox

#endif  // VOXBLOX_ROS_MESH_PCL_H_

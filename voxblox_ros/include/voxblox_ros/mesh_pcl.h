
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

#include <string>
#include <vector>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <voxblox/core/common.h>
#include <voxblox/mesh/mesh_layer.h>

namespace voxblox {

inline void toPCLPolygonMesh(
    const MeshLayer& mesh_layer, const std::string frame_id,
    pcl::PolygonMesh* polygon_mesh_ptr,
    const bool simplify_and_connect_mesh = true,
    const FloatingPoint vertex_proximity_threshold = 1e-10) {
  CHECK_NOTNULL(polygon_mesh_ptr);

  // Constructing the vertices pointcloud
  pcl::PointCloud<pcl::PointXYZ> pointcloud;
  std::vector<pcl::Vertices> polygons;

  Mesh mesh;
  convertMeshLayerToMesh(mesh_layer, &mesh, simplify_and_connect_mesh,
                         vertex_proximity_threshold);

  // add points
  pointcloud.reserve(mesh.vertices.size());
  for (const Point& point : mesh.vertices) {
    pointcloud.push_back(pcl::PointXYZ(static_cast<float>(point[0]),
                                       static_cast<float>(point[1]),
                                       static_cast<float>(point[2])));
  }
  // add triangles
  pcl::Vertices vertices_idx;
  polygons.reserve(mesh.indices.size() / 3);
  for (const VertexIndex& idx : mesh.indices) {
    vertices_idx.vertices.push_back(idx);

    if (vertices_idx.vertices.size() == 3) {
      polygons.push_back(vertices_idx);
      vertices_idx.vertices.clear();
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

inline void toSimplifiedPCLPolygonMesh(
    const MeshLayer& mesh_layer, const std::string frame_id,
    const FloatingPoint vertex_proximity_threshold,
    pcl::PolygonMesh* polygon_mesh_ptr) {
  constexpr bool kSimplifiedAndConnectedMesh = true;
  toPCLPolygonMesh(mesh_layer, frame_id, polygon_mesh_ptr,
                   kSimplifiedAndConnectedMesh, vertex_proximity_threshold);
}

inline void toConnectedPCLPolygonMesh(const MeshLayer& mesh_layer,
                                      const std::string frame_id,
                                      pcl::PolygonMesh* polygon_mesh_ptr) {
  constexpr bool kSimplifiedAndConnectedMesh = true;
  constexpr FloatingPoint kVertexThreshold = 1e-10;
  toPCLPolygonMesh(mesh_layer, frame_id, polygon_mesh_ptr,
                   kSimplifiedAndConnectedMesh, kVertexThreshold);
}

}  // namespace voxblox

#endif  // VOXBLOX_ROS_MESH_PCL_H_

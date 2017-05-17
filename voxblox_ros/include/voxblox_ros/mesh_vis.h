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

#ifndef VOXBLOX_ROS_MESH_VIS_H_
#define VOXBLOX_ROS_MESH_VIS_H_

#include <algorithm>
#include <visualization_msgs/Marker.h>
#include <eigen_conversions/eigen_msg.h>

#include <voxblox/core/common.h>
#include <voxblox/mesh/mesh.h>
#include <voxblox/mesh/mesh_layer.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/integrator/esdf_integrator.h>

#include "voxblox_ros/conversions.h"

namespace voxblox {

enum ColorMode {
  kColor = 0,
  kHeight,
  kNormals,
  kGray,
  kLambert,
  kLambertColor
};

inline Point lambertShading(const Point& normal, const Point& light,
                            const Point& color) {
  return std::max<FloatingPoint>(normal.dot(light), 0.0f) * color;
}

inline void lambertColorFromNormal(const Point& normal,
                                   std_msgs::ColorRGBA* color_msg) {
  const Point light_dir = Point(0.8f, -0.2f, 0.7f).normalized();
  const Point light_dir2 = Point(-0.5f, 0.2f, 0.2f).normalized();
  const Point ambient(0.2f, 0.2f, 0.2f);
  const Point color_pt(0.5, 0.5, 0.5);

  Point lambert = lambertShading(normal, light_dir, color_pt) +
                  lambertShading(normal, light_dir2, color_pt) + ambient;

  color_msg->r = std::min<FloatingPoint>(lambert.x(), 1.0);
  color_msg->g = std::min<FloatingPoint>(lambert.y(), 1.0);
  color_msg->b = std::min<FloatingPoint>(lambert.z(), 1.0);
  color_msg->a = 1.0;
}

inline void lambertColorFromColorAndNormal(const Color& color,
                                           const Point& normal,
                                           std_msgs::ColorRGBA* color_msg) {
  const Point light_dir = Point(0.8f, -0.2f, 0.7f).normalized();
  const Point light_dir2 = Point(-0.5f, 0.2f, 0.2f).normalized();
  const Point ambient(0.2f, 0.2f, 0.2f);
  const Point color_pt(color.r/255.0, color.g/255.0, color.b/255.0);

  Point lambert = lambertShading(normal, light_dir, color_pt) +
                  lambertShading(normal, light_dir2, color_pt) + ambient;

  color_msg->r = std::min<FloatingPoint>(lambert.x(), 1.0);
  color_msg->g = std::min<FloatingPoint>(lambert.y(), 1.0);
  color_msg->b = std::min<FloatingPoint>(lambert.z(), 1.0);
  color_msg->a = 1.0;
}

inline void normalColorFromNormal(const Point& normal,
                                  std_msgs::ColorRGBA* color_msg) {
  // Normals should be in the scale -1 to 1, so we need to shift them to
  // 0 -> 1 range.
  color_msg->r = normal.x() * 0.5 + 0.5;
  color_msg->g = normal.y() * 0.5 + 0.5;
  color_msg->b = normal.z() * 0.5 + 0.5;
  color_msg->a = 1.0;
}

inline void heightColorFromVertex(const Point& vertex,
                                  std_msgs::ColorRGBA* color_msg) {
  // TODO(helenol): figure out a nicer way to do this without hard-coded
  // constants.
  const double min_z = -1;
  const double max_z = 10;
  double mapped_height = std::min<FloatingPoint>(
      std::max<FloatingPoint>((vertex.z() - min_z) / (max_z - min_z), 0.0),
      1.0);
  colorVoxbloxToMsg(rainbowColorMap(mapped_height), color_msg);
}

inline void fillMarkerWithMesh(const MeshLayer::ConstPtr& mesh_layer,
                               ColorMode color_mode,
                               visualization_msgs::Marker* marker) {
  CHECK_NOTNULL(marker);
  marker->header.stamp = ros::Time::now();
  marker->ns = "mesh";
  marker->scale.x = 1;
  marker->scale.y = 1;
  marker->scale.z = 1;
  marker->pose.orientation.x = 0;
  marker->pose.orientation.y = 0;
  marker->pose.orientation.z = 0;
  marker->pose.orientation.w = 1;
  marker->type = visualization_msgs::Marker::TRIANGLE_LIST;

  BlockIndexList mesh_indices;
  mesh_layer->getAllAllocatedMeshes(&mesh_indices);

  for (const BlockIndex& block_index : mesh_indices) {
    Mesh::ConstPtr mesh = mesh_layer->getMeshPtrByIndex(block_index);

    if (!mesh->hasVertices()) {
      continue;
    }
    // Check that we can actually do the color stuff.
    if (color_mode == kColor) {
      CHECK(mesh->hasColors());
    }
    if (color_mode == kNormals || color_mode == kLambert) {
      CHECK(mesh->hasNormals());
    }

    for (size_t i = 0u; i < mesh->vertices.size(); i++) {
      geometry_msgs::Point point_msg;
      tf::pointEigenToMsg(mesh->vertices[i].cast<double>(), point_msg);
      marker->points.push_back(point_msg);
      std_msgs::ColorRGBA color_msg;
      switch (color_mode) {
        case kColor:
          colorVoxbloxToMsg(mesh->colors[i], &color_msg);
          break;
        case kHeight:
          heightColorFromVertex(mesh->vertices[i], &color_msg);
          break;
        case kNormals:
          normalColorFromNormal(mesh->normals[i], &color_msg);
          break;
        case kLambert:
          lambertColorFromNormal(mesh->normals[i], &color_msg);
          break;
        case kLambertColor:
          lambertColorFromColorAndNormal(mesh->colors[i], mesh->normals[i],
                                         &color_msg);
          break;
        case kGray:
          color_msg.r = color_msg.g = color_msg.b = 0.5;
          break;
      }
      color_msg.a = 1.0;
      marker->colors.push_back(color_msg);
    }
  }
}

}  // namespace voxblox

#endif  // VOXBLOX_ROS_MESH_VIS_H_

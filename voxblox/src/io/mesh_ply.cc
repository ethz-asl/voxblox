// NOTE: From open_chisel: github.com/personalrobotics/OpenChisel/
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
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "voxblox/io/mesh_ply.h"

namespace voxblox {

bool convertMeshLayerToMesh(const MeshLayer& mesh_layer, Mesh* mesh,
                            const bool connected_mesh,
                            const FloatingPoint vertex_proximity_threshold) {
  CHECK_NOTNULL(mesh);

  if (connected_mesh) {
    mesh_layer.getConnectedMesh(mesh, vertex_proximity_threshold);
  } else {
    mesh_layer.getMesh(mesh);
  }
  return mesh->size() > 0u;
}

bool outputMeshLayerAsPly(const std::string& filename,
                          const MeshLayer& mesh_layer) {
  constexpr bool kConnectedMesh = true;
  return outputMeshLayerAsPly(filename, kConnectedMesh, mesh_layer);
}

bool outputMeshLayerAsPly(const std::string& filename,
                          const bool connected_mesh,
                          const MeshLayer& mesh_layer) {
  Mesh combined_mesh(mesh_layer.block_size(), Point::Zero());

  if (!convertMeshLayerToMesh(mesh_layer, &combined_mesh, connected_mesh)) {
    return false;
  }

  bool success = outputMeshAsPly(filename, combined_mesh);
  if (!success) {
    LOG(WARNING) << "Saving to PLY failed!";
  }
  return success;
}

bool outputMeshAsPly(const std::string& filename, const Mesh& mesh) {
  std::ofstream stream(filename.c_str());

  if (!stream) {
    return false;
  }

  size_t num_points = mesh.vertices.size();
  stream << "ply" << std::endl;
  stream << "format ascii 1.0" << std::endl;
  stream << "element vertex " << num_points << std::endl;
  stream << "property float x" << std::endl;
  stream << "property float y" << std::endl;
  stream << "property float z" << std::endl;
  if (mesh.hasNormals()) {
    stream << "property float normal_x" << std::endl;
    stream << "property float normal_y" << std::endl;
    stream << "property float normal_z" << std::endl;
  }
  if (mesh.hasColors()) {
    stream << "property uchar red" << std::endl;
    stream << "property uchar green" << std::endl;
    stream << "property uchar blue" << std::endl;
    stream << "property uchar alpha" << std::endl;
  }
  if (mesh.hasTriangles()) {
    stream << "element face " << mesh.indices.size() / 3 << std::endl;
    stream << "property list uchar int vertex_indices"
           << std::endl;  // pcl-1.7(ros::kinetic) breaks ply convention by not
                          // using "vertex_index"
  }
  stream << "end_header" << std::endl;
  size_t vert_idx = 0;
  for (const Point& vert : mesh.vertices) {
    stream << vert(0) << " " << vert(1) << " " << vert(2);

    if (mesh.hasNormals()) {
      const Point& normal = mesh.normals[vert_idx];
      stream << " " << normal.x() << " " << normal.y() << " " << normal.z();
    }
    if (mesh.hasColors()) {
      const Color& color = mesh.colors[vert_idx];
      int r = static_cast<int>(color.r);
      int g = static_cast<int>(color.g);
      int b = static_cast<int>(color.b);
      int a = static_cast<int>(color.a);
      // Uint8 prints as character otherwise. :(
      stream << " " << r << " " << g << " " << b << " " << a;
    }

    stream << std::endl;
    vert_idx++;
  }
  if (mesh.hasTriangles()) {
    for (size_t i = 0; i < mesh.indices.size(); i += 3) {
      stream << "3 ";

      for (int j = 0; j < 3; j++) {
        stream << mesh.indices.at(i + j) << " ";
      }

      stream << std::endl;
    }
  }
  return true;
}

}  // namespace voxblox

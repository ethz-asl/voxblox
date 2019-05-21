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

#ifndef VOXBLOX_MESH_MESH_H_
#define VOXBLOX_MESH_MESH_H_

#include <cstdint>
#include <memory>

#include "voxblox/core/common.h"

namespace voxblox {

/**
 * Holds the vertex, normals, color and triangle index information of a mesh.
 */
struct Mesh {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<Mesh> Ptr;
  typedef std::shared_ptr<const Mesh> ConstPtr;

  static constexpr FloatingPoint kInvalidBlockSize = -1.0;

  Mesh()
      : block_size(kInvalidBlockSize), origin(Point::Zero()), updated(false) {
    // Do nothing.
  }

  Mesh(FloatingPoint _block_size, const Point& _origin)
      : block_size(_block_size), origin(_origin), updated(false) {
    CHECK_GT(block_size, 0.0);
  }
  virtual ~Mesh() {}

  inline bool hasVertices() const { return !vertices.empty(); }
  inline bool hasNormals() const { return !normals.empty(); }
  inline bool hasColors() const { return !colors.empty(); }
  inline bool hasTriangles() const { return !indices.empty(); }

  inline size_t size() const { return vertices.size(); }
  inline size_t getMemorySize() const {
    size_t size_bytes = 0u;
    size_bytes += sizeof(Pointcloud) + vertices.size() * sizeof(Point);
    size_bytes += sizeof(Pointcloud) + normals.size() * sizeof(Point);
    size_bytes += sizeof(Colors) + vertices.size() * sizeof(Color);
    size_bytes +=
        sizeof(VertexIndexList) + indices.size() * sizeof(VertexIndex);

    size_bytes += sizeof(block_size);
    size_bytes += sizeof(origin);
    size_bytes += sizeof(updated);
    return size_bytes;
  }

  inline void clear() {
    vertices.clear();
    normals.clear();
    colors.clear();
    indices.clear();
  }

  inline void clearTriangles() { indices.clear(); }
  inline void clearNormals() { normals.clear(); }
  inline void clearColors() { colors.clear(); }

  inline void resize(const size_t size, const bool has_normals = true,
                     const bool has_colors = true,
                     const bool has_indices = true) {
    vertices.resize(size);

    if (has_normals) {
      normals.resize(size);
    }

    if (has_colors) {
      colors.resize(size);
    }

    if (has_indices) {
      indices.resize(size);
    }
  }

  inline void reserve(const size_t size, const bool has_normals = true,
                      const bool has_colors = true,
                      const bool has_triangles = true) {
    vertices.reserve(size);

    if (has_normals) {
      normals.reserve(size);
    }

    if (has_colors) {
      colors.reserve(size);
    }

    if (has_triangles) {
      indices.reserve(size);
    }
  }

  void colorizeMesh(const Color& new_color) {
    colors.clear();
    colors.resize(vertices.size(), new_color);
  }

  void concatenateMesh(const Mesh& other_mesh) {
    CHECK_EQ(other_mesh.hasColors(), hasColors());
    CHECK_EQ(other_mesh.hasNormals(), hasNormals());
    CHECK_EQ(other_mesh.hasTriangles(), hasTriangles());

    reserve(size() + other_mesh.size(), hasNormals(), hasColors(),
            hasTriangles());

    const size_t num_vertices_before = vertices.size();

    for (const Point& vertex : other_mesh.vertices) {
      vertices.push_back(vertex);
    }
    for (const Color& color : other_mesh.colors) {
      colors.push_back(color);
    }
    for (const Point& normal : other_mesh.normals) {
      normals.push_back(normal);
    }
    for (const size_t index : other_mesh.indices) {
      indices.push_back(index + num_vertices_before);
    }
  }

  Pointcloud vertices;
  VertexIndexList indices;
  Pointcloud normals;
  Colors colors;

  FloatingPoint block_size;
  Point origin;

  bool updated;
};

}  // namespace voxblox

#endif  // VOXBLOX_MESH_MESH_H_

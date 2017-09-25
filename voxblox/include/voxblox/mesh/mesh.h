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

#include "voxblox/core/common.h"

namespace voxblox {

struct Mesh {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<Mesh> Ptr;
  typedef std::shared_ptr<const Mesh> ConstPtr;

  Mesh(FloatingPoint _block_size, const Point& _origin)
      : block_size(_block_size), origin(_origin), updated(false) {}
  virtual ~Mesh() {}

  inline bool hasVertices() const { return !vertices.empty(); }
  inline bool hasNormals() const { return !normals.empty(); }
  inline bool hasColors() const { return !colors.empty(); }
  inline bool hasIndices() const { return !indices.empty(); }

  inline void clear() {
    vertices.clear();
    normals.clear();
    colors.clear();
    indices.clear();
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

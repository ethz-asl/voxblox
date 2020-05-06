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

#ifndef VOXBLOX_IO_MESH_PLY_H_
#define VOXBLOX_IO_MESH_PLY_H_

#include <fstream>
#include <iostream>
#include <string>

#include "voxblox/mesh/mesh_layer.h"

namespace voxblox {

/**
 * Generates a mesh from the mesh layer.
 * @param connected_mesh if true veracities will be shared between triangles
 * @param vertex_proximity_threshold verticies that are within the specified
 * thershold distance will be merged together, simplifying the mesh.
 */
bool convertMeshLayerToMesh(
    const MeshLayer& mesh_layer, Mesh* mesh, const bool connected_mesh = true,
    const FloatingPoint vertex_proximity_threshold = 1e-10);

/// Default behaviour is to simplify the mesh.
bool outputMeshLayerAsPly(const std::string& filename,
                          const MeshLayer& mesh_layer);

/**
 * @param connected_mesh if true vertices will be shared between triangles
 */
bool outputMeshLayerAsPly(const std::string& filename,
                          const bool connected_mesh,
                          const MeshLayer& mesh_layer);

bool outputMeshAsPly(const std::string& filename, const Mesh& mesh);

}  // namespace voxblox

#endif  // VOXBLOX_IO_MESH_PLY_H_

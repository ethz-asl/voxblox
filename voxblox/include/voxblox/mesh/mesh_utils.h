#ifndef VOXBLOX_MESH_MESH_UTILS_H_
#define VOXBLOX_MESH_MESH_UTILS_H_

#include <vector>

#include "voxblox/core/block_hash.h"
#include "voxblox/core/common.h"
#include "voxblox/mesh/mesh.h"

namespace voxblox {

/**
 * Combines all given meshes into a single mesh with connected verticies. Also
 * removes triangles with zero surface area. If you only would like to connect
 * vertices, make sure that the proximity threhsold <<< voxel size. If you would
 * like to simplify the mesh, chose a threshold greater or near the voxel size
 * until you reached the level of simpliciation desired.
 */
inline void createConnectedMesh(
    const AlignedVector<Mesh::ConstPtr>& meshes, Mesh* connected_mesh,
    const FloatingPoint approximate_vertex_proximity_threshold = 1e-10) {
  CHECK_NOTNULL(connected_mesh);

  // Used to prevent double ups in vertices. We need to use a long long based
  // index, to prevent overflows.
  LongIndexHashMapType<size_t>::type uniques;

  const double threshold_inv =
      1. / static_cast<double>(approximate_vertex_proximity_threshold);

  // Combine everything in the layer into one giant combined mesh.
  size_t new_vertex_index = 0u;
  for (const Mesh::ConstPtr& mesh : meshes) {
    // Skip empty meshes.
    if (mesh->vertices.empty()) {
      continue;
    }

    // Make sure there are 3 distinct vertices for every triangle before
    // merging.
    CHECK_EQ(mesh->vertices.size(), mesh->indices.size());
    CHECK_EQ(mesh->vertices.size() % 3u, 0u);
    CHECK_EQ(mesh->indices.size() % 3u, 0u);

    // Stores the mapping from old vertex index to the new one in the combined
    // mesh. This is later used to adapt the triangles to the new, global
    // indexing of the combined mesh.
    std::vector<size_t> old_to_new_indices;
    old_to_new_indices.resize(mesh->vertices.size());

    size_t new_num_vertices_from_this_block = 0u;
    for (size_t old_vertex_idx = 0u; old_vertex_idx < mesh->vertices.size();
         ++old_vertex_idx) {
      // We scale the vertices by the inverse of the merging tolerance and
      // then compute a discretized grid index in that scale.
      // This exhibits the behaviour of merging two vertices that are
      // closer than the threshold.
      CHECK_LT(old_vertex_idx, mesh->vertices.size());
      const Point vertex = mesh->vertices[old_vertex_idx];
      const Eigen::Vector3d scaled_vector =
          vertex.cast<double>() * threshold_inv;
      const LongIndex vertex_3D_index = LongIndex(
          std::round(scaled_vector.x()), std::round(scaled_vector.y()),
          std::round(scaled_vector.z()));

      // If the current vertex falls into the same grid cell as a previous
      // vertex, we merge them. This is done by assigning the new vertex to
      // the same vertex index as the first vertex that fell into that cell.

      LongIndexHashMapType<size_t>::type::const_iterator it =
          uniques.find(vertex_3D_index);

      const bool vertex_is_unique_so_far = (it == uniques.end());
      if (vertex_is_unique_so_far) {
        // Copy vertex and associated data to combined mesh.
        connected_mesh->vertices.push_back(vertex);

        if (mesh->hasColors()) {
          CHECK_LT(old_vertex_idx, mesh->colors.size());
          connected_mesh->colors.push_back(mesh->colors[old_vertex_idx]);
        }
        if (mesh->hasNormals()) {
          CHECK_LT(old_vertex_idx, mesh->normals.size());
          connected_mesh->normals.push_back(mesh->normals[old_vertex_idx]);
        }

        // Store the new vertex index in the unique-vertex-map to be able to
        // retrieve this index later if we encounter vertices that are
        // supposed to be merged with this one.
        CHECK(uniques.emplace(vertex_3D_index, new_vertex_index).second);

        // Also store a mapping from old index to new index for this mesh
        // block to later adapt the triangle indexing.
        CHECK_LT(old_vertex_idx, old_to_new_indices.size());
        old_to_new_indices[old_vertex_idx] = new_vertex_index;

        ++new_vertex_index;
        ++new_num_vertices_from_this_block;
      } else {
        // If this vertex is not unique, we map it's vertex index to the new
        // vertex index.
        CHECK_LT(old_vertex_idx, old_to_new_indices.size());
        old_to_new_indices[old_vertex_idx] = it->second;

        // Add all normals (this will average them once they are renormalized
        // later)
        connected_mesh->normals[it->second] += mesh->normals[old_vertex_idx];
      }

      // Make sure the indexing is correct.
      CHECK_EQ(connected_mesh->vertices.size(), new_vertex_index);
    }

    // Renormalize normals
    for (Point& normal : connected_mesh->normals) {
      FloatingPoint length = normal.norm();
      if (length > kEpsilon) {
        normal /= length;
      } else {
        normal = Point(0.0f, 0.0f, 1.0f);
      }
    }

    // Make sure we have a mapping for every old vertex index.
    CHECK_EQ(old_to_new_indices.size(), mesh->vertices.size());

    // Append triangles and adjust their indices if necessary.
    // We discard triangles where all old vertices were mapped to the same
    // vertex.
    size_t new_num_triangle_from_this_block = 0u;
    for (size_t triangle_idx = 0u; triangle_idx < mesh->indices.size();
         triangle_idx += 3u) {
      CHECK_LT(triangle_idx + 2u, mesh->indices.size());

      // Retrieve old vertex indices.
      size_t vertex_0 = mesh->indices[triangle_idx];
      size_t vertex_1 = mesh->indices[triangle_idx + 1u];
      size_t vertex_2 = mesh->indices[triangle_idx + 2u];

      // Make sure the old indices were valid before remapping.
      CHECK_LT(vertex_0, old_to_new_indices.size());
      CHECK_LT(vertex_1, old_to_new_indices.size());
      CHECK_LT(vertex_2, old_to_new_indices.size());

      // Apply vertex index mapping.
      vertex_0 = old_to_new_indices[vertex_0];
      vertex_1 = old_to_new_indices[vertex_1];
      vertex_2 = old_to_new_indices[vertex_2];

      // Make sure the new indices are valid after remapping.
      CHECK_LT(vertex_0, new_vertex_index);
      CHECK_LT(vertex_1, new_vertex_index);
      CHECK_LT(vertex_2, new_vertex_index);

      // Get rid of triangles where all two or three vertices have been
      // merged.
      const bool two_or_three_vertex_indices_are_the_same =
          (vertex_0 == vertex_1) || (vertex_1 == vertex_2) ||
          (vertex_0 == vertex_2);

      if (!two_or_three_vertex_indices_are_the_same) {
        connected_mesh->indices.push_back(vertex_0);
        connected_mesh->indices.push_back(vertex_1);
        connected_mesh->indices.push_back(vertex_2);
        ++new_num_triangle_from_this_block;
      }
    }
  }

  // Verify combined mesh.
  if (connected_mesh->hasColors()) {
    CHECK_EQ(connected_mesh->vertices.size(), connected_mesh->colors.size());
  }
  if (connected_mesh->hasNormals()) {
    CHECK_EQ(connected_mesh->vertices.size(), connected_mesh->normals.size());
  }
}

inline void createConnectedMesh(
    const Mesh& mesh, Mesh* connected_mesh,
    const FloatingPoint approximate_vertex_proximity_threshold = 1e-10) {
  AlignedVector<Mesh::ConstPtr> meshes;
  meshes.push_back(Mesh::ConstPtr(&mesh, [](Mesh const*) {}));
  createConnectedMesh(meshes, connected_mesh,
                      approximate_vertex_proximity_threshold);
}

};  // namespace voxblox

#endif  // VOXBLOX_MESH_MESH_UTILS_H_

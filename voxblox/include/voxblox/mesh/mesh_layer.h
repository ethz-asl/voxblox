#ifndef VOXBLOX_MESH_MESH_LAYER_H_
#define VOXBLOX_MESH_MESH_LAYER_H_

#include <cmath>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include <glog/logging.h>

#include "voxblox/core/block_hash.h"
#include "voxblox/core/common.h"
#include "voxblox/mesh/mesh.h"

namespace voxblox {

// A special type of layer just for containing the mesh. Same general interface
// as a layer of blocks, but only contains a single thing, not a set of voxels.
class MeshLayer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<MeshLayer> Ptr;
  typedef std::shared_ptr<const MeshLayer> ConstPtr;
  typedef typename AnyIndexHashMapType<Mesh::Ptr>::type MeshMap;

  explicit MeshLayer(FloatingPoint block_size) : block_size_(block_size) {}
  virtual ~MeshLayer() {}

  // By index.
  inline const Mesh& getMeshByIndex(const BlockIndex& index) const {
    typename MeshMap::const_iterator it = mesh_map_.find(index);
    if (it != mesh_map_.end()) {
      return *(it->second);
    } else {
      LOG(FATAL) << "Accessed unallocated mesh at " << index.transpose();
    }
  }

  inline Mesh& getMeshByIndex(const BlockIndex& index) {
    typename MeshMap::iterator it = mesh_map_.find(index);
    if (it != mesh_map_.end()) {
      return *(it->second);
    } else {
      LOG(FATAL) << "Accessed unallocated mesh at " << index.transpose();
    }
  }

  inline typename Mesh::ConstPtr getMeshPtrByIndex(
      const BlockIndex& index) const {
    typename MeshMap::const_iterator it = mesh_map_.find(index);
    if (it != mesh_map_.end()) {
      return it->second;
    } else {
      LOG(WARNING) << "Returning null ptr to mesh!";
      return typename Mesh::ConstPtr();
    }
  }

  inline typename Mesh::Ptr getMeshPtrByIndex(const BlockIndex& index) {
    typename MeshMap::iterator it = mesh_map_.find(index);
    if (it != mesh_map_.end()) {
      return it->second;
    } else {
      LOG(WARNING) << "Returning null ptr to mesh!";
      return typename Mesh::Ptr();
    }
  }

  // Gets a mesh by the mesh index it if already exists,
  // otherwise allocates a new one.
  inline typename Mesh::Ptr allocateMeshPtrByIndex(const BlockIndex& index) {
    typename MeshMap::iterator it = mesh_map_.find(index);
    if (it != mesh_map_.end()) {
      return it->second;
    } else {
      return allocateNewBlock(index);
    }
  }

  inline typename Mesh::ConstPtr getMeshPtrByCoordinates(
      const Point& coords) const {
    return getMeshPtrByIndex(computeBlockIndexFromCoordinates(coords));
  }

  inline typename Mesh::Ptr getMeshPtrByCoordinates(const Point& coords) {
    return getMeshPtrByIndex(computeBlockIndexFromCoordinates(coords));
  }

  // Gets a mesh by the coordinates it if already exists,
  // otherwise allocates a new one.
  inline typename Mesh::Ptr allocateMeshPtrByCoordinates(const Point& coords) {
    return allocateMeshPtrByIndex(computeBlockIndexFromCoordinates(coords));
  }

  // Coord to mesh index.
  inline BlockIndex computeBlockIndexFromCoordinates(
      const Point& coords) const {
    return getGridIndexFromPoint<BlockIndex>(coords, block_size_inv_);
  }

  // Pure virtual function -- inheriting class MUST overwrite.
  typename Mesh::Ptr allocateNewBlock(const BlockIndex& index) {
    auto insert_status = mesh_map_.insert(std::make_pair(
        index, std::shared_ptr<Mesh>(new Mesh(
                   block_size_, index.cast<FloatingPoint>() * block_size_))));
    DCHECK(insert_status.second)
        << "Mesh already exists when allocating at " << index.transpose();
    DCHECK(insert_status.first->second);
    DCHECK_EQ(insert_status.first->first, index);
    return insert_status.first->second;
  }

  inline typename Mesh::Ptr allocateNewBlockByCoordinates(const Point& coords) {
    return allocateNewBlock(computeBlockIndexFromCoordinates(coords));
  }

  void removeMesh(const BlockIndex& index) { mesh_map_.erase(index); }

  void removeMeshByCoordinates(const Point& coords) {
    mesh_map_.erase(computeBlockIndexFromCoordinates(coords));
  }

  void clearDistantMesh(const Point& center, const double max_distance) {
    // we clear the mesh, but do not delete it from the map as the empty mesh
    // must be sent to rviz so it is also cleared there
    for (std::pair<const BlockIndex, typename Mesh::Ptr>& kv : mesh_map_) {
      if ((kv.second->origin - center).squaredNorm() >
          max_distance * max_distance) {
        kv.second->clear();
        kv.second->updated = true;
      }
    }
  }

  void getAllAllocatedMeshes(BlockIndexList* meshes) const {
    meshes->clear();
    meshes->reserve(mesh_map_.size());
    for (const std::pair<const BlockIndex, typename Mesh::Ptr>& kv :
         mesh_map_) {
      meshes->emplace_back(kv.first);
    }
  }

  void getAllUpdatedMeshes(BlockIndexList* meshes) const {
    meshes->clear();
    for (const std::pair<const BlockIndex, typename Mesh::Ptr>& kv :
         mesh_map_) {
      if (kv.second->updated) {
        meshes->emplace_back(kv.first);
      }
    }
  }

  // Get mesh from mesh layer. NOTE: The triangles and vertices in this mesh are
  // distinct, hence, this will not produce a connected mesh.
  void getMesh(Mesh* combined_mesh) const {
    CHECK_NOTNULL(combined_mesh);

    // Combine everything in the layer into one giant combined mesh.

    BlockIndexList mesh_indices;
    getAllAllocatedMeshes(&mesh_indices);

    // Check if color, normals and indices are enabled for the first non-empty
    // mesh. If they are, they need to be enabled for all other ones as well.
    bool has_colors = false;
    bool has_normals = false;
    bool has_indices = false;
    if (!mesh_indices.empty()) {
      for (const BlockIndex& block_index : mesh_indices) {
        Mesh::ConstPtr mesh = getMeshPtrByIndex(block_index);
        if (!mesh->vertices.empty()) {
          has_colors = mesh->hasColors();
          has_normals = mesh->hasNormals();
          has_indices = mesh->hasTriangles();
          break;
        }
      }
    }

    // Loop again over all meshes to figure out how big the mesh needs to be.
    size_t mesh_size = 0;
    for (const BlockIndex& block_index : mesh_indices) {
      Mesh::ConstPtr mesh = getMeshPtrByIndex(block_index);
      mesh_size += mesh->vertices.size();
    }

    // Reserve space for the mesh.
    combined_mesh->reserve(mesh_size, has_normals, has_colors, has_indices);

    size_t new_index = 0u;
    for (const BlockIndex& block_index : mesh_indices) {
      Mesh::ConstPtr mesh = getMeshPtrByIndex(block_index);

      // Check assumption that all meshes have same configuration regarding
      // colors, normals and indices.
      if (!mesh->vertices.empty()) {
        CHECK_EQ(has_colors, mesh->hasColors());
        CHECK_EQ(has_normals, mesh->hasNormals());
        CHECK_EQ(has_indices, mesh->hasTriangles());
      }

      // Copy the mesh content into the combined mesh. This is done in triplets
      // for readability only, as one loop iteration will then copy one
      // triangle.
      for (size_t i = 0; i < mesh->vertices.size(); i += 3, new_index += 3) {
        CHECK_LT(new_index + 2, mesh_size);

        combined_mesh->vertices.push_back(mesh->vertices[i]);
        combined_mesh->vertices.push_back(mesh->vertices[i + 1]);
        combined_mesh->vertices.push_back(mesh->vertices[i + 2]);

        if (has_colors) {
          combined_mesh->colors.push_back(mesh->colors[i]);
          combined_mesh->colors.push_back(mesh->colors[i + 1]);
          combined_mesh->colors.push_back(mesh->colors[i + 2]);
        }
        if (has_normals) {
          combined_mesh->normals.push_back(mesh->normals[i]);
          combined_mesh->normals.push_back(mesh->normals[i + 1]);
          combined_mesh->normals.push_back(mesh->normals[i + 2]);
        }
        if (has_indices) {
          combined_mesh->indices.push_back(new_index);
          combined_mesh->indices.push_back(new_index + 1);
          combined_mesh->indices.push_back(new_index + 2);
        }
      }
    }

    // Verify combined mesh.
    if (combined_mesh->hasColors()) {
      CHECK_EQ(combined_mesh->vertices.size(), combined_mesh->colors.size());
    }
    if (combined_mesh->hasNormals()) {
      CHECK_EQ(combined_mesh->vertices.size(), combined_mesh->normals.size());
    }

    CHECK_EQ(combined_mesh->vertices.size(), combined_mesh->indices.size());
  }

  // Get a connected mesh by merging close vertices and removing triangles with
  // zero surface area. If you only would like to connect vertices, make sure
  // that the proximity threhsold <<< voxel size. If you would like to simplify
  // the mesh, chose a threshold greater or near the voxel size until you
  // reached the level of simpliciation desired.
  void getConnectedMesh(
      Mesh* combined_mesh,
      const FloatingPoint approximate_vertex_proximity_threshold =
          1e-10) const {
    CHECK_NOTNULL(combined_mesh);

    // Used to prevent double ups in vertices. We need to use a long long based
    // index, to prevent overflows.
    LongIndexHashMapType<size_t>::type uniques;

    const double threshold_inv =
        1. / static_cast<double>(approximate_vertex_proximity_threshold);

    // Combine everything in the layer into one giant combined mesh.
    size_t new_vertex_index = 0u;
    BlockIndexList mesh_indices;
    getAllAllocatedMeshes(&mesh_indices);
    for (const BlockIndex& block_index : mesh_indices) {
      Mesh::ConstPtr mesh = getMeshPtrByIndex(block_index);

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
          combined_mesh->vertices.push_back(vertex);

          if (mesh->hasColors()) {
            CHECK_LT(old_vertex_idx, mesh->colors.size());
            combined_mesh->colors.push_back(mesh->colors[old_vertex_idx]);
          }
          if (mesh->hasNormals()) {
            CHECK_LT(old_vertex_idx, mesh->normals.size());
            combined_mesh->normals.push_back(mesh->normals[old_vertex_idx]);
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
        }

        // Make sure the indexing is correct.
        CHECK_EQ(combined_mesh->vertices.size(), new_vertex_index);
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
          combined_mesh->indices.push_back(vertex_0);
          combined_mesh->indices.push_back(vertex_1);
          combined_mesh->indices.push_back(vertex_2);
          ++new_num_triangle_from_this_block;
        }
      }
    }

    // Verify combined mesh.
    if (combined_mesh->hasColors()) {
      CHECK_EQ(combined_mesh->vertices.size(), combined_mesh->colors.size());
    }
    if (combined_mesh->hasNormals()) {
      CHECK_EQ(combined_mesh->vertices.size(), combined_mesh->normals.size());
    }

    // The number of vertices should now be less or equal the amount of triangle
    // indices, since we merged some of the vertices.
    CHECK_LE(combined_mesh->vertices.size(), combined_mesh->indices.size());
  }

  size_t getNumberOfAllocatedMeshes() const { return mesh_map_.size(); }
  // Deletes ALL parts of the mesh.
  void clear() { mesh_map_.clear(); }

  FloatingPoint block_size() const { return block_size_; }

 private:
  FloatingPoint block_size_;

  // Derived types.
  FloatingPoint block_size_inv_;

  MeshMap mesh_map_;
};

}  // namespace voxblox

#endif  // VOXBLOX_MESH_MESH_LAYER_H_

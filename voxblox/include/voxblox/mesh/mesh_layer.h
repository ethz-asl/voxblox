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
#include "voxblox/mesh/mesh_utils.h"

namespace voxblox {

/**
 * A special type of layer just for containing the mesh. Same general interface
 * as a layer of blocks, but only contains a single thing, not a set of voxels.
 */
class MeshLayer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<MeshLayer> Ptr;
  typedef std::shared_ptr<const MeshLayer> ConstPtr;
  typedef typename AnyIndexHashMapType<Mesh::Ptr>::type MeshMap;

  explicit MeshLayer(FloatingPoint block_size)
      : block_size_(block_size), block_size_inv_(1.0 / block_size) {}
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

  /**
   * Gets a mesh by the mesh index it if already exists,
   * otherwise allocates a new one.
   */
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

  /**
   * Gets a mesh by the coordinates it if already exists,
   * otherwise allocates a new one.
   */
  inline typename Mesh::Ptr allocateMeshPtrByCoordinates(const Point& coords) {
    return allocateMeshPtrByIndex(computeBlockIndexFromCoordinates(coords));
  }

  /// Coord to mesh index.
  inline BlockIndex computeBlockIndexFromCoordinates(
      const Point& coords) const {
    return getGridIndexFromPoint<BlockIndex>(coords, block_size_inv_);
  }

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

  /**
   * Get mesh from mesh layer. NOTE: The triangles and vertices in this mesh are
   * distinct, hence, this will not produce a connected mesh.
   */
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

  /**
   * Get a connected mesh by merging close vertices and removing triangles with
   * zero surface area. If you only would like to connect vertices, make sure
   * that the proximity threhsold <<< voxel size. If you would like to simplify
   * the mesh, chose a threshold greater or near the voxel size until you
   * reached the level of simpliciation desired.
   */
  void getConnectedMesh(
      Mesh* connected_mesh,
      const FloatingPoint approximate_vertex_proximity_threshold =
          1e-10) const {
    BlockIndexList mesh_indices;
    getAllAllocatedMeshes(&mesh_indices);

    AlignedVector<Mesh::ConstPtr> meshes;
    meshes.reserve(mesh_indices.size());
    for (const BlockIndex& block_index : mesh_indices) {
      meshes.push_back(getMeshPtrByIndex(block_index));
    }

    createConnectedMesh(meshes, connected_mesh,
                        approximate_vertex_proximity_threshold);
  }

  size_t getNumberOfAllocatedMeshes() const { return mesh_map_.size(); }

  inline size_t getMemorySize() const {
    size_t size_bytes = 0u;

    // Calculate size of members
    size_bytes += sizeof(block_size_);
    size_bytes += sizeof(block_size_inv_);

    // Calculate size of mesh blocks
    for (const auto& idx_mesh_pair : mesh_map_) {
      CHECK(idx_mesh_pair.second);
      size_bytes += idx_mesh_pair.second->getMemorySize();
    }
    return size_bytes;
  }

  /// Deletes ALL parts of the mesh.
  void clear() { mesh_map_.clear(); }

  FloatingPoint block_size() const { return block_size_; }

  FloatingPoint block_size_inv() const { return block_size_inv_; }

 private:
  FloatingPoint block_size_;

  // Derived types.
  FloatingPoint block_size_inv_;

  MeshMap mesh_map_;
};

}  // namespace voxblox

#endif  // VOXBLOX_MESH_MESH_LAYER_H_

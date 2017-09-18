#ifndef VOXBLOX_CORE_MESH_LAYER_H_
#define VOXBLOX_CORE_MESH_LAYER_H_

#include <glog/logging.h>
#include <utility>

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
    return getGridIndexFromPoint(coords, block_size_inv_);
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

  void combineMesh(Mesh::Ptr combined_mesh) const {
    // Used to prevent double ups in vertices
    AnyIndexHashMapType<IndexElement>::type uniques;

    // Some triangles will have zero area we store them here first then filter
    // them
    VertexIndexList temp_indices;

    // If two vertexes are closer together than (voxel_size /
    // key_multiplication_factor), then the second vertex will be discarded and
    // the first one used in its place
    constexpr FloatingPoint key_multiplication_factor = 10;

    // Combine everything in the layer into one giant combined mesh.
    size_t v = 0;
    BlockIndexList mesh_indices;
    getAllAllocatedMeshes(&mesh_indices);
    for (const BlockIndex& block_index : mesh_indices) {
      Mesh::ConstPtr mesh = getMeshPtrByIndex(block_index);

      for (size_t i = 0; i < mesh->vertices.size(); ++i) {
        // convert from 3D point to key
        BlockIndex vert_key =
            (key_multiplication_factor * mesh->vertices[i] / block_size())
                .cast<IndexElement>();
        if (uniques.find(vert_key) == uniques.end()) {
          uniques[vert_key] = v;
          combined_mesh->vertices.push_back(mesh->vertices[i]);

          if (mesh->hasColors()) {
            combined_mesh->colors.push_back(mesh->colors[i]);
          }
          if (mesh->hasNormals()) {
            combined_mesh->normals.push_back(mesh->normals[i]);
          }

          temp_indices.push_back(v);
          v++;
        } else {
          temp_indices.push_back(uniques[vert_key]);
        }
      }
    }

    // extract indices of triangles with non-zero area
    for (size_t i = 0; i < temp_indices.size(); i += 3) {
      // check that corners of triangles have not been merged
      if ((temp_indices[i] != temp_indices[i + 1]) &&
          (temp_indices[i] != temp_indices[i + 2]) &&
          (temp_indices[i + 1] != temp_indices[i + 2])) {
        combined_mesh->indices.push_back(temp_indices[i]);
        combined_mesh->indices.push_back(temp_indices[i + 1]);
        combined_mesh->indices.push_back(temp_indices[i + 2]);
      }
    }
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

#endif  // VOXBLOX_CORE_MESH_LAYER_H_

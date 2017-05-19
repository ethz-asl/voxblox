#ifndef VOXBLOX_FAST_CORE_MESH_LAYER_H_
#define VOXBLOX_FAST_CORE_MESH_LAYER_H_

#include <glog/logging.h>
#include <utility>

#include "voxblox_fast/core/block_hash.h"
#include "voxblox_fast/core/common.h"
#include "voxblox_fast/mesh/mesh.h"

namespace voxblox_fast {

// A special type of layer just for containing the mesh. Same general interface
// as a layer of blocks, but only contains a single thing, not a set of voxels.
class MeshLayer {
 public:
  typedef std::shared_ptr<MeshLayer> Ptr;
  typedef std::shared_ptr<const MeshLayer> ConstPtr;
  typedef typename BlockHashMapType<Mesh::Ptr>::type MeshMap;

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
    DCHECK(insert_status.second) << "Mesh already exists when allocating at "
                                 << index.transpose();
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

#endif  // VOXBLOX_FAST_CORE_MESH_LAYER_H_

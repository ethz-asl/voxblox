#ifndef VOXBLOX_CORE_MAP_H
#define VOXBLOX_CORE_MAP_H

#include <utility>

#include "voxblox/core/common.h"
#include "voxblox/core/block.h"
#include "voxblox/core/block_hash.h"

namespace voxblox {

struct MapConfig {};

template <typename BlockType>
class Map {
 public:
  typedef typename BlockHashMapType<typename BlockType::Ptr>::type BlockHashMap;

  virtual ~Map() {}

  // By index.
  inline const BlockType& getBlockByIndex(const BlockIndex& index) const {
    typename BlockHashMap::const_iterator it = block_map_.find(index);
    if (it != block_map_.end()) {
      return *(it->second);
    } else {
      LOG(FATAL) << "Accessed unallocated block at " << index.transpose();
    }
  }

  inline BlockType& getBlockByIndex(const BlockIndex& index) {
    typename BlockHashMap::iterator it = block_map_.find(index);
    if (it != block_map_.end()) {
      return *(it->second);
    } else {
      LOG(FATAL) << "Accessed unallocated block at " << index.transpose();
    }
  }

  inline typename BlockType::ConstPtr getBlockPtrByIndex(
      const BlockIndex& index) const {
    typename BlockHashMap::const_iterator it = block_map_.find(index);
    if (it != block_map_.end()) {
      return it->second;
    } else {
      return typename BlockType::ConstPtr();
    }
  }

  inline typename BlockType::Ptr getBlockPtrByIndex(const BlockIndex& index) {
    typename BlockHashMap::iterator it = block_map_.find(index);
    if (it != block_map_.end()) {
      return it->second;
    } else {
      return typename BlockType::Ptr();
    }
  }

  // Gets a block by the block index it if already exists, otherwise allocates a
  // new one.
  inline typename BlockType::Ptr allocateBlockPtrByIndex(
      const BlockIndex& index) {
    typename BlockHashMap::iterator it = block_map_.find(index);
    if (it != block_map_.end()) {
      return it->second;
    } else {
      return allocateNewBlock(index);
    }
  }

  inline typename BlockType::ConstPtr getBlockPtrByCoordinates(
      const Coordinates& coords) const {
    return getBlockPtrByIndex(computeBlockIndexFromCoordinates(coords));
  }

  inline typename BlockType::Ptr getBlockPtrByCoordinates(
      const Coordinates& coords) {
    return getBlockPtrByIndex(computeBlockIndexFromCoordinates(coords));
  }

  // Gets a block by the coordinates it if already exists, otherwise allocates a
  // new one.
  inline typename BlockType::Ptr allocateBlockPtrByCoords(
      const Coordinates& coords) {
    return allocateBlockPtrByIndex(computeBlockIndexFromCoordinates(coords));
  }

  // Coord to block index.
  inline BlockIndex computeBlockIndexFromCoordinates(
      const Coordinates& coords) const {
    return VoxelIndex(
        static_cast<int>(std::floor(coords.x() * block_size_inv_)),
        static_cast<int>(std::floor(coords.y() * block_size_inv_)),
        static_cast<int>(std::floor(coords.z() * block_size_inv_)));
  }

  // Pure virtual function -- inheriting class MUST overwrite.
  virtual typename BlockType::Ptr allocateNewBlock(const BlockIndex& index) = 0;

  typename BlockType::Ptr allocateNewBlockByCoordinates(
      const Coordinates& coords) {
    allocateNewBlock(computeBlockIndexFromCoordinates(coords));
  }

  void removeBlock(const BlockIndex& index) { block_map_.erase(index); }
  void removeBlockByCoordinates(const Coordinates& coords) {
    block_map_.erase(computeBlockIndexFromCoordinates(coords));
  }

 protected:
  Map(FloatingPoint block_size) : block_size_(block_size) {
    block_size_inv_ = 1.0 / block_size_;
  }

  FloatingPoint block_size_;

  // Derived types.
  FloatingPoint block_size_inv_;

  BlockHashMap block_map_;
};

class TsdfMap : public Map<TsdfBlock> {
 public:
  TsdfMap(size_t voxels_per_side, FloatingPoint voxel_size)
      : Map(voxel_size * voxels_per_side),
        voxel_size_(voxel_size),
        voxels_per_side_(voxels_per_side) {}

  virtual TsdfBlock::Ptr allocateNewBlock(const BlockIndex& index) {
    auto my_iter = block_map_.find(index);
    auto insert_status = block_map_.emplace(
        index,
        std::make_shared<TsdfBlock>(index.cast<FloatingPoint>() * block_size_,
                                    voxels_per_side_, voxel_size_));
    CHECK(insert_status.second) << "Block already exists when allocating at "
                                << index.transpose();
    return insert_status.first->second;
  }

 protected:
  size_t voxels_per_side_;
  FloatingPoint voxel_size_;
};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_MAP_H

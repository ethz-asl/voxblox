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
  typedef std::shared_ptr<Map> Ptr;

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
      LOG(WARNING) << "Returning null ptr to block!";
      return typename BlockType::ConstPtr();
    }
  }

  inline typename BlockType::Ptr getBlockPtrByIndex(const BlockIndex& index) {
    typename BlockHashMap::iterator it = block_map_.find(index);
    if (it != block_map_.end()) {
      return it->second;
    } else {
      LOG(WARNING) << "Returning null ptr to block!";
      return typename BlockType::Ptr();
    }
  }

  // Gets a block by the block index it if already exists, otherwise allocates a
  // new one.
  inline typename BlockType::Ptr allocateBlockPtrByIndex(
      const BlockIndex& index) {
    typename BlockHashMap::iterator it = block_map_.find(index);
    if (it != block_map_.end()) {
      //LOG(INFO) << "Access block index " << index.transpose();
      return it->second;
    } else {
      //LOG(INFO) << "Allocate block index " << index.transpose();
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
    return floorVectorAndDowncast(coords * block_size_inv_);
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

  // Accessor functions for all allocated blocks.
  void getAllAllocatedBlocks(BlockIndexList* blocks) const {
    blocks->clear();
    blocks->reserve(block_map_.size());
    for (const std::pair<const BlockIndex, typename BlockType::Ptr>& kv :
         block_map_) {
      blocks->emplace_back(kv.first);
    }
  }

  size_t getNumberOfAllocatedBlocks() const { return block_map_.size(); }

  FloatingPoint getBlockSize() const { return block_size_; }

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
  typedef std::shared_ptr<TsdfMap> Ptr;

  TsdfMap(size_t voxels_per_side, FloatingPoint voxel_size)
      : Map(voxel_size * voxels_per_side),
        voxel_size_(voxel_size),
        voxels_per_side_(voxels_per_side) {}

  virtual TsdfBlock::Ptr allocateNewBlock(const BlockIndex& index) {
    auto insert_status = block_map_.insert(
        std::make_pair(index, std::shared_ptr<TsdfBlock>(new TsdfBlock(
                                  index.cast<FloatingPoint>() * block_size_,
                                  voxels_per_side_, voxel_size_))));
    DCHECK(insert_status.second) << "Block already exists when allocating at "
                                << index.transpose();
    DCHECK(insert_status.first->second != nullptr) << "Second is null!";
    DCHECK(insert_status.first->first == index) << "Added to wrong index!";
    return insert_status.first->second;
  }

  // This refers to the main layer (TSDF layer in this case):
  size_t getVoxelsPerBlock() const {
    return voxels_per_side_ * voxels_per_side_ * voxels_per_side_;
  }

  FloatingPoint getVoxelSize() const { return voxel_size_; }

  size_t getVoxelsPerSide() const { return voxels_per_side_; }

 protected:
  size_t voxels_per_side_;
  FloatingPoint voxel_size_;
};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_MAP_H

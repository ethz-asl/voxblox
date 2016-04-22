#ifndef VOXBLOX_CORE_MAP_H_
#define VOXBLOX_CORE_MAP_H_

#include <glog/logging.h>
#include <utility>

#include "voxblox/core/common.h"
#include "voxblox/core/block.h"
#include "voxblox/core/block_hash.h"

namespace voxblox {

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

  // Gets a block by the block index it if already exists,
  // otherwise allocates a new one.
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
      const Point& coords) const {
    return getBlockPtrByIndex(computeBlockIndexFromCoordinates(coords));
  }

  inline typename BlockType::Ptr getBlockPtrByCoordinates(const Point& coords) {
    return getBlockPtrByIndex(computeBlockIndexFromCoordinates(coords));
  }

  // Gets a block by the coordinates it if already exists,
  // otherwise allocates a new one.
  inline typename BlockType::Ptr allocateBlockPtrByCoordinates(
      const Point& coords) {
    return allocateBlockPtrByIndex(computeBlockIndexFromCoordinates(coords));
  }

  // Coord to block index.
  inline BlockIndex computeBlockIndexFromCoordinates(const Point& coords)
      const {
    return floorVectorAndDowncast(coords * block_size_inv_);
  }

  // Pure virtual function -- inheriting class MUST overwrite.
  virtual typename BlockType::Ptr allocateNewBlock(const BlockIndex& index) = 0;

  inline typename BlockType::Ptr allocateNewBlockByCoordinates(
      const Point& coords) {
    allocateNewBlock(computeBlockIndexFromCoordinates(coords));
  }

  void removeBlock(const BlockIndex& index) { block_map_.erase(index); }

  void removeBlockByCoordinates(const Point& coords) {
    block_map_.erase(computeBlockIndexFromCoordinates(coords));
  }

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
  explicit Map(FloatingPoint block_size) : block_size_(block_size) {
    block_size_inv_ = 1.0 / block_size_;
  }

  FloatingPoint block_size_;

  // Derived types.
  FloatingPoint block_size_inv_;

  BlockHashMap block_map_;
};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_MAP_H_

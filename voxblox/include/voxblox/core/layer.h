#ifndef VOXBLOX_CORE_LAYER_H_
#define VOXBLOX_CORE_LAYER_H_

#include <memory>
#include <string>
#include <utility>

#include <glog/logging.h>

#include "voxblox/Block.pb.h"
#include "voxblox/Layer.pb.h"
#include "voxblox/core/block.h"
#include "voxblox/core/block_hash.h"
#include "voxblox/core/common.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

/**
 * A 3D information layer, containing data of type VoxelType stored in blocks.
 * This class contains functions for manipulating and accessing these blocks.
 */
template <typename VoxelType>
class Layer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<Layer> Ptr;
  typedef Block<VoxelType> BlockType;
  typedef
      typename AnyIndexHashMapType<typename BlockType::Ptr>::type BlockHashMap;
  typedef typename std::pair<BlockIndex, typename BlockType::Ptr> BlockMapPair;

  explicit Layer(FloatingPoint voxel_size, size_t voxels_per_side)
      : voxel_size_(voxel_size), voxels_per_side_(voxels_per_side) {
    CHECK_GT(voxel_size_, 0.0f);
    voxel_size_inv_ = 1.0 / voxel_size_;

    block_size_ = voxel_size_ * voxels_per_side_;
    CHECK_GT(block_size_, 0.0f);
    block_size_inv_ = 1.0 / block_size_;
    CHECK_GT(voxels_per_side_, 0u);
    voxels_per_side_inv_ = 1.0f / static_cast<FloatingPoint>(voxels_per_side_);
  }

  /// Create the layer from protobuf layer header.
  explicit Layer(const LayerProto& proto);

  /// Deep copy constructor.
  explicit Layer(const Layer& other);

  virtual ~Layer() {}

  enum class BlockMergingStrategy { kProhibit, kReplace, kDiscard, kMerge };

  inline const BlockType& getBlockByIndex(const BlockIndex& index) const {
    typename BlockHashMap::const_iterator it = block_map_.find(index);
    if (it == block_map_.end()) {
      LOG(FATAL) << "Accessed unallocated block at " << index.transpose();
    }
    return *(it->second);
  }

  inline BlockType& getBlockByIndex(const BlockIndex& index) {
    typename BlockHashMap::iterator it = block_map_.find(index);
    if (it == block_map_.end()) {
      LOG(FATAL) << "Accessed unallocated block at " << index.transpose();
    }
    return *(it->second);
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

  /**
   *  Gets a block by the block index it if already exists, otherwise allocates
   * a new one.
   */
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

  /**
   * Gets a block by the coordinates it if already exists,
   * otherwise allocates a new one.
   */
  inline typename BlockType::Ptr allocateBlockPtrByCoordinates(
      const Point& coords) {
    return allocateBlockPtrByIndex(computeBlockIndexFromCoordinates(coords));
  }

  /**
   * IMPORTANT NOTE: Due the limited accuracy of the FloatingPoint type, this
   * function doesn't always compute the correct block index for coordinates
   * near the block boundaries.
   */
  inline BlockIndex computeBlockIndexFromCoordinates(
      const Point& coords) const {
    return getGridIndexFromPoint<BlockIndex>(coords, block_size_inv_);
  }

  typename BlockType::Ptr allocateNewBlock(const BlockIndex& index) {
    auto insert_status = block_map_.emplace(
        index, std::make_shared<BlockType>(
                   voxels_per_side_, voxel_size_,
                   getOriginPointFromGridIndex(index, block_size_)));

    DCHECK(insert_status.second)
        << "Block already exists when allocating at " << index.transpose();

    DCHECK(insert_status.first->second);
    DCHECK_EQ(insert_status.first->first, index);
    return insert_status.first->second;
  }

  inline typename BlockType::Ptr allocateNewBlockByCoordinates(
      const Point& coords) {
    return allocateNewBlock(computeBlockIndexFromCoordinates(coords));
  }

  inline void insertBlock(
      const std::pair<const BlockIndex, typename Block<VoxelType>::Ptr>&
          block_pair) {
    auto insert_status = block_map_.insert(block_pair);

    DCHECK(insert_status.second) << "Block already exists when inserting at "
                                 << insert_status.first->first.transpose();

    DCHECK(insert_status.first->second);
  }

  void removeBlock(const BlockIndex& index) { block_map_.erase(index); }
  void removeAllBlocks() { block_map_.clear(); }

  void removeBlockByCoordinates(const Point& coords) {
    block_map_.erase(computeBlockIndexFromCoordinates(coords));
  }

  void removeDistantBlocks(const Point& center, const double max_distance) {
    AlignedVector<BlockIndex> needs_erasing;
    for (const std::pair<const BlockIndex, typename BlockType::Ptr>& kv :
         block_map_) {
      if ((kv.second->origin() - center).squaredNorm() >
          max_distance * max_distance) {
        needs_erasing.push_back(kv.first);
      }
    }
    for (const BlockIndex& index : needs_erasing) {
      block_map_.erase(index);
    }
  }

  void getAllAllocatedBlocks(BlockIndexList* blocks) const {
    CHECK_NOTNULL(blocks);
    blocks->clear();
    blocks->reserve(block_map_.size());
    for (const std::pair<const BlockIndex, typename BlockType::Ptr>& kv :
         block_map_) {
      blocks->emplace_back(kv.first);
    }
  }

  void getAllUpdatedBlocks(Update::Status bit, BlockIndexList* blocks) const {
    CHECK_NOTNULL(blocks);
    blocks->clear();
    for (const std::pair<const BlockIndex, typename BlockType::Ptr>& kv :
         block_map_) {
      if (kv.second->updated()[bit]) {
        blocks->emplace_back(kv.first);
      }
    }
  }

  size_t getNumberOfAllocatedBlocks() const { return block_map_.size(); }

  bool hasBlock(const BlockIndex& block_index) const {
    return block_map_.count(block_index) > 0;
  }

  /**
   * Get a pointer to the voxel if its corresponding block is allocated and a
   * nullptr otherwise.
   */
  inline const VoxelType* getVoxelPtrByGlobalIndex(
      const GlobalIndex& global_voxel_index) const {
    const BlockIndex block_index = getBlockIndexFromGlobalVoxelIndex(
        global_voxel_index, voxels_per_side_inv_);
    if (!hasBlock(block_index)) {
      return nullptr;
    }
    const VoxelIndex local_voxel_index =
        getLocalFromGlobalVoxelIndex(global_voxel_index, voxels_per_side_);
    const Block<VoxelType>& block = getBlockByIndex(block_index);
    return &block.getVoxelByVoxelIndex(local_voxel_index);
  }

  inline VoxelType* getVoxelPtrByGlobalIndex(
      const GlobalIndex& global_voxel_index) {
    const BlockIndex block_index = getBlockIndexFromGlobalVoxelIndex(
        global_voxel_index, voxels_per_side_inv_);
    if (!hasBlock(block_index)) {
      return nullptr;
    }
    const VoxelIndex local_voxel_index =
        getLocalFromGlobalVoxelIndex(global_voxel_index, voxels_per_side_);
    Block<VoxelType>& block = getBlockByIndex(block_index);
    return &block.getVoxelByVoxelIndex(local_voxel_index);
  }

  inline const VoxelType* getVoxelPtrByCoordinates(const Point& coords) const {
    typename Block<VoxelType>::ConstPtr block_ptr =
        getBlockPtrByIndex(computeBlockIndexFromCoordinates(coords));
    if (!block_ptr) {
      return nullptr;
    }
    return block_ptr->getVoxelPtrByCoordinates(coords);
  }

  inline VoxelType* getVoxelPtrByCoordinates(const Point& coords) {
    typename Block<VoxelType>::Ptr block_ptr =
        getBlockPtrByIndex(computeBlockIndexFromCoordinates(coords));
    if (!block_ptr) {
      return nullptr;
    }
    return block_ptr->getVoxelPtrByCoordinates(coords);
  }

  FloatingPoint block_size() const { return block_size_; }
  FloatingPoint block_size_inv() const { return block_size_inv_; }
  FloatingPoint voxel_size() const { return voxel_size_; }
  FloatingPoint voxel_size_inv() const { return voxel_size_inv_; }
  size_t voxels_per_side() const { return voxels_per_side_; }
  FloatingPoint voxels_per_side_inv() const { return voxels_per_side_inv_; }

  // Serialization tools.
  void getProto(LayerProto* proto) const;
  bool isCompatible(const LayerProto& layer_proto) const;
  bool isCompatible(const BlockProto& layer_proto) const;
  bool saveToFile(const std::string& file_path, bool clear_file = true) const;
  // Default behavior is to clear the file.
  bool saveSubsetToFile(const std::string& file_path,
                        BlockIndexList blocks_to_include,
                        bool include_all_blocks, bool clear_file = true) const;
  bool saveBlocksToStream(bool include_all_blocks,
                          BlockIndexList blocks_to_include,
                          std::fstream* outfile_ptr) const;
  bool addBlockFromProto(const BlockProto& block_proto,
                         BlockMergingStrategy strategy);

  size_t getMemorySize() const;

 protected:
  FloatingPoint voxel_size_;
  size_t voxels_per_side_;
  FloatingPoint block_size_;

  // Derived types.
  FloatingPoint voxel_size_inv_;
  FloatingPoint block_size_inv_;
  FloatingPoint voxels_per_side_inv_;

  std::string getType() const;

  BlockHashMap block_map_;
};

}  // namespace voxblox

#include "voxblox/core/layer_inl.h"

#endif  // VOXBLOX_CORE_LAYER_H_

#ifndef VOXBLOX_CORE_LAYER_H_
#define VOXBLOX_CORE_LAYER_H_

#include <glog/logging.h>
#include <string>
#include <utility>

#include "./Block.pb.h"
#include "./Layer.pb.h"
#include "./MapHeader.pb.h"
#include "./Volume.pb.h"
#include "voxblox/core/common.h"
#include "voxblox/core/block.h"
#include "voxblox/core/block_hash.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

/* Can't specialize an alias template; have to implement workaround
 * template<typename VoxelType>
 * using GenericLayerProto = LayerProto;
 *
 * template<>
 * using GenericLayerProto<TangoTsdfVoxel> = tsdf2::MapHeaderProto;
 */
template <typename VoxelType>
struct LayerProtoType
  { typedef LayerProto type; };

template <>
struct LayerProtoType<TangoTsdfVoxel>
  { typedef tsdf2::MapHeaderProto type; };

template <typename VoxelType>
using GenericLayerProto = typename LayerProtoType<VoxelType>::type;

/* Can't specialize an alias template; have to implement workaround
 * template<typename VoxelType>
 * using GenericBlockProto = BlockProto;
 *
 * template<>
 * using GenericBlockProto<TangoTsdfVoxel> = tsdf2::VolumeProto;
 */
template <typename VoxelType>
struct BlockProtoType
 { typedef BlockProto type; };

template <>
struct BlockProtoType<TangoTsdfVoxel>
 { typedef tsdf2::VolumeProto type; };

template <typename VoxelType>
using GenericBlockProto = typename BlockProtoType<VoxelType>::type;

template <typename VoxelType>
class Layer {
 public:
  typedef std::shared_ptr<Layer> Ptr;
  typedef Block<VoxelType> BlockType;
  typedef typename BlockHashMapType<typename BlockType::Ptr>::type BlockHashMap;
  typedef typename std::pair<BlockIndex, typename BlockType::Ptr> BlockMapPair;

  explicit Layer(FloatingPoint voxel_size, size_t voxels_per_side)
      : voxel_size_(voxel_size), voxels_per_side_(voxels_per_side) {
    block_size_ = voxel_size_ * voxels_per_side_;
    CHECK_GT(block_size_, 0.0f);
    block_size_inv_ = 1.0 / block_size_;
    CHECK_GT(voxels_per_side_, 0u);
    voxels_per_side_inv_ = 1.0f / static_cast<FloatingPoint>(voxels_per_side_);
  }

  // Create the layer from protobuf layer header.
  explicit Layer(const LayerProto& proto);
  explicit Layer(const tsdf2::MapHeaderProto& proto);

  virtual ~Layer() {}

  enum class BlockMergingStrategy { kProhibit, kReplace, kDiscard, kMerge };

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

  // IMPORTANT NOTE: Due the limited accuracy of the FloatingPoint type, this
  // function doesn't always compute the correct block index for coordinates
  // near the block boundaries.
  inline BlockIndex computeBlockIndexFromCoordinates(
      const Point& coords) const {
    return getGridIndexFromPoint(coords, block_size_inv_);
  }

  typename BlockType::Ptr allocateNewBlock(const BlockIndex& index) {
    auto insert_status = block_map_.insert(std::make_pair(
        index, std::shared_ptr<BlockType>(new BlockType(
                   voxels_per_side_, voxel_size_,
                   getOriginPointFromGridIndex(index, block_size_)))));

    DCHECK(insert_status.second) << "Block already exists when allocating at "
                                 << index.transpose();

    DCHECK(insert_status.first->second);
    DCHECK_EQ(insert_status.first->first, index);
    return insert_status.first->second;
  }

  inline typename BlockType::Ptr allocateNewBlockByCoordinates(
      const Point& coords) {
    return allocateNewBlock(computeBlockIndexFromCoordinates(coords));
  }

  void removeBlock(const BlockIndex& index) { block_map_.erase(index); }
  void removeAllBlocks() { block_map_.clear(); }

  void removeBlockByCoordinates(const Point& coords) {
    block_map_.erase(computeBlockIndexFromCoordinates(coords));
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

  void getAllUpdatedBlocks(BlockIndexList* blocks) const {
    CHECK_NOTNULL(blocks);
    blocks->clear();
    for (const std::pair<const BlockIndex, typename BlockType::Ptr>& kv :
         block_map_) {
      if (kv.second->updated()) {
        blocks->emplace_back(kv.first);
      }
    }
  }

  size_t getNumberOfAllocatedBlocks() const { return block_map_.size(); }

  bool hasBlock(const BlockIndex& block_index) const {
    return block_map_.count(block_index) > 0;
  }

  // Get a pointer to the voxel if its corresponding block is allocated and a
  // nullptr otherwise.
  inline const VoxelType* getVoxelPtrByGlobalIndex(
      const VoxelIndex& global_voxel_index) const {
    const BlockIndex block_index = getBlockIndexFromGlobalVoxelIndex(
        global_voxel_index, voxels_per_side_inv_);
    if (!hasBlock(block_index)) {
      return nullptr;
    }
    const VoxelIndex local_voxel_index = getLocalFromGlobalVoxelIndex(
        global_voxel_index, voxels_per_side_);
    const Block<VoxelType>& block = getBlockByIndex(block_index);
    return &block.getVoxelByVoxelIndex(local_voxel_index);
  }

  inline VoxelType* getVoxelPtrByGlobalIndex(
      const VoxelIndex& global_voxel_index) {
    const BlockIndex block_index = getBlockIndexFromGlobalVoxelIndex(
        global_voxel_index, voxels_per_side_inv_);
    if (!hasBlock(block_index)) {
      return nullptr;
    }
    const VoxelIndex local_voxel_index = getLocalFromGlobalVoxelIndex(
        global_voxel_index, voxels_per_side_);
    Block<VoxelType>& block = getBlockByIndex(block_index);
    return &block.getVoxelByVoxelIndex(local_voxel_index);
  }

  FloatingPoint block_size() const { return block_size_; }
  FloatingPoint voxel_size() const { return voxel_size_; }
  size_t voxels_per_side() const { return voxels_per_side_; }

  // Serialization tools.
  void getProto(LayerProto* proto) const;
  bool isCompatible(const LayerProto& layer_proto) const;
  bool isCompatible(const tsdf2::MapHeaderProto& layer_proto) const;
  /* Tango TSDF Block isCompatible has same implementation =>
   * use GenericBlockProto
   */
  bool isCompatible(const GenericBlockProto<VoxelType>& layer_proto) const;
  bool saveToFile(const std::string& file_path) const;
  bool saveSubsetToFile(const std::string& file_path,
                        BlockIndexList blocks_to_include,
                        bool include_all_blocks) const;
  bool addBlockFromProto(const GenericBlockProto<VoxelType>& block_proto,
                         BlockMergingStrategy strategy);

  size_t getMemorySize() const;

 private:
  std::string getType() const;

  FloatingPoint voxel_size_;
  size_t voxels_per_side_;
  FloatingPoint block_size_;

  // Derived types.
  FloatingPoint block_size_inv_;
  FloatingPoint voxels_per_side_inv_;

  BlockHashMap block_map_;
};

}  // namespace voxblox

#include "voxblox/core/layer_inl.h"

#endif  // VOXBLOX_CORE_LAYER_H_

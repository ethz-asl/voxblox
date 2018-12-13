#ifndef VOXBLOX_CORE_BLOCK_H_
#define VOXBLOX_CORE_BLOCK_H_

#include <algorithm>
#include <atomic>
#include <memory>
#include <vector>

#include "./Block.pb.h"
#include "voxblox/core/common.h"

namespace voxblox {

/** An nxnxn container holding VoxelType. It is aware of its 3D position and
 * contains functions for accessing voxels by position and index */
template <typename VoxelType>
class Block {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<Block<VoxelType> > Ptr;
  typedef std::shared_ptr<const Block<VoxelType> > ConstPtr;

  Block(size_t voxels_per_side, FloatingPoint voxel_size, const Point& origin)
      : has_data_(false),
        voxels_per_side_(voxels_per_side),
        voxel_size_(voxel_size),
        origin_(origin),
        updated_(false) {
    num_voxels_ = voxels_per_side_ * voxels_per_side_ * voxels_per_side_;
    voxel_size_inv_ = 1.0 / voxel_size_;
    block_size_ = voxels_per_side_ * voxel_size_;
    block_size_inv_ = 1.0 / block_size_;
    voxels_.reset(new VoxelType[num_voxels_]);
  }

  explicit Block(const BlockProto& proto);

  ~Block() {}

  // Index calculations.
  inline size_t computeLinearIndexFromVoxelIndex(
      const VoxelIndex& index) const {
    size_t linear_index = static_cast<size_t>(
        index.x() +
        voxels_per_side_ * (index.y() + index.z() * voxels_per_side_));

    DCHECK(index.x() >= 0 && index.x() < static_cast<int>(voxels_per_side_));
    DCHECK(index.y() >= 0 && index.y() < static_cast<int>(voxels_per_side_));
    DCHECK(index.z() >= 0 && index.z() < static_cast<int>(voxels_per_side_));

    DCHECK_LT(linear_index,
              voxels_per_side_ * voxels_per_side_ * voxels_per_side_);
    DCHECK_GE(linear_index, 0u);
    return linear_index;
  }

  /** NOTE: This function is dangerous, it will truncate the voxel index to an
   * index that is within this block if you pass a coordinate outside the range
   * of this block. Try not to use this function if there is an alternative to
   * directly address the voxels via precise integer indexing math.
   */
  inline VoxelIndex computeTruncatedVoxelIndexFromCoordinates(
      const Point& coords) const {
    const IndexElement max_value = voxels_per_side_ - 1;
    VoxelIndex voxel_index =
        getGridIndexFromPoint<VoxelIndex>(coords - origin_, voxel_size_inv_);
    // check is needed as getGridIndexFromPoint gives results that have a tiny
    // chance of being outside the valid voxel range.
    return VoxelIndex(std::max(std::min(voxel_index.x(), max_value), 0),
                      std::max(std::min(voxel_index.y(), max_value), 0),
                      std::max(std::min(voxel_index.z(), max_value), 0));
  }

  /**
   * NOTE: This function is also dangerous, use in combination with
   * Block::isValidVoxelIndex function.
   * This function doesn't truncate the voxel index to the [0, voxels_per_side]
   * range when the coordinate is outside the range of this block, unlike the
   * function above.
   */
  inline VoxelIndex computeVoxelIndexFromCoordinates(
      const Point& coords) const {
    VoxelIndex voxel_index =
        getGridIndexFromPoint<VoxelIndex>(coords - origin_, voxel_size_inv_);
    return voxel_index;
  }

  /**
   * NOTE: This function is dangerous, it will truncate the voxel index to an
   * index that is within this block if you pass a coordinate outside the range
   * of this block. Try not to use this function if there is an alternative to
   * directly address the voxels via precise integer indexing math.
   */
  inline size_t computeLinearIndexFromCoordinates(const Point& coords) const {
    return computeLinearIndexFromVoxelIndex(
        computeTruncatedVoxelIndexFromCoordinates(coords));
  }

  /// Returns the CENTER point of the voxel.
  inline Point computeCoordinatesFromLinearIndex(size_t linear_index) const {
    return computeCoordinatesFromVoxelIndex(
        computeVoxelIndexFromLinearIndex(linear_index));
  }

  /// Returns the CENTER point of the voxel.
  inline Point computeCoordinatesFromVoxelIndex(const VoxelIndex& index) const {
    return origin_ + getCenterPointFromGridIndex(index, voxel_size_);
  }

  inline VoxelIndex computeVoxelIndexFromLinearIndex(
      size_t linear_index) const {
    int rem = linear_index;
    VoxelIndex result;
    std::div_t div_temp = std::div(rem, voxels_per_side_ * voxels_per_side_);
    rem = div_temp.rem;
    result.z() = div_temp.quot;
    div_temp = std::div(rem, voxels_per_side_);
    result.y() = div_temp.quot;
    result.x() = div_temp.rem;
    return result;
  }

  /// Accessors to actual blocks.
  inline const VoxelType& getVoxelByLinearIndex(size_t index) const {
    return voxels_[index];
  }

  inline const VoxelType& getVoxelByVoxelIndex(const VoxelIndex& index) const {
    return voxels_[computeLinearIndexFromVoxelIndex(index)];
  }

  /**
   * NOTE: This functions is dangerous, it will truncate the voxel index to an
   *index that is within this block if you pass a coordinate outside the range
   *of this block. Try not to use this function if there is an alternative to
   * directly address the voxels via precise integer indexing math.
   */
  inline const VoxelType& getVoxelByCoordinates(const Point& coords) const {
    return voxels_[computeLinearIndexFromCoordinates(coords)];
  }

  /**
   * NOTE: This functions is dangerous, it will truncate the voxel index to an
   *index that is within this block if you pass a coordinate outside the range
   *of this block. Try not to use this function if there is an alternative to
   * directly address the voxels via precise integer indexing math.
   */
  inline VoxelType& getVoxelByCoordinates(const Point& coords) {
    return voxels_[computeLinearIndexFromCoordinates(coords)];
  }

  /**
   * NOTE: This functions is dangerous, it will truncate the voxel index to an
   *index that is within this block if you pass a coordinate outside the range
   *of this block. Try not to use this function if there is an alternative to
   * directly address the voxels via precise integer indexing math.
   */
  inline VoxelType* getVoxelPtrByCoordinates(const Point& coords) {
    return &voxels_[computeLinearIndexFromCoordinates(coords)];
  }

  inline const VoxelType* getVoxelPtrByCoordinates(const Point& coords) const {
    return &voxels_[computeLinearIndexFromCoordinates(coords)];
  }

  inline VoxelType& getVoxelByLinearIndex(size_t index) {
    DCHECK_LT(index, num_voxels_);
    return voxels_[index];
  }

  inline VoxelType& getVoxelByVoxelIndex(const VoxelIndex& index) {
    return voxels_[computeLinearIndexFromVoxelIndex(index)];
  }

  inline bool isValidVoxelIndex(const VoxelIndex& index) const {
    if (index.x() < 0 ||
        index.x() >= static_cast<IndexElement>(voxels_per_side_)) {
      return false;
    }
    if (index.y() < 0 ||
        index.y() >= static_cast<IndexElement>(voxels_per_side_)) {
      return false;
    }
    if (index.z() < 0 ||
        index.z() >= static_cast<IndexElement>(voxels_per_side_)) {
      return false;
    }
    return true;
  }

  inline bool isValidLinearIndex(size_t index) const {
    if (index < 0 || index >= num_voxels_) {
      return false;
    }
    return true;
  }

  BlockIndex block_index() const {
    return getGridIndexFromOriginPoint<BlockIndex>(origin_, block_size_inv_);
  }

  // Basic function accessors.
  size_t voxels_per_side() const { return voxels_per_side_; }
  FloatingPoint voxel_size() const { return voxel_size_; }
  FloatingPoint voxel_size_inv() const { return voxel_size_inv_; }
  size_t num_voxels() const { return num_voxels_; }
  Point origin() const { return origin_; }
  void setOrigin(const Point& new_origin) { origin_ = new_origin; }
  FloatingPoint block_size() const { return block_size_; }

  bool has_data() const { return has_data_; }
  bool updated() const { return updated_; }

  std::atomic<bool>& updated() { return updated_; }
  bool& has_data() { return has_data_; }

  void set_updated(bool updated) { updated_ = updated; }
  void set_has_data(bool has_data) { has_data_ = has_data; }

  // Serialization.
  void getProto(BlockProto* proto) const;
  void serializeToIntegers(std::vector<uint32_t>* data) const;
  void deserializeFromIntegers(const std::vector<uint32_t>& data);

  void mergeBlock(const Block<VoxelType>& other_block);

  size_t getMemorySize() const;

 protected:
  std::unique_ptr<VoxelType[]> voxels_;

  // Derived, cached parameters.
  size_t num_voxels_;

  /// Is set to true if any one of the voxels in this block received an update.
  bool has_data_;

 private:
  void deserializeProto(const BlockProto& proto);
  void serializeProto(BlockProto* proto) const;

  // Base parameters.
  const size_t voxels_per_side_;
  const FloatingPoint voxel_size_;
  Point origin_;

  // Derived, cached parameters.
  FloatingPoint voxel_size_inv_;
  FloatingPoint block_size_;
  FloatingPoint block_size_inv_;

  /// Is set to true when data is updated.
  std::atomic<bool> updated_;
};

}  // namespace voxblox

#include "voxblox/core/block_inl.h"

#endif  // VOXBLOX_CORE_BLOCK_H_

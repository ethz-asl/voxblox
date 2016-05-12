#ifndef VOXBLOX_CORE_BLOCK_H_
#define VOXBLOX_CORE_BLOCK_H_

#include <memory>

#include "voxblox/Block.pb.h"
#include "voxblox/core/common.h"

namespace voxblox {

template <typename VoxelType>
class Block {
 public:
  typedef std::shared_ptr<Block<VoxelType> > Ptr;
  typedef std::shared_ptr<const Block<VoxelType> > ConstPtr;

  Block(size_t voxels_per_side, FloatingPoint voxel_size, const Point& origin)
      : voxels_per_side_(voxels_per_side),
        voxel_size_(voxel_size),
        origin_(origin) {
    num_voxels_ = voxels_per_side_ * voxels_per_side_ * voxels_per_side_;
    voxel_size_inv_ = 1.0 / voxel_size_;
    block_size_ = voxels_per_side_ * voxel_size_;
    voxels_.reset(new VoxelType[num_voxels_]);
  }

  explicit Block(const BlockProto& proto);

  ~Block() {}

  // Index calculations.
  inline size_t computeLinearIndexFromVoxelIndex(
      const VoxelIndex& index) const {
    size_t linear_index =
        index.x() +
        voxels_per_side_ * (index.y() + index.z() * voxels_per_side_);

    DCHECK(index.x() >= 0 && index.x() < voxels_per_side_);
    DCHECK(index.y() >= 0 && index.y() < voxels_per_side_);
    DCHECK(index.z() >= 0 && index.z() < voxels_per_side_);

    DCHECK_LT(linear_index,
              voxels_per_side_ * voxels_per_side_ * voxels_per_side_);
    DCHECK_GE(linear_index, 0);
    return linear_index;
  }

  inline VoxelIndex computeVoxelIndexFromCoordinates(
      const Point& coords) const {
    return floorVectorAndDowncast((coords - origin_) * voxel_size_inv_);
  }

  inline size_t computeLinearIndexFromCoordinates(const Point& coords) const {
    return computeLinearIndexFromVoxelIndex(
        computeVoxelIndexFromCoordinates(coords));
  }

  inline Point computeCoordinatesFromLinearIndex(size_t linear_index) const {
    return computeCoordinatesFromVoxelIndex(
        computeVoxelIndexFromLinearIndex(linear_index));
  }

  inline Point computeCoordinatesFromVoxelIndex(const VoxelIndex& index) const {
    return (index.cast<FloatingPoint>() + 0.5 * Point::Ones()) * voxel_size_ +
           origin_;
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

  // Accessors to actual blocks.
  inline const VoxelType& getVoxelByLinearIndex(size_t index) const {
    return voxels_[index];
  }

  inline const VoxelType& getVoxelByVoxelIndex(const VoxelIndex& index) const {
    return voxels_[computeLinearIndexFromVoxelIndex(index)];
  }

  inline const VoxelType& getVoxelByCoordinates(const Point& coords) const {
    return voxels_[computeLinearIndexFromCoordinates(coords)];
  }

  inline VoxelType& getVoxelByLinearIndex(size_t index) {
    DCHECK_LT(index, num_voxels_);
    return voxels_[index];
  }

  inline VoxelType& getVoxelByVoxelIndex(const VoxelIndex& index) {
    return voxels_[computeLinearIndexFromVoxelIndex(index)];
  }

  inline VoxelType& getVoxelByCoordinates(const Point& coords) {
    return voxels_[computeLinearIndexFromCoordinates(coords)];
  }

  // Basic function accessors.
  size_t voxels_per_side() const { return voxels_per_side_; }
  FloatingPoint voxel_size() const { return voxel_size_; }
  Point origin() const { return origin_; }

  bool has_data() const { return has_data_; }
  bool updated() const { return updated_; }

  bool& updated() { return updated_; }
  bool& has_data() { return has_data_; }

 private:
  void DeserializeVoxelData(const BlockProto& proto, VoxelType* voxels);

  // Base parameters.
  const size_t voxels_per_side_;
  const FloatingPoint voxel_size_;
  const Point origin_;

  // Derived, cached parameters.
  size_t num_voxels_;
  FloatingPoint voxel_size_inv_;
  FloatingPoint block_size_;

  // Is set to true if any one of the voxels in this block received an update.
  bool has_data_;
  // Is set to true when data is updated.
  bool updated_;

  std::unique_ptr<VoxelType[]> voxels_;
};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_BLOCK_H_

#ifndef VOXBLOX_CORE_BLOCK_H_
#define VOXBLOX_CORE_BLOCK_H_

#include <memory>

#include "voxblox/core/common.h"
#include "voxblox/core/voxel.h"
#include "voxblox/core/voxel_array.h"

namespace voxblox {

class BaseBlock {
 public:
  typedef std::shared_ptr<BaseBlock> Ptr;
  typedef std::shared_ptr<const BaseBlock> ConstPtr;

  BaseBlock(size_t num_layers, const Point& origin, FloatingPoint block_size)
      : num_layers_(num_layers),
        origin_(origin),
        block_size_(block_size),
        has_data_(false) {}

  virtual ~BaseBlock() {}

  inline size_t computeLinearIndexFromVoxelIndex(const VoxelIndex& index,
                                                 size_t vps) const {
    size_t linear_index = index.x() + vps * (index.y() + index.z() * vps);

    DCHECK(index.x() >= 0 && index.x() < vps);
    DCHECK(index.y() >= 0 && index.y() < vps);
    DCHECK(index.z() >= 0 && index.z() < vps);

    DCHECK_LT(linear_index, vps * vps * vps);
    DCHECK_GE(linear_index, 0);
    return linear_index;
  }

  inline VoxelIndex computeVoxelIndexFromCoordinates(
      const Point& coords, FloatingPoint voxel_size_inv) const {
    return floorVectorAndDowncast((coords - origin_) * voxel_size_inv);
  }

  inline size_t computeLinearIndexFromCoordinates(const Point& coords,
                                                  FloatingPoint voxel_size_inv,
                                                  size_t vps) const {
    return computeLinearIndexFromVoxelIndex(
        computeVoxelIndexFromCoordinates(coords, voxel_size_inv), vps);
  }

  inline Point computeCoordinatesFromLinearIndex(size_t linear_index,
                                                 FloatingPoint voxel_size,
                                                 size_t vps) const {
    return computeCoordinatesFromVoxelIndex(
        computeVoxelIndexFromLinearIndex(linear_index, vps), voxel_size);
  }

  inline Point computeCoordinatesFromVoxelIndex(
      const VoxelIndex& index, FloatingPoint voxel_size) const {
    return (index.cast<FloatingPoint>() + 0.5 * Point::Ones()) * voxel_size +
           origin_;
  }

  // TODO(mfehr, helenol): Fix this function
  inline VoxelIndex computeVoxelIndexFromLinearIndex(size_t linear_index,
                                                     size_t vps) const {
    CHECK(false);

    int rem = linear_index;
    VoxelIndex result;
    std::div_t div_temp = std::div(rem, vps * vps);
    rem = div_temp.rem;
    result.z() = div_temp.quot;
    div_temp = std::div(rem, vps);
    result.y() = div_temp.quot;
    result.x() = div_temp.rem;
    return result;
  }

  const Point& getOrigin() const { return origin_; }

 protected:
  size_t num_layers_;
  Point origin_;
  FloatingPoint block_size_;

  // Is set to true if any one of the voxels in this block received an update.
  bool has_data_;
};
}  // namespace voxblox

#endif  // VOXBLOX_CORE_BLOCK_H_

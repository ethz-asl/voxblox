#ifndef VOXBLOX_CORE_VOXEL_ARRAY_H_
#define VOXBLOX_CORE_VOXEL_ARRAY_H_

#include <memory>

#include "voxblox/core/common.h"

namespace voxblox {

template <typename VoxelType>
struct VoxelArray {
 public:
  VoxelArray(size_t _voxels_per_side, FloatingPoint _voxel_size,
             const Point& _origin)
      : voxels_per_side(_voxels_per_side),
        voxel_size(_voxel_size),
        origin(_origin) {
    num_voxels = voxels_per_side * voxels_per_side * voxels_per_side;
    voxel_size_inv = 1.0 / voxel_size;
    voxels.reset(new VoxelType[num_voxels]);
  }

  ~VoxelArray() {}

  // Index calculations.
  inline size_t computeLinearIndexFromVoxelIndex(
      const VoxelIndex& index) const {
    size_t linear_index =
        index.x() + voxels_per_side * (index.y() + index.z() * voxels_per_side);

    DCHECK(index.x() >= 0 && index.x() < voxels_per_side);
    DCHECK(index.y() >= 0 && index.y() < voxels_per_side);
    DCHECK(index.z() >= 0 && index.z() < voxels_per_side);

    DCHECK_LT(linear_index,
              voxels_per_side * voxels_per_side * voxels_per_side);
    DCHECK_GE(linear_index, 0);
    return linear_index;
  }

  inline VoxelIndex computeVoxelIndexFromCoordinates(
      const Point& coords) const {
    return floorVectorAndDowncast((coords - origin) * voxel_size_inv);
  }

  inline size_t computeLinearIndexFromCoordinates(const Point& coords) const {
    return computeLinearIndexFromVoxelIndex(
        computeVoxelIndexFromCoordinates(coords));
  }

  inline Point computeCoordinatesFromLinearIndex(size_t linear_index) const {
    return computeCoordinatesFromVoxelIndex(
        computeVoxelIndexFromLinearIndex(linear_index));
  }

  inline Point computeCoordinatesFromVoxelIndex(
      const VoxelIndex& index) const {
    return (index.cast<FloatingPoint>() + 0.5 * Point::Ones()) * voxel_size +
           origin;
  }

  // TODO(mfehr, helenol): Fix this function
  inline VoxelIndex computeVoxelIndexFromLinearIndex(
      size_t linear_index) const {
    int rem = linear_index;
    VoxelIndex result;
    std::div_t div_temp = std::div(rem, voxels_per_side * voxels_per_side);
    rem = div_temp.rem;
    result.z() = div_temp.quot;
    div_temp = std::div(rem, voxels_per_side);
    result.y() = div_temp.quot;
    result.x() = div_temp.rem;
    return result;
  }

  // Accessors to actual blocks.
  inline const VoxelType& getVoxelByLinearIndex(size_t index) const {
    return voxels[index];
  }

  inline const VoxelType& getVoxelByVoxelIndex(const VoxelIndex& index) const {
    return voxels[computeLinearIndexFromVoxelIndex(index)];
  }

  inline const VoxelType& getVoxelByCoordinates(const Point& coords) const {
    return voxels[computeLinearIndexFromCoordinates(coords)];
  }

  inline VoxelType& getVoxelByLinearIndex(size_t index) {
    DCHECK_LT(index, num_voxels);
    return voxels[index];
  }

  inline VoxelType& getVoxelByVoxelIndex(const VoxelIndex& index) {
    return voxels[computeLinearIndexFromVoxelIndex(index)];
  }

  inline VoxelType& getVoxelByCoordinates(const Point& coords) {
    return voxels[computeLinearIndexFromCoordinates(coords)];
  }

  // Base parameters.
  const size_t voxels_per_side;
  const FloatingPoint voxel_size;
  const Point origin;

  // Derived, cached parameters.
  size_t num_voxels;
  FloatingPoint voxel_size_inv;

  std::unique_ptr<VoxelType[]> voxels;
};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_VOXEL_ARRAY_H_

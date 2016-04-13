#ifndef VOXBLOX_BLOCK_H
#define VOXBLOX_BLOCK_H

#include "voxblox/common.h"
#include "voxblox/voxel.h"
#include "voxblox/voxel_array.h"

namespace voxblox {

class BaseBlock {
 public:
  BaseBlock(size_t num_layers, const Coordinates& origin,
            FloatingPoint block_size)
      : num_layers_(num_layers),
        origin_(origin),
        block_size_(block_size),
        has_data_(false) {}
  virtual ~BaseBlock() {}

  size_t computeLinearIndexFrom3dIndex(const VoxelIndex& index,
                                       size_t vps) const {
    return index.x() + vps * (index.y() + index.z() * vps);
  }

  VoxelIndex compute3dIndexFromCoordinates(const Coordinates& coords,
                                           FloatingPoint voxel_size_inv) const {
    return VoxelIndex(static_cast<int>(std::floor((coords.x() - origin_.x()) *
                                                  voxel_size_inv)),
                      static_cast<int>(std::floor((coords.y() - origin_.y()) *
                                                  voxel_size_inv)),
                      static_cast<int>(std::floor((coords.z() - origin_.z()) *
                                                  voxel_size_inv)));
  }

  size_t computeLinearIndexFromCoordinates(const Coordinates& coords,
                                           FloatingPoint voxel_size_inv,
                                           size_t vps) const {
    return computeLinearIndexFrom3dIndex(
        compute3dIndexFromCoordinates(coords, voxel_size_inv), vps);
  }

  // TODO: reverse operations.

 protected:
  size_t num_layers_;
  Coordinates origin_;
  FloatingPoint block_size_;
  bool has_data_;
};

class TsdfBlock : BaseBlock {
 public:
  TsdfBlock(const Coordinates& origin, size_t voxels_per_side,
            FloatingPoint voxel_size)
      : BaseBlock(1, origin, voxels_per_side * voxel_size),
        has_changed_(false) {
    tsdf_layer_.reset(new VoxelArray<TsdfVoxel>(voxels_per_side, voxel_size));
  }
  virtual ~TsdfBlock() {}

  inline const TsdfVoxel& getTsdfVoxelByLinearIndex(size_t index) const {
    return tsdf_layer_->voxels[index];
  }
  inline const TsdfVoxel& getTsdfVoxelBy3dIndex(const VoxelIndex& index) const {
    return tsdf_layer_->voxels[computeLinearIndexFrom3dIndex(
        index, tsdf_layer_->voxel_size_inv)];
  }
  inline const TsdfVoxel& getTsdfVoxelByCoordinates(
      const Coordinates& coords) const {
    return tsdf_layer_->voxels[computeLinearIndexFromCoordinates(
        coords, tsdf_layer_->voxel_size_inv, tsdf_layer_->voxels_per_side)];
  }

  inline TsdfVoxel& getTsdfVoxelByLinearIndex(size_t index) {
    return tsdf_layer_->voxels[index];
  }
  inline TsdfVoxel& getTsdfVoxelBy3dIndex(const VoxelIndex& index) {
    return tsdf_layer_->voxels[computeLinearIndexFrom3dIndex(
        index, tsdf_layer_->voxel_size_inv)];
  }
  inline TsdfVoxel& getTsdfVoxelByCoordinates(const Coordinates& coords) {
    return tsdf_layer_->voxels[computeLinearIndexFromCoordinates(
        coords, tsdf_layer_->voxel_size_inv, tsdf_layer_->voxels_per_side)];
  }

 protected:
  bool has_changed_;

  std::unique_ptr<VoxelArray<TsdfVoxel> > tsdf_layer_;
};

}  // namespace voxblox

#endif  // VOXBLOX_BLOCK_H

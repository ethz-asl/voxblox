#ifndef VOXBLOX_CORE_TSDF_BLOCK_H_
#define VOXBLOX_CORE_TSDF_BLOCK_H_

#include <glog/logging.h>
#include <memory>

#include "voxblox/core/block.h"
#include "voxblox/core/common.h"
#include "voxblox/core/voxel.h"
#include "voxblox/core/voxel_array.h"

namespace voxblox {

class TsdfBlock : public BaseBlock {
 public:
  typedef std::shared_ptr<TsdfBlock> Ptr;
  typedef std::shared_ptr<const BaseBlock> ConstPtr;

  TsdfBlock(const Point& origin, size_t voxels_per_side,
            FloatingPoint voxel_size)
      : BaseBlock(1, origin, voxels_per_side * voxel_size),
        has_changed_(false) {
    tsdf_layer_.reset(new VoxelArray<TsdfVoxel>(voxels_per_side, voxel_size));
  }

  virtual ~TsdfBlock() {}

  inline const TsdfVoxel& getTsdfVoxelByLinearIndex(size_t index) const {
    return tsdf_layer_->voxels[index];
  }

  inline const TsdfVoxel& getTsdfVoxelByVoxelIndex(const VoxelIndex& index)
      const {
    return tsdf_layer_->voxels
        [computeLinearIndexFromVoxelIndex(index, tsdf_layer_->voxels_per_side)];
  }

  inline const TsdfVoxel& getTsdfVoxelByCoordinates(const Point& coords) const {
    return tsdf_layer_->voxels[computeLinearIndexFromCoordinates(
        coords, tsdf_layer_->voxel_size_inv, tsdf_layer_->voxels_per_side)];
  }

  inline TsdfVoxel& getTsdfVoxelByLinearIndex(size_t index) {
    DCHECK_LT(index, tsdf_layer_->num_voxels);
    return tsdf_layer_->voxels[index];
  }
  inline TsdfVoxel& getTsdfVoxelByVoxelIndex(const VoxelIndex& index) {
    DCHECK(tsdf_layer_);
    size_t linear_index =
        computeLinearIndexFromVoxelIndex(index, tsdf_layer_->voxels_per_side);
    DCHECK_GE(linear_index, 0);
    DCHECK_LT(linear_index, tsdf_layer_->num_voxels);
    return tsdf_layer_->voxels[linear_index];
  }

  inline TsdfVoxel& getTsdfVoxelByCoordinates(const Point& coords) {
    return tsdf_layer_->voxels[computeLinearIndexFromCoordinates(
        coords, tsdf_layer_->voxel_size_inv, tsdf_layer_->voxels_per_side)];
  }

  inline size_t getNumTsdfVoxels() const { return tsdf_layer_->num_voxels; }

  inline Point getCoordinatesOfTsdfVoxelByLinearIndex(size_t index) const {
    return computeCoordinatesFromLinearIndex(index, tsdf_layer_->voxel_size,
                                             tsdf_layer_->voxels_per_side);
  }

  inline Point getCoordinatesOfTsdfVoxelByVoxelIndex(const VoxelIndex& index)
      const {
    return computeCoordinatesFromVoxelIndex(index, tsdf_layer_->voxel_size);
  }

 protected:
  bool has_changed_;

  std::unique_ptr<VoxelArray<TsdfVoxel> > tsdf_layer_;
};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_TSDF_BLOCK_H_

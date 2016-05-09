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
    tsdf_layer_.reset(
        new VoxelArray<TsdfVoxel>(voxels_per_side, voxel_size, origin));
  }

  virtual ~TsdfBlock() {}

  const VoxelArray<TsdfVoxel>& getTsdfLayer() const { return *tsdf_layer_; }
  VoxelArray<TsdfVoxel>& getTsdfLayerMutable() { return *tsdf_layer_; }

  VoxelArray<TsdfVoxel>* getTsdfLayerPtr() { return tsdf_layer_.get(); }

  // Convenience functions for accessing voxels (could also be done at the
  // layer level).
  inline const TsdfVoxel& getTsdfVoxelByVoxelIndex(
      const VoxelIndex& index) const {
    return tsdf_layer_->getVoxelByVoxelIndex(index);
  }

  inline const TsdfVoxel& getTsdfVoxelByCoordinates(const Point& coords) const {
    return tsdf_layer_->getVoxelByCoordinates(coords);
  }

  inline TsdfVoxel& getTsdfVoxelByLinearIndex(size_t index) {
    return tsdf_layer_->getVoxelByLinearIndex(index);
  }

  inline TsdfVoxel& getTsdfVoxelByVoxelIndex(const VoxelIndex& index) {
    return tsdf_layer_->getVoxelByVoxelIndex(index);
  }

  inline TsdfVoxel& getTsdfVoxelByCoordinates(const Point& coords) {
    return tsdf_layer_->getVoxelByCoordinates(coords);
  }

  inline size_t getNumTsdfVoxels() const { return tsdf_layer_->num_voxels; }

  inline Point getCoordinatesOfTsdfVoxelByLinearIndex(size_t index) const {
    return tsdf_layer_->computeCoordinatesFromLinearIndex(index);
  }

  inline Point getCoordinatesOfTsdfVoxelByVoxelIndex(
      const VoxelIndex& index) const {
    return tsdf_layer_->computeCoordinatesFromVoxelIndex(index);
  }

 protected:
  bool has_changed_;

  std::unique_ptr<VoxelArray<TsdfVoxel> > tsdf_layer_;
};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_TSDF_BLOCK_H_

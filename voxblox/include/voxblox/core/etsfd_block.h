#ifndef VOXBLOX_CORE_TSDF_BLOCK_H_
#define VOXBLOX_CORE_TSDF_BLOCK_H_

#include <glog/logging.h>
#include <memory>

#include "voxblox/core/block.h"
#include "voxblox/core/common.h"
#include "voxblox/core/voxel.h"
#include "voxblox/core/voxel_array.h"

namespace voxblox {

class EtsdfBlock : public BaseBlock {
 public:
  typedef std::shared_ptr<EtsdfBlock> Ptr;
  typedef std::shared_ptr<const BaseBlock> ConstPtr;

  EtsdfBlock(const Point& origin, size_t voxels_per_side,
            FloatingPoint voxel_size)
      : BaseBlock(1, origin, voxels_per_side * voxel_size),
        has_changed_(false) {
    tsdf_layer_.reset(
        new VoxelArray<TsdfVoxel>(voxels_per_side, voxel_size, origin));
  }

  virtual ~EtsdfBlock() {}

  const VoxelArray<TsdfVoxel>& getTsdfLayer() const { return *tsdf_layer_; }
  VoxelArray<TsdfVoxel>& getTsdfLayerMutable() { return *tsdf_layer_; }
  VoxelArray<TsdfVoxel>* getTsdfLayerPtr() { return tsdf_layer_.get(); }

  const VoxelArray<TsdfVoxel>& getEsdfLayer() const { return *esdf_layer_; }
  VoxelArray<TsdfVoxel>& getEsdfLayerMutable() { return *esdf_layer_; }
  VoxelArray<TsdfVoxel>* getEsdfLayerPtr() { return esdf_layer_.get(); }

 protected:
  bool has_changed_;

  std::unique_ptr<VoxelArray<TsdfVoxel> > tsdf_layer_;
  std::unique_ptr<VoxelArray<EsdfVoxel> > esdf_layer_;
};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_TSDF_BLOCK_H_

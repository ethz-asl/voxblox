#ifndef VOXBLOX_FAST_CORE_TSDF_MAP_H_
#define VOXBLOX_FAST_CORE_TSDF_MAP_H_

#include <glog/logging.h>
#include <memory>
#include <utility>

#include "voxblox_fast/core/common.h"
#include "voxblox_fast/core/layer.h"
#include "voxblox_fast/core/voxel.h"

namespace voxblox_fast {

class TsdfMap {
 public:
  typedef std::shared_ptr<TsdfMap> Ptr;

  struct Config {
    FloatingPoint tsdf_voxel_size = 0.2;
    size_t tsdf_voxels_per_side = 16u;
  };

  explicit TsdfMap(const Config& config)
      : tsdf_layer_(new Layer<TsdfVoxel>(config.tsdf_voxel_size,
                                         config.tsdf_voxels_per_side)) {
    block_size_ = config.tsdf_voxel_size * config.tsdf_voxels_per_side;
  }

  virtual ~TsdfMap() {}

  Layer<TsdfVoxel>* getTsdfLayerPtr() { return tsdf_layer_.get(); }
  const Layer<TsdfVoxel>& getTsdfLayer() const { return *tsdf_layer_; }

  FloatingPoint block_size() const { return block_size_; }

 protected:
  FloatingPoint block_size_;

  // The layers.
  Layer<TsdfVoxel>::Ptr tsdf_layer_;
};

}  // namespace voxblox

#endif  // VOXBLOX_FAST_CORE_TSDF_MAP_H_

#ifndef VOXBLOX_CORE_TSDF_MAP_H_
#define VOXBLOX_CORE_TSDF_MAP_H_

#include <glog/logging.h>
#include <memory>
#include <utility>

#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

class TsdfMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<TsdfMap> Ptr;

  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FloatingPoint tsdf_voxel_size = 0.2;
    size_t tsdf_voxels_per_side = 16u;
  };

  explicit TsdfMap(const Config& config)
      : tsdf_layer_(new Layer<TsdfVoxel>(config.tsdf_voxel_size,
                                         config.tsdf_voxels_per_side)) {
    block_size_ = config.tsdf_voxel_size * config.tsdf_voxels_per_side;
  }

  // Creates a new TsdfMap based on a COPY of this layer.
  explicit TsdfMap(const Layer<TsdfVoxel>& layer)
      : TsdfMap(aligned_shared<Layer<TsdfVoxel>>(layer)) {}

  // Creates a new TsdfMap that contains this layer.
  explicit TsdfMap(Layer<TsdfVoxel>::Ptr layer) : tsdf_layer_(layer) {
    CHECK(layer);
    block_size_ = layer->block_size();
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

#endif  // VOXBLOX_CORE_TSDF_MAP_H_

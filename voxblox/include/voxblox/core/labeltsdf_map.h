#ifndef VOXBLOX_CORE_LABELTSDF_MAP_H_
#define VOXBLOX_CORE_LABELTSDF_MAP_H_

#include <glog/logging.h>
#include <memory>
#include <utility>

#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

class LabelTsdfMap {
 public:
  typedef std::shared_ptr<LabelTsdfMap> Ptr;

  struct Config {
    FloatingPoint voxel_size = 0.2;
    size_t voxels_per_side = 16u;
  };

  explicit LabelTsdfMap(Config config)
      : tsdf_layer_(new Layer<TsdfVoxel>(
          config.voxel_size, config.voxels_per_side)),
        label_layer_(new Layer<LabelVoxel>(
          config.voxel_size, config.voxels_per_side)),
        highest_label_(0u) {}

  virtual ~LabelTsdfMap() {}

  Layer<TsdfVoxel>* getTsdfLayerPtr() {
    return tsdf_layer_.get();
  }

  const Layer<TsdfVoxel>& getTsdfLayer() const {
    return *tsdf_layer_;
  }

  Layer<LabelVoxel>* getLabelLayerPtr() {
    return label_layer_.get();
  }

  const Layer<LabelVoxel>& getLabelLayer() const {
    return *label_layer_;
  }

  Label* getHighestLabelPtr() {
    return &highest_label_;
  }

  FloatingPoint block_size() const { return tsdf_layer_->block_size(); }

 protected:
  // The layers.
  Layer<TsdfVoxel>::Ptr tsdf_layer_;
  Layer<LabelVoxel>::Ptr label_layer_;

  Label highest_label_;
};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_LABELTSDF_MAP_H_

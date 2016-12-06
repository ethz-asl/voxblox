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
    float labeltsdf_voxel_size = 0.2;
    float labeltsdf_voxels_per_side = 16;
  };

  explicit LabelTsdfMap(Config config)
      : labeltsdf_layer_(new Layer<LabelTsdfVoxel>(
          config.labeltsdf_voxel_size, config.labeltsdf_voxels_per_side)),
        highest_label_(0) {
    block_size_ =
        config.labeltsdf_voxel_size * config.labeltsdf_voxels_per_side;
  }

  virtual ~LabelTsdfMap() {}

  Layer<LabelTsdfVoxel>* getLabelTsdfLayerPtr() {
    return labeltsdf_layer_.get();
  }

  const Layer<LabelTsdfVoxel>& getLabelTsdfLayer() const {
    return *labeltsdf_layer_;
  }

  FloatingPoint block_size() const { return block_size_; }

  Label get_new_label() { return ++highest_label_; }

 protected:
  FloatingPoint block_size_;

  // The layers.
  Layer<LabelTsdfVoxel>::Ptr labeltsdf_layer_;

  Label highest_label_;
};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_LABELTSDF_MAP_H_

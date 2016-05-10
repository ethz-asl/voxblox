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
  typedef std::shared_ptr<TsdfMap> Ptr;

  struct Config {
    float tsdf_voxel_size = 0.2;
    float tsdf_voxels_per_side = 16;
  };

  explicit TsdfMap(Config config)
      : tsdf_layer_(config.tsdf_voxel_size, config.tsdf_voxels_per_side) {
    block_size_ = config.tsdf_voxel_size * config.tsdf_voxels_per_side;
  }

  virtual ~TsdfMap() {}

 protected:
  FloatingPoint block_size_;

  // The layers.
  Layer<TsdfVoxel> tsdf_layer_;
};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_TSDF_MAP_H_

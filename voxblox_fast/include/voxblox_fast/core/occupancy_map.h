#ifndef VOXBLOX_FAST_CORE_OCCUPANCY_MAP_H_
#define VOXBLOX_FAST_CORE_OCCUPANCY_MAP_H_

#include <glog/logging.h>
#include <memory>
#include <utility>

#include "voxblox_fast/core/common.h"
#include "voxblox_fast/core/layer.h"
#include "voxblox_fast/core/voxel.h"

namespace voxblox_fast {

class OccupancyMap {
 public:
  typedef std::shared_ptr<OccupancyMap> Ptr;

  struct Config {
    FloatingPoint occupancy_voxel_size = 0.2;
    size_t occupancy_voxels_per_side = 16u;
  };

  explicit OccupancyMap(const Config& config)
      : occupancy_layer_(new Layer<OccupancyVoxel>(
            config.occupancy_voxel_size, config.occupancy_voxels_per_side)) {
    block_size_ =
        config.occupancy_voxel_size * config.occupancy_voxels_per_side;
  }

  virtual ~OccupancyMap() {}

  Layer<OccupancyVoxel>* getOccupancyLayerPtr() {
    return occupancy_layer_.get();
  }
  const Layer<OccupancyVoxel>& getOccupancyLayerPtr() const {
    return *occupancy_layer_;
  }

  FloatingPoint block_size() const { return block_size_; }

 protected:
  FloatingPoint block_size_;

  // The layers.
  Layer<OccupancyVoxel>::Ptr occupancy_layer_;
};

}  // namespace voxblox

#endif  // VOXBLOX_FAST_CORE_OCCUPANCY_MAP_H_

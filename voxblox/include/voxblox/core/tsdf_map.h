#ifndef VOXBLOX_CORE_TSDF_MAP_H_
#define VOXBLOX_CORE_TSDF_MAP_H_

#include <glog/logging.h>
#include <memory>
#include <utility>

#include "voxblox/core/common.h"
#include "voxblox/core/map.h"
#include "voxblox/core/tsdf_block.h"

namespace voxblox {

class TsdfMap : public Map<TsdfBlock> {
 public:
  typedef std::shared_ptr<TsdfMap> Ptr;

  struct Config {
    float tsdf_voxel_size = 0.2;
    float tsdf_voxels_per_side = 16;
  };

  explicit TsdfMap(Config config)
      : Map(config.tsdf_voxel_size * config.tsdf_voxels_per_side),
        voxel_size_(config.tsdf_voxel_size),
        voxels_per_side_(config.tsdf_voxels_per_side) {}

  virtual ~TsdfMap() {}

  virtual TsdfBlock::Ptr allocateNewBlock(const BlockIndex& index) {
    auto insert_status = block_map_.insert(
        std::make_pair(index, std::shared_ptr<TsdfBlock>(new TsdfBlock(
                                  index.cast<FloatingPoint>() * block_size_,
                                  voxels_per_side_, voxel_size_))));

    DCHECK(insert_status.second) << "Block already exists when allocating at "
                                 << index.transpose();

    DCHECK_NOTNULL(insert_status.first->second);
    DCHECK_EQ(insert_status.first->first, index);
    return insert_status.first->second;
  }

  size_t getTsdfVoxelsPerBlock() const {
    return voxels_per_side_ * voxels_per_side_ * voxels_per_side_;
  }

  FloatingPoint getTsdfVoxelSize() const { return voxel_size_; }

  size_t getTsdfVoxelsPerSide() const { return voxels_per_side_; }

 protected:
  FloatingPoint voxel_size_;
  size_t voxels_per_side_;
};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_TSDF_MAP_H_

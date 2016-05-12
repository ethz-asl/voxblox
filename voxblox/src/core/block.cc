
#include "voxblox/core/block.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

template <>
void Block<TsdfVoxel>::DeserializeVoxelData(const BlockProto& proto,
                                            TsdfVoxel* voxels) {
  for (size_t voxel_idx; voxel_idx < num_voxels_; ++voxel_idx) {
  }
}

template <>
void Block<TsdfVoxel>::SerializeVoxelData(const TsdfVoxel* voxels,
                                          BlockProto* proto) const {}

}  // namespace voxblox

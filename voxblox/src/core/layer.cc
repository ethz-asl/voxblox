#include "./Block.pb.h"
#include "voxblox/core/block.h"
#include "voxblox/core/layer.h"

namespace voxblox {

template <class VoxelType>
void Layer<VoxelType>::GetProto(LayerProto* proto) const {
  proto->set_voxel_size(voxel_size_);
  proto->set_voxels_per_side(voxels_per_side_);

  for (const Block<VoxelType>& block : block_map_) {
    block.GetProto(proto->add_blocks());
  }
}

}  // namespace voxblox

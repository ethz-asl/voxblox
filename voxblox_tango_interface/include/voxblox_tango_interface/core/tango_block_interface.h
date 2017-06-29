#ifndef VOXBLOX_TANGO_INTERFACE_CORE_BLOCK_H_
#define VOXBLOX_TANGO_INTERFACE_CORE_BLOCK_H_

#include "voxblox/core/block.h"

#include "./Volume.pb.h"

namespace voxblox {

class TangoBlockInterface : public Block<TsdfVoxel> {
public:
  TangoBlockInterface(size_t voxels_per_side, FloatingPoint voxel_size,
                                          const Point& origin,
                                          unsigned int max_ntsdf_voxel_weight,
                                          FloatingPoint meters_to_ntsdf)
      : voxels_per_side_(voxels_per_side),
        voxel_size_(voxel_size),
        origin_(origin),
        has_data_(false),
        updated_(false),
        max_ntsdf_voxel_weight_(max_ntsdf_voxel_weight),
        meters_to_ntsdf_(meters_to_ntsdf) {
    num_voxels_ = voxels_per_side_ * voxels_per_side_ * voxels_per_side_;
    voxel_size_inv_ = 1.0 / voxel_size_;
    block_size_ = voxels_per_side_ * voxel_size_;
    block_size_inv_ = 1.0 / block_size_;
    voxels_.reset(new VoxelType[num_voxels_]);
  }

  TangoBlockInterface(const tsdf2:: VolumeProto& proto,
                      unsigned int max_ntsdf_voxel_weight,
                      FloatingPoint meters_to_ntsdf);

private:
  void deserializeProto(const tsdf2::VolumeProto& proto);
  void deserializeFromIntegers(const std::vector<uint32_t>& data);

  unsigned int max_ntsdf_voxel_weight_;
  FloatingPoint meters_to_ntsdf_;
};

}  // namespace voxblox

#endif  // VOXBLOX_TANGO_INTERFACE_CORE_BLOCK_H_

#ifndef VOXBLOX_TANGO_INTERFACE_CORE_BLOCK_H_
#define VOXBLOX_TANGO_INTERFACE_CORE_BLOCK_H_

#include "voxblox/core/block.h"
#include "voxblox/core/voxel.h"

#include "./Volume.pb.h"

namespace voxblox {

class TangoBlockInterface : public Block<TsdfVoxel> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TangoBlockInterface(size_t voxels_per_side, FloatingPoint voxel_size,
                      const Point& origin, unsigned int max_ntsdf_voxel_weight,
                      FloatingPoint meters_to_ntsdf)
      : Block<TsdfVoxel>(voxels_per_side, voxel_size, origin),
        max_ntsdf_voxel_weight_(max_ntsdf_voxel_weight),
        meters_to_ntsdf_(meters_to_ntsdf) {}

  TangoBlockInterface(const tsdf2::VolumeProto& proto,
                      unsigned int max_ntsdf_voxel_weight,
                      FloatingPoint meters_to_ntsdf);

 private:
  void deserializeProto(const tsdf2::VolumeProto& proto);
  void deserializeFromIntegers(const AlignedVector<uint32_t>& data);

  unsigned int max_ntsdf_voxel_weight_;
  FloatingPoint meters_to_ntsdf_;
};

}  // namespace voxblox

#include "voxblox_tango_interface/core/tango_block_interface_inl.h"

#endif  // VOXBLOX_TANGO_INTERFACE_CORE_BLOCK_H_

#ifndef VOXBLOX_TANGO_INTERFACE_CORE_LAYER_H_
#define VOXBLOX_TANGO_INTERFACE_CORE_LAYER_H_

#include "voxblox/core/layer.h"

#include "./MapHeader.pb.h"
#include "./Volume.pb.h"

#include "voxblox_tango_interface/core/tango_block_interface.h"

namespace voxblox {

class TangoLayerInterface : public Layer<TsdfVoxel> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // NOTE(mereweth@jpl.nasa.gov) - need this typedef
  typedef std::shared_ptr<TangoLayerInterface> Ptr;

  explicit TangoLayerInterface(const tsdf2::MapHeaderProto& proto);

  explicit TangoLayerInterface(const FloatingPoint voxel_size,
                               const size_t voxels_per_side,
                               const unsigned int max_ntsdf_voxel_weight,
                               const FloatingPoint meters_to_ntsdf)
      : Layer<TsdfVoxel>(voxel_size, voxels_per_side),
        max_ntsdf_voxel_weight_(max_ntsdf_voxel_weight),
        meters_to_ntsdf_(meters_to_ntsdf) {}

  bool isCompatible(const tsdf2::MapHeaderProto& layer_proto) const;
  bool isCompatible(const tsdf2::VolumeProto& block_proto) const;

  /* The audit flag allows converting a Tango NTSDF dump to TSDF without
   * scaling the distances or weights. This is a debugging feature.
   */
  bool addBlockFromProto(const tsdf2::VolumeProto& block_proto,
                         const TangoLayerInterface::BlockMergingStrategy strategy,
                         const bool audit = false);

 private:
  unsigned int max_ntsdf_voxel_weight_;
  FloatingPoint meters_to_ntsdf_;
};

}  // namespace voxblox

#include "voxblox_tango_interface/core/tango_layer_interface_inl.h"

#endif  // VOXBLOX_TANGO_INTERFACE_CORE_LAYER_H_

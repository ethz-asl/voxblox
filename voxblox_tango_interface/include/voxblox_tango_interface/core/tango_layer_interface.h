#ifndef VOXBLOX_TANGO_INTERFACE_CORE_LAYER_H_
#define VOXBLOX_TANGO_INTERFACE_CORE_LAYER_H_

#include "voxblox/core/layer.h"

#include "./MapHeader.pb.h"
#include "./Volume.pb.h"

#include "voxblox_tango_interface/core/tango_block_interface.h"

namespace voxblox {

class TangoLayerInterface : public Layer<TsdfVoxel> {/*,
                            public std::enable_shared_from_this<TangoLayerInterface> {*/
public:
  // NOTE(mereweth@jpl.nasa.gov) - need this typedef
  typedef std::shared_ptr<TangoLayerInterface> Ptr;

  explicit TangoLayerInterface(const tsdf2::MapHeaderProto& proto);

  // TODO(mereweth@jpl.nasa.gov) - best way to check not null?
  explicit TangoLayerInterface(std::shared_ptr<TangoLayerInterface> tango_layer_interface)
      : TangoLayerInterface(tango_layer_interface ? *tango_layer_interface :
                            TangoLayerInterface(0.2, 16u, 1, 1.0))
    { }

  explicit TangoLayerInterface(FloatingPoint voxel_size,
                               size_t voxels_per_side,
                               unsigned int max_ntsdf_voxel_weight,
                               FloatingPoint meters_to_ntsdf)
      : Layer<TsdfVoxel>(voxel_size, voxels_per_side),
        max_ntsdf_voxel_weight_(max_ntsdf_voxel_weight),
        meters_to_ntsdf_(meters_to_ntsdf)
    { }

  bool isCompatible(const tsdf2::MapHeaderProto& layer_proto) const;
  bool isCompatible(const tsdf2::VolumeProto& block_proto) const;
  bool addBlockFromProto(const tsdf2::VolumeProto& block_proto,
                         TangoLayerInterface::BlockMergingStrategy strategy);

private:
  unsigned int max_ntsdf_voxel_weight_;
  FloatingPoint meters_to_ntsdf_;
};

}  // namespace voxblox

#include "voxblox_tango_interface/core/tango_layer_interface_inl.h"

#endif  // VOXBLOX_TANGO_INTERFACE_CORE_LAYER_H_

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

#ifndef VOXBLOX_TANGO_INTERFACE_CORE_LAYER_H_
#define VOXBLOX_TANGO_INTERFACE_CORE_LAYER_H_

#include "voxblox/core/layer.h"

#include "./MapHeader.pb.h"
#include "./Volume.pb.h"

namespace voxblox {

class TangoLayerInterface : public Layer<TsdfVoxel> {
public:
  Layer(const tsdf2::MapHeaderProto& proto);

  bool isCompatible(const tsdf2::MapHeaderProto& layer_proto) const;
  bool isCompatible(const tsdf2::VolumeProto& block_proto) const;
  bool addBlockFromProto(const tsdf2::VolumeProto& block_proto,
                         BlockMergingStrategy strategy);
private:
  unsigned int max_ntsdf_voxel_weight_;
  FloatingPoint meters_to_ntsdf_;
};

}  // namespace voxblox

#endif  // VOXBLOX_TANGO_INTERFACE_CORE_LAYER_H_

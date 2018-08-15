#ifndef VOXBLOX_TANGO_INTERFACE_CORE_LAYER_INL_H_
#define VOXBLOX_TANGO_INTERFACE_CORE_LAYER_INL_H_

#include "./MapHeader.pb.h"
#include "./Volume.pb.h"

#include <iostream>
#include <iomanip>

#define VOXBLOX_TANGO_LAYER_INTERFACE_VOXEL_EPS (1e-8f)

namespace voxblox {

inline TangoLayerInterface::TangoLayerInterface(
    const tsdf2::MapHeaderProto& proto)
    : Layer<TsdfVoxel>(proto.voxel_size(), proto.voxels_per_volume_side()),
      max_ntsdf_voxel_weight_(proto.max_ntsdf_voxel_weight()),
      meters_to_ntsdf_(proto.meters_to_ntsdf()) {
  // Derived config parameter.
  block_size_ = voxel_size_ * voxels_per_side_;
  block_size_inv_ = 1.0 / block_size_;

  LOG(INFO) << "Meters to NTSDF: " << meters_to_ntsdf_ << "\t"
            << "Max NTSDF voxel weight: " << max_ntsdf_voxel_weight_ << "\n";

  CHECK_GT(proto.voxel_size(), 0.0);
  CHECK_GT(proto.voxels_per_volume_side(), 0u);
}

inline bool TangoLayerInterface::addBlockFromProto(
    const tsdf2::VolumeProto& block_proto,
    const TangoLayerInterface::BlockMergingStrategy strategy,
    const bool audit) {
  CHECK_EQ(getType().compare(voxel_types::kTsdf), 0)
      << "The voxel type of this layer is not TsdfVoxel!";

  if (isCompatible(block_proto)) {
    TangoBlockInterface::Ptr block_ptr(new TangoBlockInterface(
        block_proto, max_ntsdf_voxel_weight_, meters_to_ntsdf_, audit));

    const BlockIndex block_index = getGridIndexFromOriginPoint<BlockIndex>(
        block_ptr->origin(), block_size_inv_);
    switch (strategy) {
      case TangoLayerInterface::BlockMergingStrategy::kProhibit:
        CHECK_EQ(block_map_.count(block_index), 0u)
            << "Block collision at index: " << block_index;
        block_map_[block_index] = block_ptr;
        break;
      case TangoLayerInterface::BlockMergingStrategy::kReplace:
        block_map_[block_index] = block_ptr;
        break;
      case TangoLayerInterface::BlockMergingStrategy::kDiscard:
        block_map_.insert(std::make_pair(block_index, block_ptr));
        break;
      case TangoLayerInterface::BlockMergingStrategy::kMerge: {
        typename BlockHashMap::iterator it = block_map_.find(block_index);
        if (it == block_map_.end()) {
          block_map_[block_index] = block_ptr;
        } else {
          it->second->mergeBlock(*block_ptr);
        }
      } break;
      default:
        LOG(FATAL) << "Unknown BlockMergingStrategy: "
                   << static_cast<int>(strategy);
        return false;
    }
    // Mark that this block has been updated.
    block_map_[block_index]->updated() = true;
  } else {
    LOG(ERROR)
        << "The blocks from this protobuf are not compatible with this layer!";
    return false;
  }
  return true;
}

inline bool TangoLayerInterface::isCompatible(
    const tsdf2::MapHeaderProto& layer_proto) const {
  bool compatible = true;
  compatible &= (fabs(layer_proto.voxel_size() - voxel_size_) <=
                 VOXBLOX_TANGO_LAYER_INTERFACE_VOXEL_EPS);
  // NOTE(mereweth@jpl.nasa.gov) - MapHeader voxels_per_volume_side is a double
  compatible &= (fabs(layer_proto.voxels_per_volume_side() -
                      static_cast<double>(voxels_per_side_)) <=
                 VOXBLOX_TANGO_LAYER_INTERFACE_VOXEL_EPS);

  if (!compatible) {
    LOG(INFO) << std::setprecision(10)
              << "TangoLayerInterface voxel_size: " << voxel_size_
              << ", voxels_per_side: " << voxels_per_side_;
    LOG(INFO) << std::setprecision(10)
              << "MapHeaderProto voxel_size: " << layer_proto.voxel_size()
              << ", voxels_per_volume_side: "
              << layer_proto.voxels_per_volume_side();
  }

  return compatible;
}

inline bool TangoLayerInterface::isCompatible(
    const tsdf2::VolumeProto& block_proto) const {
  bool compatible = true;
  compatible &= (fabs(block_proto.voxel_size() - voxel_size_) <=
                 VOXBLOX_TANGO_LAYER_INTERFACE_VOXEL_EPS);
  // NOTE(mereweth@jpl.nasa.gov) - Volume voxels_per_side is an int32
  compatible &=
      (block_proto.voxels_per_side() == static_cast<int>(voxels_per_side_));

  if (!compatible) {
    LOG(INFO) << std::setprecision(10)
              << "TangoLayerInterface voxel_size: " << voxel_size_
              << ", voxels_per_side: " << voxels_per_side_;
    LOG(INFO) << std::setprecision(10)
              << "VolumeProto voxel_size: " << block_proto.voxel_size()
              << ", voxels_per_side: " << block_proto.voxels_per_side();
  }

  return compatible;
}

}  // namespace voxblox

#endif  // VOXBLOX_TANGO_INTERFACE_CORE_LAYER_INL_H_

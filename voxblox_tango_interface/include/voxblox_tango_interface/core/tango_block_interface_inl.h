#ifndef VOXBLOX_TANGO_INTERFACE_CORE_BLOCK_INL_H_
#define VOXBLOX_TANGO_INTERFACE_CORE_BLOCK_INL_H_

#include "./Volume.pb.h"

/* TODO(mereweth@jpl.nasa.gov) - this is not a template class, so no need to
 * have these definitions here
 */

namespace voxblox {

inline TangoBlockInterface::TangoBlockInterface(
    const tsdf2::VolumeProto& proto, const unsigned int max_ntsdf_voxel_weight,
    const FloatingPoint meters_to_ntsdf, const bool audit)
    : TangoBlockInterface(proto.voxels_per_side(), proto.voxel_size(),
                          proto.has_origin()
                              ? Point(proto.origin().x(), proto.origin().y(),
                                      proto.origin().z())
                              :
                              /* NOTE(mereweth@jpl.nasa.gov) - origin field
                               * seems to be deprecated
                               * as of 2017/06/15
                               * Without it, loading of old TSDF2 dumps is
                               * broken, so we
                               * pass it if present in the protobuf dump
                               */
                              Point(proto.index().x() * proto.voxel_size() *
                                        proto.voxels_per_side(),
                                    proto.index().y() * proto.voxel_size() *
                                        proto.voxels_per_side(),
                                    proto.index().z() * proto.voxel_size() *
                                        proto.voxels_per_side()),
                          max_ntsdf_voxel_weight, meters_to_ntsdf) {
  has_data_ = proto.has_data();

  // Convert the data into a vector of integers.
  AlignedVector<uint32_t> data;
  data.reserve(proto.ntsdf_voxels_size());

  auto ntsdf_word = proto.ntsdf_voxels().begin();
  auto color_word = proto.color_voxels().begin();
  for (; ntsdf_word != proto.ntsdf_voxels().end() &&
         color_word != proto.color_voxels().end();
       ++ntsdf_word, ++color_word) {
    data.push_back(*ntsdf_word);
    data.push_back(*color_word);
  }
  if (ntsdf_word != proto.ntsdf_voxels().end() ||
      color_word != proto.color_voxels().end()) {
    LOG(FATAL) << "Unequal number of tsdf and color voxels in Tango TSDF";
  }

  deserializeFromIntegers(data, audit);
}

}  // namespace voxblox

#endif  // VOXBLOX_TANGO_INTERFACE_CORE_BLOCK_INL_H_

#include "voxblox_tango_interface/core/tango_block_interface.h"

#include <cstdint>

namespace voxblox {

void TangoBlockInterface::deserializeFromIntegers(
    const AlignedVector<uint32_t>& data, const bool audit) {
  constexpr size_t kNumDataPacketsPerVoxel = 2u;
  const size_t num_data_packets = data.size();

  CHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, num_data_packets);
  for (size_t voxel_idx = 0u, data_idx = 0u;
       voxel_idx < num_voxels_ && data_idx < num_data_packets;
       ++voxel_idx, data_idx += kNumDataPacketsPerVoxel) {
    const uint32_t bytes_1 = data[data_idx];
    const uint32_t bytes_2 = data[data_idx + 1u];

    TsdfVoxel& voxel = voxels_[voxel_idx];

    // TODO(mereweth@jpl.nasa.gov) - is this the best way to unpack NTSDF?

    if (audit) {
      voxel.distance = static_cast<int16_t>(bytes_1 >> 16);
      voxel.weight = static_cast<uint16_t>(bytes_1 & 0x0000FFFF);
    } else {
      voxel.distance = static_cast<int16_t>(bytes_1 >> 16) / meters_to_ntsdf_;

      voxel.weight = static_cast<float>(max_ntsdf_voxel_weight_) /
                     static_cast<float>(UINT16_MAX) *
                     static_cast<uint16_t>(bytes_1 & 0x0000FFFF);
    }

    voxel.color.r = static_cast<uint8_t>(bytes_2 >> 24);
    voxel.color.g = static_cast<uint8_t>((bytes_2 & 0x00FF0000) >> 16);
    voxel.color.b = static_cast<uint8_t>((bytes_2 & 0x0000FF00) >> 8);
    voxel.color.a = static_cast<uint8_t>(bytes_2 & 0x000000FF);
  }
}

}  // namespace voxblox

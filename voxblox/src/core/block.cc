#include "voxblox/core/block.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

template <>
void Block<TsdfVoxel>::DeserializeVoxelData(const BlockProto& proto,
                                            TsdfVoxel* voxels) {
  constexpr size_t kNumDataPacketsPerVoxel = 3;
  const size_t num_data_packets = proto.voxel_data_size();
  DCHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, num_data_packets);
  for (size_t voxel_idx = 0u, data_idx = 0u;
       voxel_idx < num_voxels_ && data_idx < num_data_packets;
       ++voxel_idx, data_idx += 3u) {
    const uint32_t bytes_1 = proto.voxel_data(data_idx);
    const uint32_t bytes_2 = proto.voxel_data(data_idx + 1u);
    const uint32_t bytes_3 = proto.voxel_data(data_idx + 2u);

    TsdfVoxel& voxel = voxels[voxel_idx];

    // TODO(mfehr, helenol): find a better way to do this!

    memcpy(&(voxel.distance), &bytes_1, sizeof(bytes_1));
    memcpy(&(voxel.weight), &bytes_2, sizeof(bytes_2));

    voxel.color.r = static_cast<uint8_t>(bytes_3 >> 24);
    voxel.color.g = static_cast<uint8_t>((bytes_3 & 0x00FF0000) >> 16);
    voxel.color.b = static_cast<uint8_t>((bytes_3 & 0x0000FF00) >> 8);
    voxel.color.a = static_cast<uint8_t>(bytes_3 & 0x000000FF);
  }
}

template <>
void Block<TsdfVoxel>::SerializeVoxelData(const TsdfVoxel* voxels,
                                          BlockProto* proto) const {
  constexpr size_t kNumDataPacketsPerVoxel = 3;
  for (size_t voxel_idx = 0u; voxel_idx < num_voxels_; ++voxel_idx) {
    const TsdfVoxel& voxel = voxels[voxel_idx];

    // TODO(mfehr, helenol): find a better way to do this!

    const uint32_t* bytes_1_ptr =
        reinterpret_cast<const uint32_t*>(&voxel.distance);
    proto->add_voxel_data(*bytes_1_ptr);

    const uint32_t* bytes_2_ptr =
        reinterpret_cast<const uint32_t*>(&voxel.weight);
    proto->add_voxel_data(*bytes_2_ptr);

    proto->add_voxel_data(static_cast<uint32_t>(voxel.color.a) |
                          (static_cast<uint32_t>(voxel.color.b) << 8) |
                          (static_cast<uint32_t>(voxel.color.g) << 16) |
                          (static_cast<uint32_t>(voxel.color.r) << 24));
  }
  DCHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, proto->voxel_data_size());
}

// TODO(mfehr): Add serialization for EsdfVoxel

// TODO(mfehr): Add serialization for OccupancyVoxel

}  // namespace voxblox

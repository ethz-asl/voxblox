#include "voxblox/core/block.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

// Hidden serialization helpers:
uint8_t serializeDirection(const Eigen::Vector3i& dir) {
  uint8_t data = 0;
  // [0 0] = 0, [1 0] = -1, [0 1] = 1.
  uint8_t dir_x = dir.x() == 0 ? 0 : dir.x() > 0 ? 1 : 2;
  uint8_t dir_y = dir.y() == 0 ? 0 : dir.y() > 0 ? 1 : 2;
  uint8_t dir_z = dir.z() == 0 ? 0 : dir.z() > 0 ? 1 : 2;

  // Leading 2 bits are 0.
  data = dir_x << 4 | dir_y << 2 | dir_z;
  return data;
}

Eigen::Vector3i deserializeDirection(uint8_t data) {
  Eigen::Vector3i dir;
  uint8_t byte_x = data >> 4;
  uint8_t byte_y = (data >> 2) & (2 | 1);
  uint8_t byte_z = data & (2 | 1);

  // [0 0] = 0, [1 0] = -1, [0 1] = 1.
  dir.x() = byte_x == 0 ? 0 : byte_x == 2 ? -1 : 1;
  dir.y() = byte_y == 0 ? 0 : byte_y == 2 ? -1 : 1;
  dir.z() = byte_z == 0 ? 0 : byte_z == 2 ? -1 : 1;

  return dir;
}

// Deserialization functions:
template <>
void Block<TsdfVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data) {
  constexpr size_t kNumDataPacketsPerVoxel = 3u;
  const size_t num_data_packets = data.size();
  CHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, num_data_packets);
  for (size_t voxel_idx = 0u, data_idx = 0u;
       voxel_idx < num_voxels_ && data_idx < num_data_packets;
       ++voxel_idx, data_idx += kNumDataPacketsPerVoxel) {
    const uint32_t bytes_1 = data[data_idx];
    const uint32_t bytes_2 = data[data_idx + 1u];
    const uint32_t bytes_3 = data[data_idx + 2u];

    TsdfVoxel& voxel = voxels_[voxel_idx];

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
void Block<OccupancyVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data) {
  constexpr size_t kNumDataPacketsPerVoxel = 2u;
  const size_t num_data_packets = data.size();
  CHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, num_data_packets);
  for (size_t voxel_idx = 0u, data_idx = 0u;
       voxel_idx < num_voxels_ && data_idx < num_data_packets;
       ++voxel_idx, data_idx += kNumDataPacketsPerVoxel) {
    const uint32_t bytes_1 = data[data_idx];
    const uint32_t bytes_2 = data[data_idx + 1u];

    OccupancyVoxel& voxel = voxels_[voxel_idx];

    memcpy(&(voxel.probability_log), &bytes_1, sizeof(bytes_1));
    voxel.observed = static_cast<bool>(bytes_2 & 0x000000FF);
  }
}

template <>
void Block<EsdfVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data) {
  constexpr size_t kNumDataPacketsPerVoxel = 2u;
  const size_t num_data_packets = data.size();
  CHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, num_data_packets);
  for (size_t voxel_idx = 0u, data_idx = 0u;
       voxel_idx < num_voxels_ && data_idx < num_data_packets;
       ++voxel_idx, data_idx += kNumDataPacketsPerVoxel) {
    const uint32_t bytes_1 = data[data_idx];
    const uint32_t bytes_2 = data[data_idx + 1u];

    EsdfVoxel& voxel = voxels_[voxel_idx];

    memcpy(&(voxel.distance), &bytes_1, sizeof(bytes_1));

    voxel.observed = static_cast<bool>(bytes_2 & 0x000000FF);
    voxel.in_queue = static_cast<bool>(bytes_2 & 0x0000FF00);
    voxel.fixed = static_cast<bool>(bytes_2 & 0x00FF0000);
    voxel.parent =
        deserializeDirection(static_cast<uint8_t>(bytes_2 & 0xFF000000));
  }
}

// Serialization functions:
template <>
void Block<TsdfVoxel>::serializeToIntegers(std::vector<uint32_t>* data) const {
  CHECK_NOTNULL(data);
  constexpr size_t kNumDataPacketsPerVoxel = 3u;
  data->clear();
  data->reserve(num_voxels_ * kNumDataPacketsPerVoxel);
  for (size_t voxel_idx = 0u; voxel_idx < num_voxels_; ++voxel_idx) {
    const TsdfVoxel& voxel = voxels_[voxel_idx];

    // TODO(mfehr, helenol): find a better way to do this!
    const uint32_t* bytes_1_ptr =
        reinterpret_cast<const uint32_t*>(&voxel.distance);
    data->push_back(*bytes_1_ptr);

    const uint32_t* bytes_2_ptr =
        reinterpret_cast<const uint32_t*>(&voxel.weight);
    data->push_back(*bytes_2_ptr);

    data->push_back(static_cast<uint32_t>(voxel.color.a) |
                    (static_cast<uint32_t>(voxel.color.b) << 8) |
                    (static_cast<uint32_t>(voxel.color.g) << 16) |
                    (static_cast<uint32_t>(voxel.color.r) << 24));
  }
  CHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, data->size());
}

template <>
void Block<OccupancyVoxel>::serializeToIntegers(
    std::vector<uint32_t>* data) const {
  CHECK_NOTNULL(data);
  constexpr size_t kNumDataPacketsPerVoxel = 2u;
  data->clear();
  data->reserve(num_voxels_ * kNumDataPacketsPerVoxel);
  for (size_t voxel_idx = 0u; voxel_idx < num_voxels_; ++voxel_idx) {
    const OccupancyVoxel& voxel = voxels_[voxel_idx];

    const uint32_t* bytes_1_ptr =
        reinterpret_cast<const uint32_t*>(&voxel.probability_log);
    data->push_back(*bytes_1_ptr);
    data->push_back(static_cast<uint32_t>(voxel.observed));
  }
  CHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, data->size());
}

template <>
void Block<EsdfVoxel>::serializeToIntegers(std::vector<uint32_t>* data) const {
  CHECK_NOTNULL(data);
  constexpr size_t kNumDataPacketsPerVoxel = 2u;
  data->clear();
  data->reserve(num_voxels_ * kNumDataPacketsPerVoxel);
  for (size_t voxel_idx = 0u; voxel_idx < num_voxels_; ++voxel_idx) {
    const EsdfVoxel& voxel = voxels_[voxel_idx];

    const uint32_t* bytes_1_ptr =
        reinterpret_cast<const uint32_t*>(&voxel.distance);
    data->push_back(*bytes_1_ptr);
    // Repack this as a bunch of bools. Could also pack into a since uint8_t
    // to save space, but this maybe simpler.
    // observed is byte 1, in_queue is byte 2, fixed is byte 3,
    // then direction is packed into byte 4.
    uint8_t byte1 = voxel.observed;
    uint8_t byte2 = voxel.in_queue;
    uint8_t byte3 = voxel.fixed;
    // Packing here is a bit more creative. 2 bits per direction.
    // [0 0] = 0, [1 0] = -1, [0 1] = 1.
    uint8_t byte4 = serializeDirection(voxel.parent);
    data->push_back(static_cast<uint32_t>(byte1) |
                    (static_cast<uint32_t>(byte2) << 8) |
                    (static_cast<uint32_t>(byte3) << 16) |
                    (static_cast<uint32_t>(byte4) << 24));
  }
  CHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, data->size());
}

}  // namespace voxblox

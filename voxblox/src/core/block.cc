#include "voxblox/core/block.h"

#include "voxblox/core/voxel.h"

namespace voxblox {

// Hidden serialization helpers
void serializeDirection(const Eigen::Vector3i& parent_direction,
                        uint32_t* data) {
  CHECK_NOTNULL(data);
  CHECK_EQ(*data, 0u);

  // NOTE: these checks will fail until the TODO below is adressed.
  DCHECK_GE(parent_direction.x(), INT8_MIN);
  DCHECK_LE(parent_direction.x(), INT8_MAX);

  DCHECK_GE(parent_direction.y(), INT8_MIN);
  DCHECK_LE(parent_direction.y(), INT8_MAX);

  DCHECK_GE(parent_direction.z(), INT8_MIN);
  DCHECK_LE(parent_direction.z(), INT8_MAX);

  // TODO(helenol, mfehr):
  // Serialization should never change the values, the parent values will either
  // need to be truncated in the ESDF integrator or we need to use more bytes to
  // serialize the ESDF voxel.
  const int8_t parent_direction_x =
      std::min(INT8_MAX, std::max(parent_direction.x(), INT8_MIN));
  const int8_t parent_direction_y =
      std::min(INT8_MAX, std::max(parent_direction.y(), INT8_MIN));
  const int8_t parent_direction_z =
      std::min(INT8_MAX, std::max(parent_direction.z(), INT8_MIN));

  // Layout:
  // | 3x8bit (int8_t) for parent (X,Y,Z) | 8bit ESDF |

  *data |= static_cast<uint32_t>(static_cast<int8_t>(parent_direction_x) << 24);
  *data |= static_cast<uint32_t>(static_cast<int8_t>(parent_direction_y) << 16);
  *data |= static_cast<uint32_t>(static_cast<int8_t>(parent_direction_z) << 8);
}

Eigen::Vector3i deserializeDirection(const uint32_t data) {
  // Layout:
  // | 3x8bit (int8_t) for parent (X,Y,Z) | 8bit ESDF |

  Eigen::Vector3i parent_direction;

  parent_direction.x() = static_cast<int8_t>((data >> 24) & 0x000000FF);
  parent_direction.y() = static_cast<int8_t>((data >> 16) & 0x000000FF);
  parent_direction.z() = static_cast<int8_t>((data >> 8) & 0x000000FF);

  DCHECK_GE(parent_direction.x(), INT8_MIN);
  DCHECK_LE(parent_direction.x(), INT8_MAX);

  DCHECK_GE(parent_direction.y(), INT8_MIN);
  DCHECK_LE(parent_direction.y(), INT8_MAX);

  DCHECK_GE(parent_direction.z(), INT8_MIN);
  DCHECK_LE(parent_direction.z(), INT8_MAX);

  return parent_direction;
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
    // Layout:
    // | 32 bit sdf | 3x8bit (int8_t) parent | 8 bit flags|

    const uint32_t bytes_1 = data[data_idx];
    const uint32_t bytes_2 = data[data_idx + 1u];

    EsdfVoxel& voxel = voxels_[voxel_idx];

    memcpy(&(voxel.distance), &bytes_1, sizeof(bytes_1));

    voxel.observed = static_cast<bool>(bytes_2 & 0x00000001);
    voxel.hallucinated = static_cast<bool>((bytes_2 & 0x00000002));
    voxel.in_queue = static_cast<bool>((bytes_2 & 0x00000004));
    voxel.fixed = static_cast<bool>((bytes_2 & 0x00000008));

    voxel.parent = deserializeDirection(bytes_2);
  }
}

template <>
void Block<IntensityVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data) {
  constexpr size_t kNumDataPacketsPerVoxel = 2u;
  const size_t num_data_packets = data.size();
  CHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, num_data_packets);
  for (size_t voxel_idx = 0u, data_idx = 0u;
       voxel_idx < num_voxels_ && data_idx < num_data_packets;
       ++voxel_idx, data_idx += kNumDataPacketsPerVoxel) {
    const uint32_t bytes_1 = data[data_idx];
    const uint32_t bytes_2 = data[data_idx + 1u];

    IntensityVoxel& voxel = voxels_[voxel_idx];

    memcpy(&(voxel.intensity), &bytes_1, sizeof(bytes_1));
    memcpy(&(voxel.weight), &bytes_2, sizeof(bytes_2));
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

    // Current Layout:
    // | 32 bit sdf | 3x8bit (int8_t) parent | 8 bit flags|

    const uint32_t* bytes_1_ptr =
        reinterpret_cast<const uint32_t*>(&voxel.distance);
    data->push_back(*bytes_1_ptr);

    uint32_t bytes_2 = 0u;
    serializeDirection(voxel.parent, &bytes_2);

    uint8_t flag_byte = 0b00000000;
    flag_byte |= static_cast<uint8_t>(voxel.observed ? 0b00000001 : 0b00000000);
    flag_byte |=
        static_cast<uint8_t>(voxel.hallucinated ? 0b00000010 : 0b00000000);
    flag_byte |= static_cast<uint8_t>(voxel.in_queue ? 0b00000100 : 0b00000000);
    flag_byte |= static_cast<uint8_t>(voxel.fixed ? 0b00001000 : 0b00000000);

    bytes_2 |= static_cast<uint32_t>(flag_byte) & 0x000000FF;

    data->push_back(bytes_2);
  }
  CHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, data->size());
}

template <>
void Block<IntensityVoxel>::serializeToIntegers(
    std::vector<uint32_t>* data) const {
  CHECK_NOTNULL(data);
  constexpr size_t kNumDataPacketsPerVoxel = 2u;
  data->clear();
  data->reserve(num_voxels_ * kNumDataPacketsPerVoxel);
  for (size_t voxel_idx = 0u; voxel_idx < num_voxels_; ++voxel_idx) {
    const IntensityVoxel& voxel = voxels_[voxel_idx];

    const uint32_t* bytes_1_ptr =
        reinterpret_cast<const uint32_t*>(&voxel.intensity);
    data->push_back(*bytes_1_ptr);

    const uint32_t* bytes_2_ptr =
        reinterpret_cast<const uint32_t*>(&voxel.weight);
    data->push_back(*bytes_2_ptr);
  }
  CHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, data->size());
}

}  // namespace voxblox

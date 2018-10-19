#include "voxblox/io/sdf_ply.h"

#include <algorithm>

namespace voxblox {

namespace io {

template <>
bool getColorFromVoxel(const TsdfVoxel& voxel, const float sdf_color_range,
                       const float sdf_max_value, Color* color) {
  CHECK_NOTNULL(color);

  static constexpr float kTolerance = 1e-6;
  if (voxel.weight <= kTolerance) {
    return false;
  }

  if (std::abs(voxel.distance) > sdf_max_value && sdf_max_value > 0.0f) {
    return false;
  }

  const float truncated_voxel_distance =
      std::min(std::max(voxel.distance, -sdf_color_range), sdf_color_range);

  const float color_factor =
      0.5f * (1.0f + (truncated_voxel_distance / sdf_color_range));

  CHECK_LE(color_factor, 1.0f);
  CHECK_GE(color_factor, 0.0f);

  *color = rainbowColorMap(0.66f - 0.66f * color_factor);

  return true;
}

template <>
bool getColorFromVoxel(const EsdfVoxel& voxel, const float sdf_color_range,
                       const float sdf_max_value, Color* color) {
  CHECK_NOTNULL(color);
  if (!voxel.observed) {
    return false;
  }

  if (std::abs(voxel.distance) > sdf_max_value && sdf_max_value > 0.0f) {
    return false;
  }

  const float truncated_voxel_distance =
      std::min(std::max(voxel.distance, -sdf_color_range), sdf_color_range);

  const float color_factor =
      0.5f * (1.0f + (truncated_voxel_distance / sdf_color_range));

  CHECK_LE(color_factor, 1.0f);
  CHECK_GE(color_factor, 0.0f);

  *color = rainbowColorMap(0.66f - 0.66f * color_factor);

  return true;
}

}  // namespace io
}  // namespace voxblox

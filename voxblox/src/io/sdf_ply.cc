#include "voxblox/io/sdf_ply.h"

#include <algorithm>

#include "voxblox/core/layer.h"

namespace voxblox {

namespace io {

template <>
bool getColorFromVoxel(const TsdfVoxel& voxel,
                       const FloatingPoint sdf_color_range,
                       const FloatingPoint sdf_max_value, Color* color) {
  CHECK_NOTNULL(color);

  static constexpr float kTolerance = 1e-6;
  if (voxel.weight <= kTolerance) {
    return false;
  }

  if (std::abs(voxel.distance) > sdf_max_value && sdf_max_value > 0.0) {
    return false;
  }

  FloatingPoint truncated_voxel_distance =
      std::min(std::max(voxel.distance, -sdf_color_range), sdf_color_range);

  FloatingPoint color_factor =
      0.5 * (1.0 + (truncated_voxel_distance / sdf_color_range));

  CHECK_LE(color_factor, 1.0);
  CHECK_GE(color_factor, 0.0);

  *color = rainbowColorMap(0.66 - 0.66 * color_factor);

  return true;
}

template <>
bool getColorFromVoxel(const EsdfVoxel& voxel,
                       const FloatingPoint sdf_color_range,
                       const FloatingPoint sdf_max_value, Color* color) {
  CHECK_NOTNULL(color);
  if (!voxel.observed) {
    return false;
  }

  if (std::abs(voxel.distance) > sdf_max_value && sdf_max_value > 0.0) {
    return false;
  }

  FloatingPoint truncated_voxel_distance =
      std::min(std::max(voxel.distance, -sdf_color_range), sdf_color_range);

  FloatingPoint color_factor =
      0.5 * (1.0 + (truncated_voxel_distance / sdf_color_range));

  CHECK_LE(color_factor, 1.0);
  CHECK_GE(color_factor, 0.0);

  *color = rainbowColorMap(0.66 - 0.66 * color_factor);

  return true;
}

}  // namespace io
}  // namespace voxblox

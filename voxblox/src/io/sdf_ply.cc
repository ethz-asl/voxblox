#include "voxblox/io/sdf_ply.h"

#include <algorithm>

#include "voxblox/core/layer.h"

namespace voxblox {

namespace io {

template <>
bool getColorFromVoxel(const TsdfVoxel& voxel, const FloatingPoint max_distance,
                       Color* color) {
  CHECK_NOTNULL(color);

  static constexpr float kTolerance = 1e-6;
  if (voxel.weight <= kTolerance) {
    return false;
  }

  // Decide how to color this.
  // Distance > 0 = blue, distance < 0 = red.
  Color distance_color = Color::blendTwoColors(
      Color(255, 0, 0, 0),
      std::max<float>(1 - voxel.distance / max_distance, 0.0),
      Color(0, 0, 255, 0),
      std::max<float>(1 + voxel.distance / max_distance, 0.0));

  *color = distance_color;
  return true;
}

template <>
bool getColorFromVoxel(const EsdfVoxel& voxel, const FloatingPoint max_distance,
                       Color* color) {
  CHECK_NOTNULL(color);
  if (!voxel.observed) {
    return false;
  }

  // Decide how to color this.
  // Distance > 0 = blue, distance < 0 = red.
  Color distance_color = Color::blendTwoColors(
      Color(255, 0, 0, 0),
      std::max<float>(1 - voxel.distance / max_distance, 0.0),
      Color(0, 0, 255, 0),
      std::max<float>(1 + voxel.distance / max_distance, 0.0));

  *color = distance_color;
  return true;
}

}  // namespace io
}  // namespace voxblox

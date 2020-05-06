#include "voxblox/utils/layer_utils.h"

namespace voxblox {

namespace utils {

template <>
bool isSameVoxel(const TsdfVoxel& voxel_A, const TsdfVoxel& voxel_B) {
  bool is_the_same = true;

  constexpr double kTolerance = 1e-10;

  is_the_same &= std::abs(voxel_A.distance - voxel_B.distance) < kTolerance;
  is_the_same &= std::abs(voxel_A.weight - voxel_B.weight) < kTolerance;
  is_the_same &= voxel_A.color.r == voxel_B.color.r;
  is_the_same &= voxel_A.color.g == voxel_B.color.g;
  is_the_same &= voxel_A.color.b == voxel_B.color.b;
  is_the_same &= voxel_A.color.a == voxel_B.color.a;

  return is_the_same;
}

template <>
bool isSameVoxel(const EsdfVoxel& voxel_A, const EsdfVoxel& voxel_B) {
  bool is_the_same = true;

  constexpr double kTolerance = 1e-10;

  is_the_same &= std::abs(voxel_A.distance - voxel_B.distance) < kTolerance;
  is_the_same &= voxel_A.observed == voxel_B.observed;
  is_the_same &= voxel_A.in_queue == voxel_B.in_queue;
  is_the_same &= voxel_A.fixed == voxel_B.fixed;

  is_the_same &= voxel_A.parent.x() == voxel_B.parent.x();
  is_the_same &= voxel_A.parent.y() == voxel_B.parent.y();
  is_the_same &= voxel_A.parent.z() == voxel_B.parent.z();

  return is_the_same;
}

template <>
bool isSameVoxel(const OccupancyVoxel& voxel_A, const OccupancyVoxel& voxel_B) {
  bool is_the_same = true;

  constexpr double kTolerance = 1e-10;

  is_the_same &=
      std::abs(voxel_A.probability_log - voxel_B.probability_log) < kTolerance;
  is_the_same &= voxel_A.observed == voxel_B.observed;

  return is_the_same;
}

}  // namespace utils
}  // namespace voxblox

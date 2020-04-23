#include "voxblox/utils/evaluation_utils.h"

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

namespace utils {

template <>
VoxelEvaluationResult computeVoxelError(
    const TsdfVoxel& voxel_gt, const TsdfVoxel& voxel_test,
    const VoxelEvaluationMode evaluation_mode, FloatingPoint* error) {
  CHECK_NOTNULL(error);
  *error = 0.0;

  // Ignore voxels that are not observed in both layers.
  if (!isObservedVoxel(voxel_gt) || !isObservedVoxel(voxel_test)) {
    return VoxelEvaluationResult::kNoOverlap;
  }

  const bool ignore_behind_test_surface =
      (evaluation_mode == VoxelEvaluationMode::kIgnoreErrorBehindTestSurface) ||
      (evaluation_mode == VoxelEvaluationMode::kIgnoreErrorBehindAllSurfaces);

  const bool ignore_behind_gt_surface =
      (evaluation_mode == VoxelEvaluationMode::kIgnoreErrorBehindGtSurface) ||
      (evaluation_mode == VoxelEvaluationMode::kIgnoreErrorBehindAllSurfaces);

  if ((ignore_behind_test_surface && voxel_test.distance < 0.0) ||
      (ignore_behind_gt_surface && voxel_gt.distance < 0.0)) {
    return VoxelEvaluationResult::kIgnored;
  }

  *error = voxel_test.distance - voxel_gt.distance;

  return VoxelEvaluationResult::kEvaluated;
}

template <>
VoxelEvaluationResult computeVoxelError(
    const EsdfVoxel& voxel_gt, const EsdfVoxel& voxel_test,
    const VoxelEvaluationMode evaluation_mode, FloatingPoint* error) {
  CHECK_NOTNULL(error);
  *error = 0.0;

  // Ignore voxels that are not observed in both layers.
  if (!isObservedVoxel(voxel_gt) || !isObservedVoxel(voxel_test)) {
    return VoxelEvaluationResult::kNoOverlap;
  }

  const bool ignore_behind_test_surface =
      (evaluation_mode == VoxelEvaluationMode::kIgnoreErrorBehindTestSurface) ||
      (evaluation_mode == VoxelEvaluationMode::kIgnoreErrorBehindAllSurfaces);

  const bool ignore_behind_gt_surface =
      (evaluation_mode == VoxelEvaluationMode::kIgnoreErrorBehindGtSurface) ||
      (evaluation_mode == VoxelEvaluationMode::kIgnoreErrorBehindAllSurfaces);

  if ((ignore_behind_test_surface && voxel_test.distance < 0.0) ||
      (ignore_behind_gt_surface && voxel_gt.distance < 0.0)) {
    return VoxelEvaluationResult::kIgnored;
  }

  *error = voxel_test.distance - voxel_gt.distance;

  return VoxelEvaluationResult::kEvaluated;
}

template <typename VoxelType>
bool isObservedVoxel(const VoxelType& /*voxel*/) {
  return false;
}

template <>
bool isObservedVoxel(const TsdfVoxel& voxel) {
  return voxel.weight > 1e-6;
}

template <>
bool isObservedVoxel(const EsdfVoxel& voxel) {
  return voxel.observed;
}

template <>
FloatingPoint getVoxelSdf(const TsdfVoxel& voxel) {
  return voxel.distance;
}

template <>
FloatingPoint getVoxelSdf(const EsdfVoxel& voxel) {
  return voxel.distance;
}

template <>
void setVoxelSdf(const FloatingPoint sdf, TsdfVoxel* voxel) {
  CHECK_NOTNULL(voxel);
  voxel->distance = sdf;
}

template <>
void setVoxelSdf(const FloatingPoint sdf, EsdfVoxel* voxel) {
  CHECK_NOTNULL(voxel);
  voxel->distance = sdf;
}

template <>
void setVoxelWeight(const FloatingPoint weight, TsdfVoxel* voxel) {
  CHECK_NOTNULL(voxel);
  voxel->weight = weight;
}

template <>
void setVoxelWeight(const FloatingPoint weight, EsdfVoxel* voxel) {
  CHECK_NOTNULL(voxel);
  voxel->observed = weight > 0.;
}

}  // namespace utils
}  // namespace voxblox

#ifndef VOXBLOX_UTILS_EVALUATION_UTILS_H_
#define VOXBLOX_UTILS_EVALUATION_UTILS_H_

#include <algorithm>
#include <string>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

namespace voxblox {
namespace utils {

enum class VoxelEvaluationMode {
  kEvaluateAllVoxels,
  kIgnoreErrorBehindTestSurface,
  kIgnoreErrorBehindGtSurface,
  kIgnoreErrorBehindAllSurfaces
};

struct VoxelEvaluationDetails {
  FloatingPoint rmse = 0.0;
  // Max and min of absolute distance error.
  FloatingPoint max_error = 0.0;
  FloatingPoint min_error = 0.0;
  size_t num_evaluated_voxels = 0u;
  size_t num_ignored_voxels = 0u;
  size_t num_overlapping_voxels = 0u;
  size_t num_non_overlapping_voxels = 0u;
  size_t num_gt_un_test_un = 0u;
  size_t num_gt_un_test_occ = 0u;
  size_t num_gt_un_test_free = 0u;
  size_t num_gt_occ_test_un = 0u;
  size_t num_gt_occ_test_occ = 0u;
  size_t num_gt_occ_test_free = 0u;
  size_t num_gt_free_test_un = 0u;
  size_t num_gt_free_test_occ = 0u;
  size_t num_gt_free_test_free = 0u;

  size_t num_observed_voxels_layer_gt = 0u;

  // These values rely on the threshold below. Voxels that have a distance to
  // the surface that is larger than this factor x voxel size are counted as
  // free space.
  static constexpr float kFreeSpaceThresholdFactor = 1.0;
  float free_space_threshold = 0.0;

  size_t num_erroneous_occupied_voxels = 0u;
  size_t num_erroneous_free_voxels = 0u;

  std::string toString() const {
    size_t num_gt_occ = num_gt_occ_test_occ + num_gt_occ_test_free + num_gt_occ_test_un;
    size_t num_gt_free = num_gt_free_test_occ + num_gt_free_test_free + num_gt_free_test_un;
    double false_pos = 100.0 * (num_gt_free_test_occ + num_gt_free_test_un) / num_gt_free;
    double false_neg = 100.0 * num_gt_occ_test_free / num_gt_occ;
    double coverage = 100.0 * num_overlapping_voxels / num_observed_voxels_layer_gt;
    std::stringstream ss;
    ss << "\n\n======= Layer Evaluation Results =======\n"
       << "\n num evaluated voxels:           " << num_evaluated_voxels
       << "\n num overlapping voxels:         " << num_overlapping_voxels
       << "\n num non-overlapping voxels:     " << num_non_overlapping_voxels
       << "\n num ignored voxels:             " << num_ignored_voxels
       << "\n num observed voxels layer gt:   " << num_observed_voxels_layer_gt
       << "\n num erroneous occupied voxels:  " << num_erroneous_occupied_voxels
       << "\n num erroneous free voxels:      " << num_erroneous_free_voxels
       << "\n min error:                      " << min_error
       << "\n max error:                      " << max_error
       << "\n num_gt_un_test_un:              " << num_gt_un_test_un
       << "\n num_gt_un_test_occ:             " << num_gt_un_test_occ
       << "\n num_gt_un_test_free:            " << num_gt_un_test_free
       << "\n num_gt_occ_test_un:             " << num_gt_occ_test_un
       << "\n num_gt_occ_test_occ:            " << num_gt_occ_test_occ
       << "\n num_gt_occ_test_free:           " << num_gt_occ_test_free
       << "\n num_gt_free_test_un:            " << num_gt_free_test_un
       << "\n num_gt_free_test_occ:           " << num_gt_free_test_occ
       << "\n num_gt_free_test_free:          " << num_gt_free_test_free
       << "\n False Postive  [%]:             " << false_pos
       << "\n False Negative [%]:             " << false_neg
       << "\n RMSE           [m]:             " << rmse
       << "\n Coverage       [%]:             " << coverage
        << "\n========================================\n";

    return ss.str();
  }
};

template <typename VoxelType>
bool computeVoxelError(const VoxelType& voxel_gt, const VoxelType& voxel_test,
                       const VoxelEvaluationMode evaluation_mode,
                       VoxelEvaluationDetails* eval_details,
                       FloatingPoint* error);

// Returns true if the voxel has been observed.
template <typename VoxelType>
bool isObservedVoxel(const VoxelType& voxel);

// First check if voxel is observed, if it's not initialized, the distance might not be initialized aswell.
template <typename VoxelType>
bool isIgnoredVoxel(const VoxelType& voxel, bool ignore_behind_surface) {
  return ignore_behind_surface && voxel.distance < 0.0;
}

// Allow this class to be templated on all kinds of voxels.
template <typename VoxelType>
FloatingPoint getVoxelSdf(const VoxelType& voxel);

template <typename VoxelType>
void setVoxelSdf(const FloatingPoint sdf, VoxelType* voxel);

template <typename VoxelType>
void setVoxelWeight(const FloatingPoint weight, VoxelType* voxel);

// Evaluate a test layer vs a ground truth layer. The comparison is symmetrical
// unless the VoxelEvaluationMode is set to ignore the voxels of one of the two
// layers behind the surface. The parameter 'evaluation_result' and
// 'error_layer' can be a nullptr.
template <typename VoxelType>
FloatingPoint evaluateLayersRmse(
    const Layer<VoxelType>& layer_gt, const Layer<VoxelType>& layer_test,
    const VoxelEvaluationMode& voxel_evaluation_mode,
    VoxelEvaluationDetails* evaluation_result = nullptr,
    Layer<VoxelType>* error_layer = nullptr) {
  // Iterate over all voxels in the test layer and look them up in the ground
  // truth layer. Then compute RMSE.
  size_t vps = layer_test.voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  VoxelEvaluationDetails evaluation_details;

  // Initialize free space threshold based on voxel size if not evaluated yet
  evaluation_details.free_space_threshold =
    VoxelEvaluationDetails::kFreeSpaceThresholdFactor * layer_gt.voxel_size();
  // Kinda hacky way to specifiy free_space_threshold
  if(evaluation_result && evaluation_result->free_space_threshold > 0.0) {
    evaluation_details.free_space_threshold = evaluation_result->free_space_threshold;
  }

  const bool ignore_behind_test_surface =
      (voxel_evaluation_mode == VoxelEvaluationMode::kIgnoreErrorBehindTestSurface) ||
      (voxel_evaluation_mode == VoxelEvaluationMode::kIgnoreErrorBehindAllSurfaces);

  const bool ignore_behind_gt_surface =
      (voxel_evaluation_mode == VoxelEvaluationMode::kIgnoreErrorBehindGtSurface) ||
      (voxel_evaluation_mode == VoxelEvaluationMode::kIgnoreErrorBehindAllSurfaces);

  double total_squared_error = 0.0;

  BlockIndexList test_block_list;
  layer_test.getAllAllocatedBlocks(&test_block_list);
  for (const BlockIndex& block_index : test_block_list) {
    const Block<VoxelType>& test_block = layer_test.getBlockByIndex(block_index);

    // The ground truth layer does not have that block, i.e. voxel is unknown
    if (!layer_gt.hasBlock(block_index)) {
      for (size_t linear_index = 0u; linear_index < num_voxels_per_block; ++linear_index) {
        const VoxelType& test_voxel = test_block.getVoxelByLinearIndex(linear_index);
        if (isObservedVoxel(test_voxel)) {
          if (isIgnoredVoxel(test_voxel, ignore_behind_test_surface)) {
            ++evaluation_details.num_ignored_voxels;
            ++evaluation_details.num_gt_un_test_un;
          } else {
            ++evaluation_details.num_non_overlapping_voxels;
            const bool test_voxel_is_free = getVoxelSdf(test_voxel) > evaluation_details.free_space_threshold;
            if(test_voxel_is_free) {
              ++evaluation_details.num_gt_un_test_free;
            } else {
              ++evaluation_details.num_gt_un_test_occ;
            }
          }
        } else {
          ++evaluation_details.num_gt_un_test_un;
        }
        continue;
      }
    const Block<VoxelType>& gt_block = layer_gt.getBlockByIndex(block_index);

    typename Block<VoxelType>::Ptr error_block;
    if (error_layer != nullptr) {
      error_block = error_layer->allocateBlockPtrByIndex(block_index);
    }

    for (size_t linear_index = 0u; linear_index < num_voxels_per_block; ++linear_index) {
      FloatingPoint error = 0.0;
      if (computeVoxelError(gt_block.getVoxelByLinearIndex(linear_index),
                            test_block.getVoxelByLinearIndex(linear_index),
                            voxel_evaluation_mode, &evaluation_details,
                            &error)) {
        if (error_block) {
          VoxelType& error_voxel = error_block->getVoxelByLinearIndex(linear_index);
          setVoxelSdf<VoxelType>(std::abs(error), &error_voxel);
          setVoxelWeight<VoxelType>(1.0, &error_voxel);
        }

        total_squared_error += error * error;
      }
    }
  }

  // Iterate over all blocks in the grond truth layer and look them up in the
  // test truth layer. This is only done to get the exact number of
  // non-overlapping voxels.
  BlockIndexList gt_block_list;
  layer_gt.getAllAllocatedBlocks(&gt_block_list);
  for (const BlockIndex& gt_block_index : gt_block_list) {
    const Block<VoxelType>& gt_block = layer_gt.getBlockByIndex(gt_block_index);
    if (!layer_test.hasBlock(gt_block_index)) {
      for (size_t linear_index = 0u; linear_index < num_voxels_per_block;
           ++linear_index) {
        const VoxelType& gt_voxel = gt_block.getVoxelByLinearIndex(linear_index);
        // Block does not exist in test (i.e. voxel unoccupied)
        if(isObservedVoxel(gt_voxel)) {
          if (isIgnoredVoxel(gt_voxel, ignore_behind_gt_surface)) {
            ++evaluation_details.num_ignored_voxels;
            ++evaluation_details.num_gt_un_test_un;
          } else {
            ++evaluation_details.num_non_overlapping_voxels;
            ++evaluation_details.num_observed_voxels_layer_gt;
            const bool gt_voxel_is_free = getVoxelSdf(gt_voxel) > evaluation_details.free_space_threshold;
            if (gt_voxel_is_free) {
              ++evaluation_details.num_gt_free_test_un;
            } else {
              ++evaluation_details.num_gt_occ_test_un;
            }
          }
        } else {
          ++evaluation_details.num_gt_un_test_un;
        }
      }
    }
  }

  // Return the RMSE.
  if (evaluation_details.num_evaluated_voxels == 0) {
    evaluation_details.rmse = 0.0;
  } else {
    evaluation_details.rmse =
        sqrt(total_squared_error / evaluation_details.num_evaluated_voxels);
  }

  // If the details are requested, output them.
  if (evaluation_result != nullptr) {
    *evaluation_result = evaluation_details;
  }

  VLOG(2) << evaluation_details.toString();

  return evaluation_details.rmse;
}

// Overload for convenient RMSE calculation. Per default this function does not
// evaluate errors behind the test surface.
template <typename VoxelType>
FloatingPoint evaluateLayersRmse(const Layer<VoxelType>& layer_gt,
                                 const Layer<VoxelType>& layer_test) {
  return evaluateLayersRmse<VoxelType>(
      layer_gt, layer_test, VoxelEvaluationMode::kIgnoreErrorBehindTestSurface);
}

template <typename VoxelType>
bool computeVoxelError(const VoxelType& voxel_gt, const VoxelType& voxel_test,
                       const VoxelEvaluationMode evaluation_mode,
                       VoxelEvaluationDetails* eval_details,
                       FloatingPoint* error) {
  CHECK_NOTNULL(eval_details);
  CHECK_NOTNULL(error);

  // Init error.
  *error = 0.0;

  const bool ignore_behind_test_surface =
      (evaluation_mode == VoxelEvaluationMode::kIgnoreErrorBehindTestSurface) ||
      (evaluation_mode == VoxelEvaluationMode::kIgnoreErrorBehindAllSurfaces);

  const bool ignore_behind_gt_surface =
      (evaluation_mode == VoxelEvaluationMode::kIgnoreErrorBehindGtSurface) ||
      (evaluation_mode == VoxelEvaluationMode::kIgnoreErrorBehindAllSurfaces);

  const bool gt_observed = isObservedVoxel(voxel_gt) && !isIgnoredVoxel(voxel_gt, ignore_behind_gt_surface);
  const bool test_observed = isObservedVoxel(voxel_test) && !isIgnoredVoxel(voxel_test, ignore_behind_test_surface);

  if (isIgnoredVoxel(voxel_gt, ignore_behind_gt_surface)) {
    ++eval_details->num_ignored_voxels;
  }
  if (gt_observed) {
    ++eval_details->num_observed_voxels_layer_gt;
  }
  if (isIgnoredVoxel(voxel_test, ignore_behind_test_surface)) {
    ++eval_details->num_ignored_voxels;
  }

  const bool both_voxels_observed = gt_observed && test_observed;
  const bool test_voxel_is_free =
      getVoxelSdf(voxel_test) > eval_details->free_space_threshold;
  const bool gt_voxel_is_free =
      getVoxelSdf(voxel_gt) > eval_details->free_space_threshold;

  if (gt_observed) {
    if (gt_voxel_is_free) {
      if (test_observed) {
        if (test_voxel_is_free) { ++eval_details->num_gt_free_test_free; }
        else { ++eval_details->num_gt_free_test_occ; }
      } else { ++eval_details->num_gt_free_test_un; }
    } else {
      if (test_observed) {
        if (test_voxel_is_free) { ++eval_details->num_gt_occ_test_free; }
        else { ++eval_details->num_gt_occ_test_occ; }
      } else { ++eval_details->num_gt_occ_test_un; }
    }
  } else if (test_observed) { // test_observed, !gt_observed
    if (test_voxel_is_free) { ++eval_details->num_gt_un_test_free; }
    else { ++eval_details->num_gt_un_test_occ; }
  } else { ++eval_details->num_gt_un_test_un; } // neither observed

  // Ignore voxels that are not observed in both layers.
  if (!both_voxels_observed) {
    ++eval_details->num_non_overlapping_voxels;
    // There is no overlap.
    return false;
  }

  *error = getVoxelSdf(voxel_test) - getVoxelSdf(voxel_gt);

  eval_details->min_error = std::min(eval_details->min_error, std::abs(*error));
  eval_details->max_error = std::max(eval_details->max_error, std::abs(*error));
  ++eval_details->num_evaluated_voxels;
  ++eval_details->num_overlapping_voxels;

  return true;
}

template <>
bool isObservedVoxel(const TsdfVoxel& voxel);
template <>
bool isObservedVoxel(const EsdfVoxel& voxel);
template <>
FloatingPoint getVoxelSdf(const TsdfVoxel& voxel);
template <>
FloatingPoint getVoxelSdf(const EsdfVoxel& voxel);
template <>
void setVoxelSdf(const FloatingPoint sdf, TsdfVoxel* voxel);
template <>
void setVoxelSdf(const FloatingPoint sdf, EsdfVoxel* voxel);
template <>
void setVoxelWeight(const FloatingPoint weight, TsdfVoxel* voxel);
template <>
void setVoxelWeight(const FloatingPoint weight, EsdfVoxel* voxel);

}  // namespace utils
}  // namespace voxblox

#endif  // VOXBLOX_UTILS_EVALUATION_UTILS_H_

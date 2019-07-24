#include "voxblox/utils/voxel_utils.h"

#include <cmath>
#include <limits>

#include <glog/logging.h>

#include "voxblox/core/color.h"
#include "voxblox/core/common.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

template <>
void mergeVoxelAIntoVoxelB(const TsdfVoxel& voxel_A, TsdfVoxel* voxel_B) {
  CHECK_NOTNULL(voxel_B);
  float combined_weight = voxel_A.weight + voxel_B->weight;
  // Check for overflow and reset the combined weight in this case.
  if (!std::isfinite(combined_weight)) {
    LOG(WARNING) << "overflow";
    combined_weight = std::numeric_limits<float>::max();
  }
  if (combined_weight > 0) {
    voxel_B->distance = (voxel_A.distance * voxel_A.weight +
                         voxel_B->distance * voxel_B->weight) /
                        combined_weight;

    voxel_B->color = Color::blendTwoColors(voxel_A.color, voxel_A.weight,
                                           voxel_B->color, voxel_B->weight);

    voxel_B->weight = combined_weight;
  } else {
    // If the combined voxel weight is smaller or equal to 0, and one of voxel
    // weights is smaller than 0, this means that we try to substract the two
    // (e.g. an integrated point cloud that we want remove again). In this
    // case, we just reset the voxel.
    *voxel_B = TsdfVoxel();
  }
}

template <>
void mergeVoxelAIntoVoxelB(const EsdfVoxel& voxel_A, EsdfVoxel* voxel_B) {
  if (voxel_A.observed && voxel_B->observed) {
    voxel_B->distance = (voxel_A.distance + voxel_B->distance) / 2.f;
  } else if (voxel_A.observed && !voxel_B->observed) {
    voxel_B->distance = voxel_A.distance;
  }
  voxel_B->observed = voxel_B->observed || voxel_A.observed;
}

template <>
void mergeVoxelAIntoVoxelB(const OccupancyVoxel& voxel_A,
                           OccupancyVoxel* voxel_B) {
  voxel_B->probability_log += voxel_A.probability_log;
  voxel_B->observed = voxel_B->observed || voxel_A.observed;
}

}  // namespace voxblox

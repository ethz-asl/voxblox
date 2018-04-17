#include "voxblox/integrator/merge_integration.h"

namespace voxblox {

template <>
void mergeVoxelAIntoVoxelB(const TsdfVoxel& voxel_A, TsdfVoxel* voxel_B) {
  float combined_weight = voxel_A.weight + voxel_B->weight;
  if (combined_weight > 0) {
    voxel_B->distance = (voxel_A.distance * voxel_A.weight +
                         voxel_B->distance * voxel_B->weight) /
                        combined_weight;

    voxel_B->color = Color::blendTwoColors(voxel_A.color, voxel_A.weight,
                                           voxel_B->color, voxel_B->weight);

    voxel_B->weight = combined_weight;
  }
}

template <>
void mergeVoxelAIntoVoxelB(const OccupancyVoxel& voxel_A,
                           OccupancyVoxel* voxel_B) {
  voxel_B->probability_log += voxel_A.probability_log;
  voxel_B->observed = voxel_B->observed || voxel_A.observed;
}

}  // namespace voxblox

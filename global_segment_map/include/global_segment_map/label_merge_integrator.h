#ifndef GLOBAL_SEGMENT_MAP_LABEL_MERGE_INTEGRATOR_H_
#define GLOBAL_SEGMENT_MAP_LABEL_MERGE_INTEGRATOR_H_

#include <voxblox/integrator/merge_integrator.h>

#include "global_segment_map/label_voxel.h"

namespace voxblox {
template <>
void MergeIntegrator::mergeVoxelAIntoVoxelB(const LabelVoxel& voxel_A,
                                            LabelVoxel* voxel_B) {
  if (voxel_A.label == voxel_B->label) {
    voxel_B->label_confidence += voxel_A.label_confidence;
  } else if (voxel_A.label_confidence > voxel_B->label_confidence) {
    voxel_B->label = voxel_A.label;
    voxel_B->label_confidence =
        voxel_A.label_confidence - voxel_B->label_confidence;
  } else {
    voxel_B->label_confidence -= voxel_A.label_confidence;
  }
}
}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_LABEL_MERGE_INTEGRATOR_H_

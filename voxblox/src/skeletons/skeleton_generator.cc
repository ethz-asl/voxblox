#include "voxblox/skeletons/skeleton_generator.h"

namespace voxblox {

SkeletonGenerator::SkeletonGenerator(Layer<EsdfVoxel>* esdf_layer)
    : esdf_layer_(esdf_layer) {
  CHECK_NOTNULL(esdf_layer);
}

void SkeletonGenerator::generateSkeleton() {}

}  // namespace voxblox

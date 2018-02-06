#ifndef VOXBLOX_SKELETONS_SKELETON_GENERATOR_H_
#define VOXBLOX_SKELETONS_SKELETON_GENERATOR_H_

#include <glog/logging.h>
#include <Eigen/Core>
#include <algorithm>
#include <queue>
#include <utility>
#include <vector>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/skeletons/skeleton.h"
#include "voxblox/utils/timing.h"

namespace voxblox {

class SkeletonGenerator {
  SkeletonGenerator(Layer<EsdfVoxel>* esdf_layer);

  void generateSkeleton();

  // Skeleton access.
  const Skeleton& getSkeleton() const { return skeleton_; }
  Skeleton& getSkeleton() { return skeleton_; }

 private:
  float min_separation_angle_;

  Skeleton skeleton_;

  Layer<EsdfVoxel>* esdf_layer_;
};

}  // namespace voxblox

#endif  // VOXBLOX_SKELETONS_SKELETON_GENERATOR_H_

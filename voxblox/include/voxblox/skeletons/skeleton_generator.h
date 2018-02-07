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
 public:
  SkeletonGenerator(Layer<EsdfVoxel>* esdf_layer);

  void generateSkeleton();

  // Skeleton access.
  const Skeleton& getSkeleton() const { return skeleton_; }
  Skeleton& getSkeleton() { return skeleton_; }

  float getMinSeparationAngle() const { return min_separation_angle_; }
  void setMinSeparationAngle(float min_separation_angle) {
    min_separation_angle_ = min_separation_angle;
  }

  void getNeighborsAndDistances(
      const BlockIndex& block_index, const VoxelIndex& voxel_index,
      AlignedVector<VoxelKey>* neighbors, AlignedVector<float>* distances,
      AlignedVector<Eigen::Vector3i>* directions) const;

  void getNeighbor(const BlockIndex& block_index, const VoxelIndex& voxel_index,
                   const Eigen::Vector3i& direction,
                   BlockIndex* neighbor_block_index,
                   VoxelIndex* neighbor_voxel_index) const;

 private:
  float min_separation_angle_;
  int esdf_voxels_per_side_;

  Skeleton skeleton_;

  Layer<EsdfVoxel>* esdf_layer_;
};

}  // namespace voxblox

#endif  // VOXBLOX_SKELETONS_SKELETON_GENERATOR_H_

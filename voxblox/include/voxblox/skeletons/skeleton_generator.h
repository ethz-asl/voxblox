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

// TODO(helenol): move as an integrator?
class SkeletonGenerator {
 public:
  SkeletonGenerator(Layer<EsdfVoxel>* esdf_layer);

  void generateSkeleton();
  void generateSparseGraph();

  // Skeleton access.
  const Skeleton& getSkeleton() const { return skeleton_; }
  Skeleton& getSkeleton() { return skeleton_; }

  // Sparse graph access.
  const SparseSkeletonGraph& getSparseGraph() const { return graph_; }
  SparseSkeletonGraph& getSparseGraph() { return graph_; }

  float getMinSeparationAngle() const { return min_separation_angle_; }
  void setMinSeparationAngle(float min_separation_angle) {
    min_separation_angle_ = min_separation_angle;
  }

  void getNeighborsAndDistances(
      const BlockIndex& block_index, const VoxelIndex& voxel_index,
      int connectivity, AlignedVector<VoxelKey>* neighbors,
      AlignedVector<float>* distances,
      AlignedVector<Eigen::Vector3i>* directions) const;

  void getNeighbor(const BlockIndex& block_index, const VoxelIndex& voxel_index,
                   const Eigen::Vector3i& direction,
                   BlockIndex* neighbor_block_index,
                   VoxelIndex* neighbor_voxel_index) const;

  // Follow an edge through the layer, aborting when either no more neighbors
  // exist or a vertex is found.
  bool followEdge(const BlockIndex& start_block_index,
                  const VoxelIndex& start_voxel_index,
                  const Eigen::Vector3i& direction_from_vertex,
                  int64_t* connected_vertex_id, float* min_distance,
                  float* max_distance);

 private:
  float min_separation_angle_;
  int esdf_voxels_per_side_;

  Skeleton skeleton_;

  Layer<EsdfVoxel>* esdf_layer_;
  // Owned by the generator! Since it's an intermediate by-product of
  // constructing the graph.
  std::unique_ptr<Layer<SkeletonVoxel>> skeleton_layer_;

  SparseSkeletonGraph graph_;
};

}  // namespace voxblox

#endif  // VOXBLOX_SKELETONS_SKELETON_GENERATOR_H_

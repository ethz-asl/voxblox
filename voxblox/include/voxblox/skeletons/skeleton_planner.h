#ifndef VOXBLOX_SKELETONS_SKELETON_PLANNER_H_
#define VOXBLOX_SKELETONS_SKELETON_PLANNER_H_

#include "voxblox/core/common.h"
#include "voxblox/skeletons/skeleton.h"
#include "voxblox/skeletons/neighbor_tools.h"

namespace voxblox {

// Takes in a skeleton layer. Outputs shortest path within this layer.
// Only uses edges + vertices.
// Uses the layer itself rather than the sparse graph.
class SkeletonAStar {
 public:
  typedef typename AnyIndexHashMapType<FloatingPoint>::type IndexToDistanceMap;
  typedef typename AnyIndexHashMapType<Eigen::Vector3i>::type IndexToParentMap;

  SkeletonAStar() {}
  SkeletonAStar(const Layer<SkeletonVoxel>* skeleton_layer)
      : skeleton_layer_(skeleton_layer) {
    CHECK_NOTNULL(skeleton_layer_);
    neighbor_tools_.setLayer(skeleton_layer_);
  }

  inline void setSkeletonLayer(const Layer<SkeletonVoxel>* skeleton_layer) {
    CHECK_NOTNULL(skeleton_layer);
    skeleton_layer_ = skeleton_layer;
    neighbor_tools_.setLayer(skeleton_layer_);
  }

  // Requires that the start and end points are on the diagram.
  bool getPathOnDiagram(const Point& start_position, const Point& end_position,
                        AlignedVector<Point>* coordinate_path) const;

  bool getPathInVoxels(const BlockIndex& start_block_index,
                       const VoxelIndex& start_voxel_index,
                       const Eigen::Vector3i& goal_offset_voxels,
                       AlignedVector<Eigen::Vector3i>* voxel_path) const;

  Eigen::Vector3i popSmallestFromOpen(const IndexToDistanceMap& f_score_map,
                                      IndexSet* open_set) const;

  void getSolutionPath(const Eigen::Vector3i& end_voxel_offset,
                       const IndexToParentMap& parent_map,
                       AlignedVector<Eigen::Vector3i>* voxel_path) const;

  FloatingPoint estimateCostToGoal(
      const Eigen::Vector3i& voxel_offset,
      const Eigen::Vector3i& goal_voxel_offset) const;

 private:
  NeighborTools<SkeletonVoxel> neighbor_tools_;

  const Layer<SkeletonVoxel>* skeleton_layer_;
};

}  // namespace voxblox

#endif  // VOXBLOX_SKELETONS_SKELETON_PLANNER_H_

#ifndef VOXBLOX_SKELETONS_SKELETON_PLANNER_H_
#define VOXBLOX_SKELETONS_SKELETON_PLANNER_H_

#include "voxblox/core/common.h"
#include "voxblox/skeletons/skeleton.h"
#include "voxblox/skeletons/neighbor_tools.h"

namespace voxblox {

// Takes in a skeleton layer. Outputs shortest path within this layer.
// Only uses edges + vertices.
// Uses the layer itself rather than the sparse graph.
// Also supports searching through the ESDF until the diagram is reached, or
// (for evaluation reasons) just searching through the ESDF.
class SkeletonAStar {
 public:
  typedef typename AnyIndexHashMapType<FloatingPoint>::type IndexToDistanceMap;
  typedef typename AnyIndexHashMapType<Eigen::Vector3i>::type IndexToParentMap;

  SkeletonAStar();
  SkeletonAStar(const Layer<SkeletonVoxel>* skeleton_layer);

  inline void setSkeletonLayer(const Layer<SkeletonVoxel>* skeleton_layer) {
    CHECK_NOTNULL(skeleton_layer);
    skeleton_layer_ = skeleton_layer;
    neighbor_tools_.setLayer(skeleton_layer_);
  }

  inline void setEsdfLayer(const Layer<EsdfVoxel>* esdf_layer) {
    CHECK_NOTNULL(esdf_layer);
    esdf_layer_ = esdf_layer;
    // Just assume that neighbor tools is set to the skeleton layer...
    // As long as the layers are the same size, it doesn't matter.
  }

  // If 0, then unlimited iterations.
  int getMaxIterations() const { return max_iterations_; }
  void setMaxIterations(int max_iterations) {
    max_iterations_ = max_iterations;
  }

  float getMinEsdfDistance() const { return min_esdf_distance_; }
  void setMinEsdfDistance(float min_esdf_distance) {
    min_esdf_distance_ = min_esdf_distance;
  }

  // Search JUST in the ESDF.
  bool getPathInEsdf(const Point& start_position, const Point& end_position,
                     AlignedVector<Point>* coordinate_path) const;

  // First gets the path in the ESDF to the beginning of the diagram, then
  // along the diagram, then at the end to the point on the ESDF.
  bool getPathUsingEsdfAndDiagram(const Point& start_position,
                                  const Point& end_position,
                                  AlignedVector<Point>* coordinate_path) const;

  // Requires that the start and end points are on the diagram.
  bool getPathOnDiagram(const Point& start_position, const Point& end_position,
                        AlignedVector<Point>* coordinate_path) const;

  // Gets the path in the skeleton diagram in terms of voxel offsets.
  template <typename VoxelType>
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

  void voxelPathToCoordinatePath(
      const Point& start_location, FloatingPoint voxel_size,
      const AlignedVector<Eigen::Vector3i>& voxel_path,
      AlignedVector<Point>* coordinate_path) const;

 private:
  bool getPathToNearestDiagramPt(
      const BlockIndex& start_block_index, const VoxelIndex& start_voxel_index,
      const Eigen::Vector3i& goal_voxel_offset,
      AlignedVector<Eigen::Vector3i>* voxel_path) const;

  // Is this voxel a valid neighbor for the A*?
  template <typename VoxelType>
  bool isValidVoxel(const BlockIndex& block_index,
                    const VoxelIndex& voxel_index,
                    const BlockIndex& block_ptr_index,
                    typename Block<VoxelType>::ConstPtr block_ptr) const;

  template <typename VoxelType>
  typename Block<VoxelType>::ConstPtr getBlockPtrByIndex(
      const BlockIndex& block_index) const;

  // Calculate what the block and voxel indices should be based on different
  // layer types.
  template <typename VoxelType>
  Point getBlockAndVoxelIndex(const Point& coord, BlockIndex* block_index,
                              VoxelIndex* voxel_index) const;

  // Max number of voxel evaluations the algorithm is allowed to do. If 0
  // (default), this is limitless.
  int max_iterations_;
  // The robot radius or minimum distance to consider in the ESDF.
  // For the skeleton/graph, assumes that the GVD was already built with
  // this minimum distance.
  float min_esdf_distance_;

  NeighborTools<SkeletonVoxel> neighbor_tools_;

  const Layer<SkeletonVoxel>* skeleton_layer_;
  const Layer<EsdfVoxel>* esdf_layer_;
};

template <typename VoxelType>
bool SkeletonAStar::getPathInVoxels(
    const BlockIndex& start_block_index, const VoxelIndex& start_voxel_index,
    const Eigen::Vector3i& goal_voxel_offset,
    AlignedVector<Eigen::Vector3i>* voxel_path) const {
  CHECK_NOTNULL(voxel_path);
  CHECK_NOTNULL(skeleton_layer_);

  int num_iterations = 0;

  // Make the 3 maps we need.
  IndexToDistanceMap f_score_map;
  IndexToDistanceMap g_score_map;
  IndexToParentMap parent_map;

  // Make the 2 sets we need.
  IndexSet open_set;
  IndexSet closed_set;

  Eigen::Vector3i current_voxel_offset = Eigen::Vector3i::Zero();

  // Set up storage for voxels and blocks.
  BlockIndex block_index = start_block_index;
  VoxelIndex voxel_index = start_voxel_index;
  typename Block<VoxelType>::ConstPtr block_ptr =
      getBlockPtrByIndex<VoxelType>(block_index);

  f_score_map[current_voxel_offset] =
      estimateCostToGoal(current_voxel_offset, goal_voxel_offset);
  g_score_map[current_voxel_offset] = 0.0;

  open_set.insert(current_voxel_offset);

  while (!open_set.empty()) {
    num_iterations++;
    if (max_iterations_ > 0 && num_iterations > max_iterations_) {
      break;
    }
    // Find the smallest f-value in the open set.
    current_voxel_offset = popSmallestFromOpen(f_score_map, &open_set);

    // Check if this is already the goal...
    if (current_voxel_offset == goal_voxel_offset) {
      getSolutionPath(goal_voxel_offset, parent_map, voxel_path);
      return true;
    }

    closed_set.insert(current_voxel_offset);

    // Get the block and voxel index of this guy.
    neighbor_tools_.getNeighbor(start_block_index, start_voxel_index,
                                current_voxel_offset, &block_index,
                                &voxel_index);
    block_ptr = getBlockPtrByIndex<VoxelType>(block_index);
    AlignedVector<VoxelKey> neighbors;
    AlignedVector<float> distances;
    AlignedVector<Eigen::Vector3i> directions;
    neighbor_tools_.getNeighborsAndDistances(
        block_index, voxel_index, Connectivity::kTwentySix, &neighbors,
        &distances, &directions);
    for (size_t i = 0; i < neighbors.size(); ++i) {
      BlockIndex neighbor_block_index = neighbors[i].first;
      VoxelIndex neighbor_voxel_index = neighbors[i].second;

      if (!isValidVoxel<VoxelType>(neighbor_block_index, neighbor_voxel_index,
                                   block_index, block_ptr)) {
        continue;
      }
      Eigen::Vector3i neighbor_voxel_offset =
          current_voxel_offset + directions[i];
      if (closed_set.count(neighbor_voxel_offset) > 0) {
        // Already checked this guy as well.
        continue;
      }
      if (open_set.count(neighbor_voxel_offset) == 0) {
        open_set.insert(neighbor_voxel_offset);
      }

      FloatingPoint tentative_g_score =
          g_score_map[current_voxel_offset] + distances[i];
      if (g_score_map.count(neighbor_voxel_offset) == 0 ||
          g_score_map[neighbor_voxel_offset] < tentative_g_score) {
        g_score_map[neighbor_voxel_offset] = tentative_g_score;
        f_score_map[neighbor_voxel_offset] =
            tentative_g_score +
            estimateCostToGoal(neighbor_voxel_offset, goal_voxel_offset);
        parent_map[neighbor_voxel_offset] = current_voxel_offset;
      }
    }
  }
  return false;
}

}  // namespace voxblox

#endif  // VOXBLOX_SKELETONS_SKELETON_PLANNER_H_

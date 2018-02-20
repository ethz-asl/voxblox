#include "voxblox/skeletons/skeleton_planner.h"

namespace voxblox {

bool SkeletonAStar::getPathOnDiagram(
    const Point& start_position, const Point& end_position,
    AlignedVector<Point>* coordinate_path) const {
  CHECK_NOTNULL(coordinate_path);
  CHECK_NOTNULL(skeleton_layer_);

  // Look up where in the skeleton diagram the start position is.

  // Get the distance to the goal position.

  // Now get the path in voxels back out.
}

bool SkeletonAStar::getPathInVoxels(
    const BlockIndex& start_block_index, const VoxelIndex& start_voxel_index,
    const Eigen::Vector3i& goal_voxel_offset,
    AlignedVector<Eigen::Vector3i>* voxel_path) const {
  CHECK_NOTNULL(voxel_path);
  CHECK_NOTNULL(skeleton_layer_);

  // Make the 3 maps we need.
  IndexToDistanceMap f_score_map;
  IndexToDistanceMap g_score_map;
  IndexToParentMap parent_map;

  // Make the 2 sets we need.
  IndexSet open_set;  // This should be a priority queue... But then harder
  // to check for belonging. Ehhh. Just sort it each time.
  IndexSet closed_set;

  Eigen::Vector3i current_voxel_offset = Eigen::Vector3i::Zero();

  // Set up storage for voxels and blocks.
  BlockIndex block_index = start_block_index;
  VoxelIndex voxel_index = start_voxel_index;
  Block<SkeletonVoxel>::ConstPtr block_ptr =
      skeleton_layer_->getBlockPtrByIndex(block_index);

  f_score_map[current_voxel_offset] =
      estimateCostToGoal(current_voxel_offset, goal_voxel_offset);
  g_score_map[current_voxel_offset] = 0.0;

  open_set.insert(current_voxel_offset);

  // TODO(helenol): also set max number of iterations?
  while (!open_set.empty()) {
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
    block_ptr = skeleton_layer_->getBlockPtrByIndex(block_index);
    AlignedVector<VoxelKey> neighbors;
    AlignedVector<float> distances;
    AlignedVector<Eigen::Vector3i> directions;
    neighbor_tools_.getNeighborsAndDistances(
        block_index, voxel_index, 26, &neighbors, &distances, &directions);
    for (size_t i = 0; i < neighbors.size(); ++i) {
      BlockIndex neighbor_block_index = neighbors[i].first;
      VoxelIndex neighbor_voxel_index = neighbors[i].second;
      Block<SkeletonVoxel>::ConstPtr neighbor_block;
      if (neighbor_block_index == block_index) {
        neighbor_block = block_ptr;
      } else {
        neighbor_block =
            skeleton_layer_->getBlockPtrByIndex(neighbor_block_index);
      }
      if (!neighbor_block) {
        continue;
      }

      const SkeletonVoxel& neighbor_voxel =
          neighbor_block->getVoxelByVoxelIndex(neighbor_voxel_index);
      if (!neighbor_voxel.is_edge) {
        // Not an edge, can just skip this.
        continue;
      }
      Eigen::Vector3i neighbor_voxel_offset =
          current_voxel_offset - directions[i];
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

Eigen::Vector3i SkeletonAStar::popSmallestFromOpen(
    const IndexToDistanceMap& f_score_map, IndexSet* open_set) const {
  FloatingPoint min_distance = std::numeric_limits<FloatingPoint>::max();
  IndexSet::const_iterator min_iter = open_set->cbegin();

  for (IndexSet::const_iterator iter = open_set->cbegin();
       iter != open_set->cend(); ++iter) {
    FloatingPoint distance = f_score_map.at(*iter);
    if (distance < min_distance) {
      min_distance = distance;
      min_iter = iter;
    }
  }

  Eigen::Vector3i return_val = *min_iter;
  open_set->erase(min_iter);
  return return_val;
}

void SkeletonAStar::getSolutionPath(
    const Eigen::Vector3i& end_voxel_offset, const IndexToParentMap& parent_map,
    AlignedVector<Eigen::Vector3i>* voxel_path) const {
  CHECK_NOTNULL(voxel_path);
  voxel_path->clear();
  voxel_path->push_back(end_voxel_offset);
  Eigen::Vector3i voxel_offset = end_voxel_offset;
  while (parent_map.count(voxel_offset) > 0) {
    voxel_offset = parent_map.at(voxel_offset);
    voxel_path->push_back(voxel_offset);
  }

  std::reverse(voxel_path->begin(), voxel_path->end());
}

FloatingPoint SkeletonAStar::estimateCostToGoal(
    const Eigen::Vector3i& voxel_offset,
    const Eigen::Vector3i& goal_voxel_offset) const {
  return (goal_voxel_offset.cast<FloatingPoint>() -
          voxel_offset.cast<FloatingPoint>())
      .norm();
}

}  // namespace voxblox

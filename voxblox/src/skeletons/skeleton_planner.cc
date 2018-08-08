#include "voxblox/skeletons/skeleton_planner.h"

namespace voxblox {

SkeletonAStar::SkeletonAStar() : max_iterations_(0) {}

SkeletonAStar::SkeletonAStar(const Layer<SkeletonVoxel>* skeleton_layer)
    : max_iterations_(0), skeleton_layer_(skeleton_layer) {
  CHECK_NOTNULL(skeleton_layer_);
  neighbor_tools_.setLayer(skeleton_layer_);
}

template <>
Point SkeletonAStar::getBlockAndVoxelIndex<SkeletonVoxel>(
    const Point& coord, BlockIndex* block_index,
    VoxelIndex* voxel_index) const {
  *block_index = skeleton_layer_->computeBlockIndexFromCoordinates(coord);
  Block<SkeletonVoxel>::ConstPtr block_ptr;
  block_ptr = skeleton_layer_->getBlockPtrByIndex(*block_index);
  CHECK(block_ptr);
  *voxel_index = block_ptr->computeVoxelIndexFromCoordinates(coord);

  Point position_center =
      block_ptr->computeCoordinatesFromVoxelIndex(*voxel_index);
  return position_center;
}

template <>
Point SkeletonAStar::getBlockAndVoxelIndex<EsdfVoxel>(
    const Point& coord, BlockIndex* block_index,
    VoxelIndex* voxel_index) const {
  *block_index = esdf_layer_->computeBlockIndexFromCoordinates(coord);
  Block<EsdfVoxel>::ConstPtr block_ptr;
  block_ptr = esdf_layer_->getBlockPtrByIndex(*block_index);
  CHECK(block_ptr);
  *voxel_index = block_ptr->computeVoxelIndexFromCoordinates(coord);

  Point position_center =
      block_ptr->computeCoordinatesFromVoxelIndex(*voxel_index);
  return position_center;
}

template <>
inline Block<SkeletonVoxel>::ConstPtr SkeletonAStar::getBlockPtrByIndex<
    SkeletonVoxel>(const BlockIndex& block_index) const {
  return skeleton_layer_->getBlockPtrByIndex(block_index);
}

template <>
inline Block<EsdfVoxel>::ConstPtr SkeletonAStar::getBlockPtrByIndex<EsdfVoxel>(
    const BlockIndex& block_index) const {
  return esdf_layer_->getBlockPtrByIndex(block_index);
}

template <>
bool SkeletonAStar::isValidVoxel<SkeletonVoxel>(
    const BlockIndex& block_index, const VoxelIndex& voxel_index,
    const BlockIndex& block_ptr_index,
    Block<SkeletonVoxel>::ConstPtr block_ptr) const {
  Block<SkeletonVoxel>::ConstPtr neighbor_block;
  if (block_index == block_ptr_index) {
    neighbor_block = block_ptr;
  } else {
    neighbor_block = skeleton_layer_->getBlockPtrByIndex(block_index);
  }
  if (!neighbor_block) {
    return false;
  }

  const SkeletonVoxel& neighbor_voxel =
      neighbor_block->getVoxelByVoxelIndex(voxel_index);
  if (!neighbor_voxel.is_edge && !neighbor_voxel.is_vertex) {
    // Not an edge, can just skip this.
    return false;
  }
  return true;
}

template <>
bool SkeletonAStar::isValidVoxel<EsdfVoxel>(
    const BlockIndex& block_index, const VoxelIndex& voxel_index,
    const BlockIndex& block_ptr_index,
    Block<EsdfVoxel>::ConstPtr block_ptr) const {
  Block<EsdfVoxel>::ConstPtr neighbor_block;
  if (block_index == block_ptr_index) {
    neighbor_block = block_ptr;
  } else {
    neighbor_block = esdf_layer_->getBlockPtrByIndex(block_index);
  }
  if (!neighbor_block) {
    return false;
  }

  const EsdfVoxel& neighbor_voxel =
      neighbor_block->getVoxelByVoxelIndex(voxel_index);
  if (!neighbor_voxel.observed ||
      neighbor_voxel.distance < min_esdf_distance_) {
    // Then it's not a valid voxel!
    return false;
  }
  return true;
}

bool SkeletonAStar::getPathInEsdf(const Point& start_position,
                                  const Point& end_position,
                                  AlignedVector<Point>* coordinate_path) const {
  CHECK_NOTNULL(coordinate_path);
  CHECK_NOTNULL(esdf_layer_);

  // Look up where in the skeleton diagram the start position is.
  // Get the voxel.
  BlockIndex start_block_index, end_block_index;
  VoxelIndex start_voxel_index, end_voxel_index;

  Point start_position_center = getBlockAndVoxelIndex<EsdfVoxel>(
      start_position, &start_block_index, &start_voxel_index);
  getBlockAndVoxelIndex<EsdfVoxel>(end_position, &end_block_index,
                                   &end_voxel_index);

  // Get the distance to the goal index.
  Eigen::Vector3i goal_voxel_offset = neighbor_tools_.getOffsetBetweenVoxels(
      start_block_index, start_voxel_index, end_block_index, end_voxel_index);

  // Now get the path in voxels back out.
  AlignedVector<Eigen::Vector3i> voxel_path;
  bool success = getPathInVoxels<EsdfVoxel>(
      start_block_index, start_voxel_index, goal_voxel_offset, &voxel_path);

  if (!success) {
    return false;
  }

  // Ok it worked, so convert the voxel offset path to a coordinate path.
  // Get the voxel center start position back out.
  voxelPathToCoordinatePath(start_position_center, esdf_layer_->voxel_size(),
                            voxel_path, coordinate_path);
  return success;
}

bool SkeletonAStar::getPathOnDiagram(
    const Point& start_position, const Point& end_position,
    AlignedVector<Point>* coordinate_path) const {
  CHECK_NOTNULL(coordinate_path);
  CHECK_NOTNULL(skeleton_layer_);

  // Look up where in the skeleton diagram the start position is.
  // Get the voxel.
  BlockIndex start_block_index, end_block_index;
  VoxelIndex start_voxel_index, end_voxel_index;

  Point start_position_center = getBlockAndVoxelIndex<SkeletonVoxel>(
      start_position, &start_block_index, &start_voxel_index);
  getBlockAndVoxelIndex<SkeletonVoxel>(end_position, &end_block_index,
                                       &end_voxel_index);

  // Get the distance to the goal index.
  Eigen::Vector3i goal_voxel_offset = neighbor_tools_.getOffsetBetweenVoxels(
      start_block_index, start_voxel_index, end_block_index, end_voxel_index);

  // Now get the path in voxels back out.
  AlignedVector<Eigen::Vector3i> voxel_path;
  bool success = getPathInVoxels<SkeletonVoxel>(
      start_block_index, start_voxel_index, goal_voxel_offset, &voxel_path);

  if (!success) {
    return false;
  }

  // Ok it worked, so convert the voxel offset path to a coordinate path.
  // Get the voxel center start position back out.
  voxelPathToCoordinatePath(start_position_center,
                            skeleton_layer_->voxel_size(), voxel_path,
                            coordinate_path);
  return success;
}

bool SkeletonAStar::getPathUsingEsdfAndDiagram(
    const Point& start_position, const Point& end_position,
    AlignedVector<Point>* coordinate_path) const {
  // First, we need to find the start and end points on the diagram.
  CHECK_NOTNULL(coordinate_path);
  CHECK_NOTNULL(skeleton_layer_);
  CHECK_NOTNULL(esdf_layer_);

  // Look up where in the skeleton diagram the start position is.
  // Get the voxel.
  BlockIndex start_block_index, end_block_index;
  VoxelIndex start_voxel_index, end_voxel_index;

  Point start_position_center = getBlockAndVoxelIndex<EsdfVoxel>(
      start_position, &start_block_index, &start_voxel_index);
  Point end_position_center = getBlockAndVoxelIndex<EsdfVoxel>(
      end_position, &end_block_index, &end_voxel_index);

  // Get the distance to the goal index.
  Eigen::Vector3i goal_voxel_offset = neighbor_tools_.getOffsetBetweenVoxels(
      start_block_index, start_voxel_index, end_block_index, end_voxel_index);

  // First the diagram start and end points.
  // For the diagram start, search toward the goal until you hit the diagram.
  AlignedVector<Eigen::Vector3i> voxel_path_to_start, voxel_path_from_end,
      voxel_path_on_diagram;
  if (!getPathToNearestDiagramPt(start_block_index, start_voxel_index,
                                 goal_voxel_offset, &voxel_path_to_start)) {
    return false;
  }
  // Now figure out where the end is.
  if (!getPathToNearestDiagramPt(end_block_index, end_voxel_index,
                                 -goal_voxel_offset, &voxel_path_from_end)) {
    return false;
  }

  // Figure out the start and end voxel positions on the diagram.
  BlockIndex diagram_start_block_index, diagram_end_block_index;
  VoxelIndex diagram_start_voxel_index, diagram_end_voxel_index;
  // Get the block and voxel index of this guy.
  neighbor_tools_.getNeighbor(
      start_block_index, start_voxel_index, voxel_path_to_start.back(),
      &diagram_start_block_index, &diagram_start_voxel_index);
  neighbor_tools_.getNeighbor(
      end_block_index, end_voxel_index, voxel_path_from_end.back(),
      &diagram_end_block_index, &diagram_end_voxel_index);

  goal_voxel_offset = neighbor_tools_.getOffsetBetweenVoxels(
      diagram_start_block_index, diagram_start_voxel_index,
      diagram_end_block_index, diagram_end_voxel_index);

  if (!getPathInVoxels<SkeletonVoxel>(
          diagram_start_block_index, diagram_start_voxel_index,
          goal_voxel_offset, &voxel_path_on_diagram)) {
    return false;
  }

  AlignedVector<Point> coordinate_path_start, coordinate_path_diagram,
      coordinate_path_end;

  // Get the coordinates back out, one at a time.
  voxelPathToCoordinatePath(start_position_center,
                            skeleton_layer_->voxel_size(), voxel_path_to_start,
                            &coordinate_path_start);
  voxelPathToCoordinatePath(coordinate_path_start.back(),
                            skeleton_layer_->voxel_size(),
                            voxel_path_on_diagram, &coordinate_path_diagram);
  voxelPathToCoordinatePath(end_position_center, skeleton_layer_->voxel_size(),
                            voxel_path_from_end, &coordinate_path_end);
  // Reverse the last one...
  std::reverse(coordinate_path_end.begin(), coordinate_path_end.end());

  // Now concatenate them together.
  coordinate_path->insert(coordinate_path->end(), coordinate_path_start.begin(),
                          coordinate_path_start.end());
  coordinate_path->insert(coordinate_path->end(),
                          coordinate_path_diagram.begin(),
                          coordinate_path_diagram.end());
  coordinate_path->insert(coordinate_path->end(), coordinate_path_end.begin(),
                          coordinate_path_end.end());
  return true;
}

void SkeletonAStar::voxelPathToCoordinatePath(
    const Point& start_location, FloatingPoint voxel_size,
    const AlignedVector<Eigen::Vector3i>& voxel_path,
    AlignedVector<Point>* coordinate_path) const {
  coordinate_path->clear();
  coordinate_path->reserve(voxel_path.size());

  for (const Eigen::Vector3i& voxel_offset : voxel_path) {
    coordinate_path->push_back(start_location +
                               voxel_offset.cast<FloatingPoint>() * voxel_size);
  }
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

// TODO(helenol): figure out how to do this without SOOOOO much copy and paste.
bool SkeletonAStar::getPathToNearestDiagramPt(
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
  IndexSet open_set;  // This should be a priority queue... But then harder
  // to check for belonging. Ehhh. Just sort it each time.
  IndexSet closed_set;

  Eigen::Vector3i current_voxel_offset = Eigen::Vector3i::Zero();

  // Set up storage for voxels and blocks.
  BlockIndex block_index = start_block_index;
  VoxelIndex voxel_index = start_voxel_index;
  Block<EsdfVoxel>::ConstPtr block_ptr =
      getBlockPtrByIndex<EsdfVoxel>(block_index);

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
    block_ptr = getBlockPtrByIndex<EsdfVoxel>(block_index);

    // Figure out if this is on the diagram.
    // TODO(helenol): this is the only new part...
    Block<SkeletonVoxel>::ConstPtr skeleton_block_ptr =
        getBlockPtrByIndex<SkeletonVoxel>(block_index);
    if (isValidVoxel<SkeletonVoxel>(block_index, voxel_index, block_index,
                                    skeleton_block_ptr)) {
      // Consider this done!
      getSolutionPath(current_voxel_offset, parent_map, voxel_path);
      return true;
    }

    AlignedVector<VoxelKey> neighbors;
    AlignedVector<float> distances;
    AlignedVector<Eigen::Vector3i> directions;
    neighbor_tools_.getNeighborsAndDistances(
        block_index, voxel_index, Connectivity::kTwentySix, &neighbors,
        &distances, &directions);
    for (size_t i = 0; i < neighbors.size(); ++i) {
      BlockIndex neighbor_block_index = neighbors[i].first;
      VoxelIndex neighbor_voxel_index = neighbors[i].second;

      if (!isValidVoxel<EsdfVoxel>(neighbor_block_index, neighbor_voxel_index,
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

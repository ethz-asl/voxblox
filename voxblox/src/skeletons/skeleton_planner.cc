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

}  // namespace voxblox

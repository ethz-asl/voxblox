#include "nbvp_voxblox/nbvp_exploration_planner.h"

namespace nbvp_voxblox {

NbvpExplorationPlanner::NbvpExplorationPlanner(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      rrt_iterations_(15),
      robot_radius_(0.6) {}

void NbvpExplorationPlanner::setup() {
  rrt_system_.setIsInFreeSpaceFunction(std::bind(
      &NbvpExplorationPlanner::isInFreeSpace, this, std::placeholders::_1));
  rrt_planner_.setSystem(rrt_system_);
}

bool NbvpExplorationPlanner::getNextWaypoint(
    const mav_msgs::EigenTrajectoryPoint& current_state,
    mav_msgs::EigenTrajectoryPoint* waypoint) {
  rrt_system_.setRootState(current_state.position_W, current_state.getYaw());
  rrt_planner_.initialize();
  rrt_planner_.setGamma(0.5);

  for (int i = 0; i < rrt_iterations_; ++i) {
    rrt_planner_.iteration();
  }

  // Get the solution out somehow? Go over all the vertices in the graph
  // and find the best branch of the tree based on cost.
  int num_vertices = rrt_planner_.numVertices;

  // Somehow ended up with no vertices, everything is probably terrible and we
  // should just give up.
  if (num_vertices <= 0) {
    return false;
  }

  double best_gain = 0.0;
  VertexType* best_vertex;

  // Otherwise iterate over all vertices and find the best branch.
  for (std::list<VertexType*>::iterator iter =
           rrt_planner_.listVertices.begin();
       iter != rrt_planner_.listVertices.end(); iter++) {
    VertexType& current_vertex = **iter;
    State& current_state = current_vertex.getState();
    double current_gain = current_state.getGain();

    // Iterate over the parents of each vertex and accumulate the gains.
    VertexType* parent_vertex = current_vertex.getParentPtr();
    while (parent_vertex != NULL) {
      current_gain += parent_vertex->getState().getGain();
      parent_vertex = parent_vertex->getParentPtr();
    }

    std::cout << "Vertex: pos: " << current_state.getPosition().transpose()
              << " yaw: " << current_state.getYaw()
              << " gain: " << current_state.getGain()
              << " total branch gain: " << current_gain << std::endl;

    if (current_gain > best_gain) {
      best_vertex = &current_vertex;
    }
  }

  // Get the first waypoint in this vertex chain. Basically trace back until
  // the one before the first (hopefully the very first vertex in the chain is
  // the start vertex).
  VertexType* first_best_vertex;
  VertexType* best_vertex_parent = best_vertex->getParentPtr();
  // This makes 0 sense since then the best vertex is actually just the first...
  // But I guess no other choice in this case!
  if (best_vertex_parent == NULL) {
    first_best_vertex = best_vertex;
    // Otherwise find the vertex who's parents parent is null (2nd in the
    // chain).
  } else if (best_vertex_parent->getParentPtr() == NULL) {
    first_best_vertex = best_vertex;
  } else {
    while (best_vertex_parent->getParentPtr() != NULL) {
      if (best_vertex_parent->getParentPtr()->getParentPtr() == NULL) {
        first_best_vertex = best_vertex_parent;
        break;
      }
      best_vertex_parent = best_vertex_parent->getParentPtr();
    }
  }

  return true;
}

bool NbvpExplorationPlanner::isInFreeSpace(
    const Eigen::Vector3d& robot_position) const {
  const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_layer =
      tsdf_map_->getTsdfLayer();
  voxblox::Point robot_point = robot_position.cast<voxblox::FloatingPoint>();

  BlockVoxelListMap block_voxel_list;
  getSphereAroundPoint(robot_point, robot_radius_, &block_voxel_list);

  for (const std::pair<voxblox::BlockIndex, voxblox::VoxelIndexList>& kv :
       block_voxel_list) {
    // Get block -- only already existing blocks are in the list.
    voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
        tsdf_layer_->getBlockPtrByIndex(kv.first);

    // Block that should exist doesn't exist...
    if (!block_ptr) {
      return false;
    }

    for (const voxblox::VoxelIndex& voxel_index : kv.second) {
      if (!block_ptr->isValidVoxelIndex(voxel_index)) {
        continue;
      }
      voxblox::TsdfVoxel& tsdf_voxel =
          block_ptr->getVoxelByVoxelIndex(voxel_index);
      if (tsdf_voxel.weight < voxblox::kEpsilon) {
        return false;
      }
      if (tsdf_voxel.distance <= 0.0f) {
        return false;
      }
    }
  }

  // No collision if nothing in the sphere had a negative or 0 distance.
  // Unknown space is occupied.
  return true;
}

void NbvpExplorationPlanner::getSphereAroundPoint(
    const voxblox::Point& center, voxblox::FloatingPoint radius,
    BlockVoxelListMap* block_voxel_list) const {
  // search a cube with side length 2*radius
  for (voxblox::FloatingPoint x = -radius; x <= radius; x += tsdf_voxel_size_) {
    for (voxblox::FloatingPoint y = -radius; y <= radius;
         y += tsdf_voxel_size_) {
      for (voxblox::FloatingPoint z = -radius; z <= radius;
           z += tsdf_voxel_size_) {
        voxblox::Point point(x, y, z);

        // check if point is inside the spheres radius
        if (point.squaredNorm() <= radius * radius) {
          // convert to global coordinate
          point += center;

          voxblox::BlockIndex block_index =
              tsdf_layer_->computeBlockIndexFromCoordinates(point);

          const voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
              tsdf_layer_->getBlockPtrByIndex(block_index);
          if (block_ptr) {
            (*block_voxel_list)[block_index].push_back(
                block_ptr->computeVoxelIndexFromCoordinates(point));
          } else {
            (*block_voxel_list)[block_index].clear();
          }
        }
      }
    }
  }
}

}  // namespace nbvp_voxblox

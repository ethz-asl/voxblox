#ifndef NBVP_VOXBLOX_NBVP_EXPLORATION_PLANNER_H_
#define NBVP_VOXBLOX_NBVP_EXPLORATION_PLANNER_H_

#include <mav_trajectory_generation/timing.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/trajectory_sampling.h>
#include <planner_base/common/timing.h>
#include <planner_base/planner_base.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox/core/tsdf_map.h>

#include "nbvp_voxblox/rrt/nbvp_system.h"
#include "nbvp_voxblox/rrt/rrts.h"

namespace nbvp_voxblox {

class NbvpExplorationPlanner {
 public:
  typedef voxblox::BlockHashMapType<voxblox::VoxelIndexList>::type
      BlockVoxelListMap;

  NbvpExplorationPlanner(const ros::NodeHandle& nh,
                         const ros::NodeHandle& nh_private);

  void setTsdfMap(const std::shared_ptr<voxblox::TsdfMap>& tsdf_map) {
    tsdf_map_ = tsdf_map;
    tsdf_layer_ = tsdf_map_->getTsdfLayerPtr();
    tsdf_voxel_size_ = tsdf_layer_->voxel_size();
  }

  void setRobotRadius(double radius) { robot_radius_ = radius; }
  double getRobotRadius() const { return robot_radius_; }

  void setBounds(const Eigen::Vector3d& lower_bound,
                 const Eigen::Vector3d& upper_bound) {
    rrt_system_.setBounds(lower_bound, upper_bound);
  }

  // MUST be called before the first instance of getNextWaypoint.
  // Sets up the underlying RRT problem.
  void setup();

  bool getNextWaypoint(const mav_msgs::EigenTrajectoryPoint& current_state,
                       mav_msgs::EigenTrajectoryPoint* waypoint);

  bool getNextTrajectory(const mav_msgs::EigenTrajectoryPoint& current_state,
                         mav_trajectory_generation::Trajectory* trajectory);

  bool getNextPath(const mav_msgs::EigenTrajectoryPoint& current_state,
                   mav_msgs::EigenTrajectoryPointVector* path);

  bool isInFreeSpace(const Eigen::Vector3d& position) const;
  bool evaluateExplorationGain(const Eigen::Vector3d& position,
                               const Eigen::Quaterniond& orientation) const;

 private:
  void getSphereAroundPoint(const voxblox::Point& center,
                            voxblox::FloatingPoint radius,
                            BlockVoxelListMap* block_voxel_list) const;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Settings
  int rrt_iterations_;
  double robot_radius_;
  double tsdf_voxel_size_;

  // The TSDF layer that we get collision information from.
  std::shared_ptr<voxblox::TsdfMap> tsdf_map_;
  voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer_;

  // RRT planner.
  RRTstar::Planner<State, Trajectory, System> rrt_planner_;
  System rrt_system_;

  typedef RRTstar::Vertex<State, Trajectory, System> VertexType;
};

}  // namespace nbvp_voxblox

#endif  // NBVP_VOXBLOX_NBVP_EXPLORATION_PLANNER_H_

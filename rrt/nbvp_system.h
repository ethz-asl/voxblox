#ifndef NBVP_VOXBLOX_RRT_NBVP_SYSTEM_H_
#define NBVP_VOXBLOX_RRT_NBVP_SYSTEM_H_

#include <list>
#include <vector>
#include <Eigen/Core>

namespace nbvp_voxblox {

// Individual RRT states, in this case 4D (xyz and yaw).
class State {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  State()
      : position_(Eigen::Vector3d::Zero()), yaw_(0), gain_computed_(false) {}
  State(const Eigen::Vector3d& position, double yaw)
      : position_(position), yaw_(yaw), gain_computed_(false) {}

  State& operator=(const State& state_in) {
    position_ = state_in.getPosition();
    yaw_ = state_in.getYaw();
    gain_computed_ = state_in.gainComputed();
    exploration_gain_ = state_in.getGainIfComputed();
  }

  Eigen::Vector3d getPosition() const { return position_; }
  void setPosition(const Eigen::Vector3d& position) { position_ = position; }
  double getYaw() const { return yaw_; }
  void setYaw(double yaw) { yaw_ = yaw; }
  bool gainComputed() const { return gain_computed_; }
  double getGainIfComputed() const { return exploration_gain_; }
  double computeGain() {
    if (gain_computed_) {
      return exploration_gain_;
    }
    // TODO: compute gain!
    return 0.0;
  }

  double& operator[](const int i) {
    if (i < 3) {
      return position_(i);
    } else {
      return yaw_;
    }
  }

 private:
  Eigen::Vector3d position_;
  double yaw_;  // In radians, wrapped to +/- pi.

  // Cache computing the gain, since this is expensive.
  bool gain_computed_;
  double exploration_gain_;
};

// Trajectory between points.
class Trajectory {
 public:
  Trajectory& operator=(const Trajectory& trajectory_in);

  State& getEndState() { return states_.back(); }

  const State& getEndState() const { return states_.back(); }

  void addState(const State& state) { states_.push_back(state); }

  // Returns the cost of this trajectory. Just path length cost.
  double evaluateCost() {
    double distance = 0.0;
    for (size_t i = 0; i < states_.size() - 1; ++i) {
      distance +=
          (states_[i + 1].getPosition() - states_[i].getPosition()).norm();
    }
  }

  // Returns the exploration gain of the trajectory (a combination of all
  // gains of all states). The states cache their own exploration gain, so
  // hopefully no unnecessary computations are performed here.
  double evaluateExplorationGain();

 private:
  std::vector<State> states_;
};

class System {
 public:
  typedef std::function<bool(const Eigen::VectorXd& position)>
      IsInFreeSpaceFunctionType;

  // Returns the dimensionality of the Euclidean space.
  int getNumDimensions() { return 4; }

  //  Returns a reference to the root state.
  State& getRootState() { return root_state_; }

  void setRootState(const Eigen::Vector3d& position, double yaw) {
    // TODO(helenol): clear???
    root_state_ = State(position, yaw);
  }

  // Returns the state_key for the given state.
  // state_in the given state
  // state_key the key to the state. An array of dimension
  // getNumDimensions()
  int getStateKey(State& state_in, double* state_key);

  // Returns true of the given state reaches the target.
  bool isReachingTarget(State& state_in) { return false; }

  // Returns a sample state.
  int sampleState(State& randomStateOut);

  // Returns a the cost of the trajectory that connects state_from_in and
  //       state_towards_in. The trajectory is also returned in
  //       trajectory_out.
  // state_from_in Initial state
  // state_towards_in Final state
  // trajectory_out Trajectory that starts the from the initial state and
  //                      reaches near the final state.
  // exact_connection_out Set to true if the initial and the final states
  //                           can be connected exactly.
  int extendTo(State& state_from_in, State& state_towards_in,
               Trajectory& trajectory_out, bool& exact_connection_out);

  // Returns the cost of the trajectory that connects state_from_in and
  // state_towards_in.
  // state_from_in Initial state
  // state_towards_in Final state
  // exact_connection_out Set to true if the initial and the final states
  //                           can be connected exactly.
  double evaluateExtensionCost(State& state_from_in, State& state_towards_in,
                               bool& exact_connection_out);

  // Returns a lower bound on the cost to go starting from state_in
  // state_in Starting state.
  // In NBVP always 0 so that we approximate a normal RRT...
  double evaluateCostToGo(State& state_in) { return 0.0; }

  // Returns the trajectory as a list of double arrays, each with
  // dimension getNumDimensions.
  // state_from_in Initial state
  // state_to_in Final state
  // trajectory_out The list of double arrays that represent the
  // trajectory
  int getTrajectory(State& state_from_in, State& state_to_in,
                    std::list<double*>& trajectory_out);

  // Sets the sampling bounds of the space.
  void setBounds(const Eigen::Vector3d& lower_bound,
                 const Eigen::Vector3d& upper_bound) {
    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;
  }

  void setMaxEdgeLength(double max_edge_length) {
    max_edge_length_ = max_edge_length;
  }

  void setIsInFreeSpaceFunction(
      const IsInFreeSpaceFunctionType& is_in_free_space) {
    is_in_free_space_func_ = is_in_free_space;
  }

  bool isInFreeSpace(const Eigen::Vector3d& position) {
    if (!is_in_free_space_func_) {
      return true;
    }
    return is_in_free_space_func_(position);
  }

 private:
  State root_state_;

  Eigen::Vector3d lower_bound_;
  Eigen::Vector3d upper_bound_;
  double max_edge_length_;

  IsInFreeSpaceFunctionType is_in_free_space_func_;
};

}  // namespace nvbp_voxblox

#endif

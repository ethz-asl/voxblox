#ifndef NBVP_VOXBLOX_RRT_NBVP_SYSTEM_H_
#define NBVP_VOXBLOX_RRT_NBVP_SYSTEM_H_

#include <list>
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

  Eigen::Vector3d getPosition() const { return postition_; }
  double getYaw() const { return yaw_; }
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

  State& getEndState();

  const State& getEndState() const;

  // Returns the cost of this trajectory. Just path length cost.
  double evaluateCost();

  // Returns the exploration gain of the trajectory (a combination of all
  // gains of all states). The states cache their own exploration gain, so
  // hopefully no unnecessary computations are performed here.
  double evaluateExplorationGain();

 private:
  std::vector<State> states_;
};

class System {
 public:
  // Returns the dimensionality of the Euclidean space.
  int getNumDimensions() { return 4; }

  //  Returns a reference to the root state.
  State& getRootState();

  // Returns the state_key for the given state.
  // state_in the given state
  // state_key the key to the state. An array of dimension
  // getNumDimensions()
  int getstate_key(State& state_in, double* state_key);

  // Returns true of the given state reaches the target.
  bool isReachingTarget(State& state_in) { return false; }

  // Returns a sample state.
  int sampleState(State& randomStateOut);

  // Returns a the cost of the trajectory that connects state_from_in and
  //       state_towards_in. The trajectory is also returned in trajectory_out.
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

 private:
  State root_state_;
};

}  // namespace nvbp_voxblox

#endif

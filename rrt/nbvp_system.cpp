#include "nbvp_voxblox/rrt/nbvp_system.h"
#include "nbvp_voxblox/utils.h"

namespace nbvp_voxblox {

int System::sampleState(State& random_state_out) {
  // Returns 1 if valid, 0 if invalid.
  // Sample a random state in the space.
  Eigen::Vector3d random_position(randMToN(lower_bound_.x(), upper_bound_.x()),
                                  randMToN(lower_bound_.y(), upper_bound_.y()),
                                  randMToN(lower_bound_.z(), upper_bound_.z()));

  if (!isInFreeSpace(random_position)) {
    return 0;
  }

  // Select a random yaw.
  double yaw = randMToN(0, M_PI);

  random_state_out.setPosition(random_position);
  random_state_out.setYaw(yaw);
  return 1;
}

int System::extendTo(State& state_from_in, State& state_towards_in,
                     Trajectory& trajectory_out, bool& exact_connection_out) {
  if (!isInFreeSpace(state_from_in.getPosition()) ||
      !isInFreeSpace(state_towards_in.getPosition())) {
    return 0;
  }

  // Check intermediate states.
  Eigen::Vector3d dir =
      state_towards_in.getPosition() - state_from_in.getPosition();
  double total_distance = dir.norm();
  dir.normalize();
  const double kSamplingDistance = 0.2;  // Meters
  // Last sample is anyway already checked (with the goal).
  int num_samples = std::floor(total_distance / kSamplingDistance);
  Eigen::Vector3d current_position = state_from_in.getPosition();
  // Can skip the zeroth, already checked.
  for (int i = 1; i < num_samples; ++i) {
    current_position += dir * kSamplingDistance;
    if (!isInFreeSpace(current_position)) {
      return 0;
    }
  }

  // If we made it this far, it's a valid trajectory.
  trajectory_out.addState(state_from_in);
  trajectory_out.addState(state_towards_in);
  exact_connection_out = true;

  return 1;
}

double System::evaluateExtensionCost(State& state_from_in,
                                     State& state_towards_in,
                                     bool& exact_connection_out) {
  double distance =
      (state_from_in.getPosition() - state_towards_in.getPosition()).norm();

  exact_connection_out = true;
  return distance;
}

int System::getTrajectory(State& state_from_in, State& state_to_in,
                          std::list<double*>& trajectory_out) {
  // C&P below
  constexpr int kNumDimensions = 4;
  double* state_array = new double[kNumDimensions];
  for (int i = 0; i < kNumDimensions; i++) {
    state_array[i] = state_to_in[i];
    trajectory_out.push_front(state_array);
  }
  return 1;
}

}  // namespace nbvp_voxblox

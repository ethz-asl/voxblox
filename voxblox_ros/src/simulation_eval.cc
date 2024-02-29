#include <gflags/gflags.h>
#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>

#include "voxblox_ros/simulation_server.h"

namespace voxblox {
class SimulationServerImpl : public voxblox::SimulationServer {
 public:
  SimulationServerImpl() : SimulationServer() {}

  void prepareWorld() {
    CHECK_NOTNULL(world_);
    world_->addObject(std::unique_ptr<Object>(
        new Sphere(Point(0.0, 0.0, 2.0), 2.0, Color::Red())));

    world_->addObject(std::unique_ptr<Object>(new PlaneObject(
        Point(-2.0, -4.0, 2.0), Point(0, 1, 0), Color::White())));

    world_->addObject(std::unique_ptr<Object>(
        new PlaneObject(Point(4.0, 0.0, 0.0), Point(-1, 0, 0), Color::Pink())));

    world_->addObject(std::unique_ptr<Object>(
        new Cube(Point(-4.0, 4.0, 2.0), Point(4, 4, 4), Color::Green())));

    world_->addGroundLevel(0.03);

    world_->generateSdfFromWorld(truncation_distance_, tsdf_gt_.get());
    world_->generateSdfFromWorld(esdf_max_distance_, esdf_gt_.get());
  }
};

}  // namespace voxblox

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  // google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  auto sim_eval = std::make_shared<voxblox::SimulationServerImpl>();
  sim_eval.run();

  RCLCPP_INFO(sim_eval->get_logger(), "Done.");

  rclcpp::spin(sim_eval);
  return 0;
}

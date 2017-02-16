#include <ros/ros.h>

#include <voxblox/simulation/simulation_world.h>

namespace voxblox {
class SimulationServer {
 public:
  SimulationServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : nh_(nh), nh_private_(nh_private) {}





 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher sim_pub_;

  SimulationWorld world_;
};

}  // namespace voxblox

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox_sim");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  voxblox::SimulationServer node(nh, nh_private);

  ros::spin();
  return 0;
}

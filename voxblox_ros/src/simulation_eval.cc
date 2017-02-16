#include <ros/ros.h>

#include <voxblox/simulation/simulation_world.h>

namespace voxblox {

class SimulationServer {
 public:
  SimulationServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : nh_(nh), nh_private_(nh_private) {}

  // Runs all of the below functions in the correct order:
  void run();

  // Creates a new world, and prepares ground truth SDF(s).
  void prepareWorld();

  // Generates a SDF by generating random poses and putting them into an SDF.
  void generateSDF() {}

  // Evaluate errors...
  void evaluate() {}

  // Visualize results. :)
  void visualize() {}

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher sim_pub_;

  // Actual simulation server.
  SimulationWorld world_;

  // Maps (GT and generates from sensors) generated here.
};

void SimulationServer::prepareWorld() {
  world_.addObject(
      std::unique_ptr<Object>(new Sphere(Point(2.0, 5.0, 2.0), 2.0)));

  FloatingPoint voxel_size = 0.05;
  int voxels_per_side = 16;
  std::unique_ptr<Layer<TsdfVoxel> > tsdf_gt(
      new Layer<TsdfVoxel>(voxel_size, voxels_per_side));
  world_.generateSdfFromWorld(0.5, tsdf_gt.get());

  std::unique_ptr<Layer<EsdfVoxel> > esdf_gt(
      new Layer<EsdfVoxel>(voxel_size, voxels_per_side));
  world_.generateSdfFromWorld(2.0, esdf_gt.get());
}

void SimulationServer::run() {
  prepareWorld();
  generateSDF();
  evaluate();
  visualize();
}

}  // namespace voxblox

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox_sim");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  voxblox::SimulationServer sim_eval(nh, nh_private);

  sim_eval.run();

  ROS_INFO("Done.");
  ros::spin();
  return 0;
}

#include <ros/ros.h>

#include <voxblox/simulation/simulation_world.h>
#include "voxblox_ros/ptcloud_vis.h"
#include "voxblox_ros/conversions.h"

namespace voxblox {

class SimulationServer {
 public:
  SimulationServer(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);

  // Runs all of the below functions in the correct order:
  void run();

  // Creates a new world, and prepares ground truth SDF(s).
  void prepareWorld();

  // Generates a SDF by generating random poses and putting them into an SDF.
  void generateSDF() {}

  // Evaluate errors...
  void evaluate() {}

  // Visualize results. :)
  void visualize();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // A bunch of publishers :)
  ros::Publisher sim_pub_;
  ros::Publisher tsdf_gt_pub_;
  ros::Publisher esdf_gt_pub_;

  std::string world_frame_;

  // Actual simulation server.
  SimulationWorld world_;

  // Maps (GT and generates from sensors) generated here.
  std::unique_ptr<Layer<TsdfVoxel> > tsdf_gt;
  std::unique_ptr<Layer<EsdfVoxel> > esdf_gt;
};

SimulationServer::SimulationServer(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), world_frame_("world") {
  FloatingPoint voxel_size = 0.20;
  int voxels_per_side = 16;

  tsdf_gt.reset(new Layer<TsdfVoxel>(voxel_size, voxels_per_side));
  esdf_gt.reset(new Layer<EsdfVoxel>(voxel_size, voxels_per_side));

  // ROS stuff.
  tsdf_gt_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "tsdf_gt", 1, true);

  esdf_gt_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "esdf_gt", 1, true);
}

void SimulationServer::prepareWorld() {
  world_.addObject(
      std::unique_ptr<Object>(new Sphere(Point(2.0, 5.0, 2.0), 2.0)));

  world_.addObject(std::unique_ptr<Object>(
      new Plane(Point(-2.0, -3.0, 2.0), Point(0, 1, 0))));

  world_.addObject(
      std::unique_ptr<Object>(new Cube(Point(-2.0, 5.0, 3.0), Point(2, 2, 2))));

  // world_.generateSdfFromWorld(0.5, tsdf_gt.get());
  world_.generateSdfFromWorld(2.0, esdf_gt.get());
}

void SimulationServer::visualize() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  pointcloud.header.frame_id = world_frame_;

  // createDistancePointcloudFromTsdfLayer(*tsdf_gt, &pointcloud);
  // tsdf_gt_pub_.publish(pointcloud);

  pointcloud.clear();
  createDistancePointcloudFromEsdfLayer(*esdf_gt, &pointcloud);
  esdf_gt_pub_.publish(pointcloud);
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

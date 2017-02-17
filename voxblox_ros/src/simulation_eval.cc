#include <ros/ros.h>

#include <voxblox/simulation/simulation_world.h>
#include <voxblox/mesh/mesh_integrator.h>

#include "voxblox_ros/ptcloud_vis.h"
#include "voxblox_ros/mesh_vis.h"
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
  ros::Publisher tsdf_gt_mesh_pub_;

  // Settings
  std::string world_frame_;
  bool visualize_;
  bool generate_mesh_;

  // Actual simulation server.
  SimulationWorld world_;

  // Maps (GT and generates from sensors) generated here.
  std::unique_ptr<Layer<TsdfVoxel> > tsdf_gt_;
  std::unique_ptr<Layer<EsdfVoxel> > esdf_gt_;
};

SimulationServer::SimulationServer(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      world_frame_("world"),
      visualize_(true),
      generate_mesh_(true) {
  FloatingPoint voxel_size = 0.20;
  int voxels_per_side = 16;

  tsdf_gt_.reset(new Layer<TsdfVoxel>(voxel_size, voxels_per_side));
  esdf_gt_.reset(new Layer<EsdfVoxel>(voxel_size, voxels_per_side));

  // ROS stuff.
  tsdf_gt_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "tsdf_gt", 1, true);
  esdf_gt_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "esdf_gt", 1, true);
  tsdf_gt_mesh_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "tsdf_gt_mesh", 1, true);
}

void SimulationServer::prepareWorld() {
  world_.addObject(std::unique_ptr<Object>(
      new Sphere(Point(2.0, 5.0, 2.0), 2.0, Color::Red())));

  world_.addObject(std::unique_ptr<Object>(
      new Plane(Point(-2.0, -3.0, 2.0), Point(0, 1, 0), Color::White())));

  world_.addObject(std::unique_ptr<Object>(
      new Cube(Point(-2.0, 5.0, 3.0), Point(2, 2, 2), Color::Green())));

  world_.addGroundLevel(0.0);

  world_.generateSdfFromWorld(0.5, tsdf_gt_.get());
  world_.generateSdfFromWorld(2.0, esdf_gt_.get());
}

void SimulationServer::visualize() {
  if (!visualize_) {
    return;
  }

  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  pointcloud.header.frame_id = world_frame_;

  createDistancePointcloudFromTsdfLayer(*tsdf_gt_, &pointcloud);
  tsdf_gt_pub_.publish(pointcloud);

  pointcloud.clear();
  createDistancePointcloudFromEsdfLayer(*esdf_gt_, &pointcloud);
  esdf_gt_pub_.publish(pointcloud);

  if (generate_mesh_) {
    // Generate TSDF GT mesh.
    MeshIntegrator::Config mesh_config;
    MeshLayer::Ptr mesh(new MeshLayer(tsdf_gt_->block_size()));
    MeshIntegrator mesh_integrator(mesh_config, tsdf_gt_.get(), mesh.get());
    mesh_integrator.generateWholeMesh();
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(1);
    ColorMode color_mode = ColorMode::kNormals;
    fillMarkerWithMesh(mesh, color_mode, &marker_array.markers[0]);
    marker_array.markers[0].header.frame_id = world_frame_;
    tsdf_gt_mesh_pub_.publish(marker_array);
  }
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

#include <ros/ros.h>
#include <memory>
#include <string>

#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/simulation/simulation_world.h>
#include <voxblox/skeletons/skeleton_generator.h>

#include "voxblox_ros/conversions.h"
#include "voxblox_ros/esdf_server.h"
#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"
#include "voxblox_ros/ros_params.h"
#include "voxblox_ros/skeleton_vis.h"

namespace voxblox {

class SkeletonEvalNode {
 public:
  SkeletonEvalNode(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);

  void generateWorld();
  void generateSkeleton();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher skeleton_pub_;
  ros::Publisher sparse_graph_pub_;

  std::string frame_id_;

  bool visualize_;
  bool full_euclidean_distance_;

  double esdf_max_distance_;
  double tsdf_max_distance_;

  voxblox::EsdfServer voxblox_server_;
  voxblox::SimulationWorld world_;
};

SkeletonEvalNode::SkeletonEvalNode(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      frame_id_("world"),
      visualize_(true),
      full_euclidean_distance_(true),
      esdf_max_distance_(5.0),
      tsdf_max_distance_(0.4),
      voxblox_server_(nh_, nh_private_) {
  nh_private_.param("visualize", visualize_, visualize_);
  nh_private_.param("full_euclidean_distance", full_euclidean_distance_,
                    full_euclidean_distance_);
  nh_private_.param("esdf_max_distance", esdf_max_distance_,
                    esdf_max_distance_);
  nh_private_.param("tsdf_max_distance", tsdf_max_distance_,
                    tsdf_max_distance_);

  skeleton_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ> >(
      "skeleton", 1, true);
  sparse_graph_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "sparse_graph", 1, true);
}

void SkeletonEvalNode::generateWorld() {
  voxblox_server_.clear();
  world_.clear();

  world_.addPlaneBoundaries(0.0, 15.0, 0.0, 10.0);
  world_.addGroundLevel(0.0);
  world_.addObject(std::unique_ptr<voxblox::Object>(
      new PlaneObject(Point(0.0, 0.0, 5.0), Point(0.0, 0.0, -1.0))));

  // Sets the display bounds.
  world_.setBounds(Point(-1.0, -1.0, -1.0), Point(16.0, 11.0, 6.0));

  // Corridor/room walls. Horizontal first, vertical second, top to bottom.
  world_.addObject(std::unique_ptr<voxblox::Object>(new voxblox::Cube(
      Point(3.75, 5.0, 2.5), Point(7.5, 0.5, 5.0), voxblox::Color::Gray())));
  world_.addObject(std::unique_ptr<voxblox::Object>(new voxblox::Cube(
      Point(9.5, 5.0, 2.5), Point(1, 0.5, 5.0), voxblox::Color::Gray())));

  world_.addObject(std::unique_ptr<voxblox::Object>(new voxblox::Cube(
      Point(1.25, 7.5, 2.5), Point(2.5, 0.5, 5.0), voxblox::Color::Gray())));
  world_.addObject(std::unique_ptr<voxblox::Object>(new voxblox::Cube(
      Point(6.5, 7.5, 2.5), Point(5, 0.5, 5.0), voxblox::Color::Gray())));
  world_.addObject(std::unique_ptr<voxblox::Object>(new voxblox::Cube(
      Point(12.5, 7.5, 2.5), Point(5, 0.5, 5.0), voxblox::Color::Gray())));

  world_.addObject(std::unique_ptr<voxblox::Object>(new voxblox::Cube(
      Point(10, 2.5, 2.5), Point(0.5, 5.0, 5.0), voxblox::Color::Gray())));
  world_.addObject(std::unique_ptr<voxblox::Object>(new voxblox::Cube(
      Point(8, 8.75, 2.5), Point(0.5, 2.5, 5.0), voxblox::Color::Gray())));
  world_.addObject(std::unique_ptr<voxblox::Object>(new voxblox::Cube(
      Point(5, 8.25, 2.5), Point(0.5, 1.5, 5.0), voxblox::Color::Gray())));

  // Random objects.
  world_.addObject(std::unique_ptr<voxblox::Object>(new voxblox::Cube(
      Point(2.5, 2.5, 2.5), Point(1.0, 1.0, 5.0), voxblox::Color::Gray())));
  world_.addObject(std::unique_ptr<voxblox::Object>(new voxblox::Cube(
      Point(3.5, 2.5, 1.25), Point(1.0, 1.0, 2.5), voxblox::Color::Green())));
  world_.addObject(std::unique_ptr<voxblox::Object>(new voxblox::Cube(
      Point(13.75, 8.5, 1.25), Point(2.5, 2.0, 2.5), voxblox::Color::Green())));

  // ..."Fun" object...
  /* world_.addObject(std::unique_ptr<voxblox::Object>(
      new voxblox::Cube(Point(12.5, 3.125, 1.25), Point(1.0, 3.75, 2.5),
                         voxblox::Color::Pink()))); */
  world_.addObject(std::unique_ptr<voxblox::Object>(new voxblox::Cylinder(
      Point(12.5, 5.0, 1.25), 0.5, 2.5, voxblox::Color::Pink())));
  world_.addObject(std::unique_ptr<voxblox::Object>(new voxblox::Sphere(
      Point(11.5, 1.5, 1.25), 1.0, voxblox::Color::Pink())));
  world_.addObject(std::unique_ptr<voxblox::Object>(new voxblox::Sphere(
      Point(13.5, 1.5, 1.25), 1.0, voxblox::Color::Pink())));

  voxblox_server_.setSliceLevel(1.5);
  // world_.generateSdfFromWorld<voxblox::EsdfVoxel>(
  //    esdf_max_distance_, voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());
  world_.generateSdfFromWorld<voxblox::TsdfVoxel>(
      tsdf_max_distance_, voxblox_server_.getTsdfMapPtr()->getTsdfLayerPtr());
  voxblox_server_.setEsdfMaxDistance(esdf_max_distance_);
  voxblox_server_.updateEsdfBatch(full_euclidean_distance_);
  if (visualize_) {
    voxblox_server_.generateMesh();
    voxblox_server_.publishSlices();
    voxblox_server_.publishPointclouds();
  }
}

void SkeletonEvalNode::generateSkeleton() {
  SkeletonGenerator skeleton_generator(
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());
  skeleton_generator.setMinSeparationAngle(0.7);
  bool generate_by_layer_neighbors =
      skeleton_generator.getGenerateByLayerNeighbors();
  nh_private_.param("generate_by_layer_neighbors", generate_by_layer_neighbors,
                    generate_by_layer_neighbors);
  skeleton_generator.setGenerateByLayerNeighbors(generate_by_layer_neighbors);

  int num_neighbors_for_edge = skeleton_generator.getNumNeighborsForEdge();
  nh_private_.param("num_neighbors_for_edge", num_neighbors_for_edge,
                    num_neighbors_for_edge);
  skeleton_generator.setNumNeighborsForEdge(num_neighbors_for_edge);

  skeleton_generator.generateSkeleton();

  Pointcloud pointcloud;
  std::vector<float> distances;
  skeleton_generator.getSkeleton().getEdgePointcloudWithDistances(&pointcloud,
                                                                  &distances);

  // Publish the skeleton.
  pcl::PointCloud<pcl::PointXYZI> ptcloud_pcl;
  pointcloudToPclXYZI(pointcloud, distances, &ptcloud_pcl);
  ptcloud_pcl.header.frame_id = frame_id_;
  skeleton_pub_.publish(ptcloud_pcl);
  ROS_INFO("Finished generating skeleton.");

  skeleton_generator.generateSparseGraph();
  ROS_INFO("Finished generating sparse graph.");

  skeleton_generator.splitEdges();

  ROS_INFO_STREAM("Total Timings: " << std::endl << timing::Timing::Print());

  // Now visualize the graph.
  const SparseSkeletonGraph& graph = skeleton_generator.getSparseGraph();
  visualization_msgs::MarkerArray marker_array;
  visualizeSkeletonGraph(graph, frame_id_, &marker_array);
  sparse_graph_pub_.publish(marker_array);
}

}  // namespace voxblox

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox_skeletonizer");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  FLAGS_alsologtostderr = true;

  voxblox::SkeletonEvalNode node(nh, nh_private);

  node.generateWorld();
  node.generateSkeleton();

  ros::spin();
  return 0;
}

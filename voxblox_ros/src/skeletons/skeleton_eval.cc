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

  void generateMapFromGroundTruth();
  void generateMapFromRobotPoses(int num_poses, int seed,
                                 FloatingPoint noise_level);

  // Utility functions.
  double randMToN(double m, double n) const;

  bool selectRandomFreePose(const Point& min_bound, const Point& max_bound,
                            Point* sampled_pose) const;
  void transformPointcloud(const voxblox::Transformation& T_N_O,
                           const voxblox::Pointcloud& ptcloud,
                           voxblox::Pointcloud* ptcloud_out) const;

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

  Eigen::Vector2i camera_resolution_;
  double camera_fov_h_rad_;
  double camera_min_dist_;
  double camera_max_dist_;

  bool apply_noise_;
  FloatingPoint noise_sigma_;

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
      camera_resolution_(320, 240),
      camera_fov_h_rad_(1.5708),  // 90 deg
      camera_min_dist_(0.5),
      camera_max_dist_(10.0),
      apply_noise_(false),
      noise_sigma_(0.0),
      voxblox_server_(nh_, nh_private_) {
  nh_private_.param("visualize", visualize_, visualize_);
  nh_private_.param("full_euclidean_distance", full_euclidean_distance_,
                    full_euclidean_distance_);
  nh_private_.param("esdf_max_distance", esdf_max_distance_,
                    esdf_max_distance_);
  nh_private_.param("tsdf_max_distance", tsdf_max_distance_,
                    tsdf_max_distance_);
  nh_private_.param("apply_noise", apply_noise_, apply_noise_);
  nh_private_.param("noise_sigma", noise_sigma_, noise_sigma_);

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
}

void SkeletonEvalNode::generateMapFromGroundTruth() {
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

double SkeletonEvalNode::randMToN(double m, double n) const {
  return m + (rand() / (RAND_MAX / (n - m)));
}

bool SkeletonEvalNode::selectRandomFreePose(const Point& min_bound,
                                            const Point& max_bound,
                                            Point* sampled_pose) const {
  CHECK_NOTNULL(sampled_pose);

  const int max_tries = 100;
  const FloatingPoint max_dist = 5.0;
  const FloatingPoint robot_radius = 0.5;

  for (int i = 0; i < max_tries; ++i) {
    *sampled_pose = Point(randMToN(min_bound.x(), max_bound.x()),
                          randMToN(min_bound.y(), max_bound.y()),
                          randMToN(min_bound.z(), max_bound.z()));
    if (world_.getDistanceToPoint(*sampled_pose, max_dist) > robot_radius) {
      return true;
    }
  }
  return false;
}

void SkeletonEvalNode::transformPointcloud(
    const voxblox::Transformation& T_N_O, const voxblox::Pointcloud& ptcloud,
    voxblox::Pointcloud* ptcloud_out) const {
  ptcloud_out->clear();
  ptcloud_out->resize(ptcloud.size());

  for (size_t i = 0; i < ptcloud.size(); ++i) {
    (*ptcloud_out)[i] = T_N_O * ptcloud[i];
  }
}

void SkeletonEvalNode::generateMapFromRobotPoses(int num_poses, int seed,
                                                 FloatingPoint noise_level) {
  voxblox_server_.setSliceLevel(1.5);
  voxblox_server_.setClearSphere(true);
  voxblox_server_.setEsdfMaxDistance(esdf_max_distance_);

  srand(seed);

  Point min_boundary(1.0, 1.0, 1.0);
  Point max_boundary(14.0, 10.0, 5.0);

  for (int i = 0; i < num_poses; i++) {
    Point view_origin = Point::Zero();
    selectRandomFreePose(min_boundary, max_boundary, &view_origin);
    Point view_direction(1.0, 0.0, 0.0);
    Eigen::Quaternionf view_orientation(
        Eigen::AngleAxisf(randMToN(0, 2 * M_PI), Eigen::Vector3f::UnitZ()));
    view_direction = view_orientation * view_direction;

    // T_G_C is from z-positive since that's how camera coordinates work.
    voxblox::Transformation T_G_C(
        view_origin.cast<float>(),
        Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(0.0, 0.0, 1.0),
                                           view_direction));
    // Step 2: actually get the pointcloud.
    voxblox::Pointcloud ptcloud, ptcloud_C;
    voxblox::Colors colors;
    if (!apply_noise_) {
      world_.getPointcloudFromViewpoint(view_origin, view_direction,
                                        camera_resolution_, camera_fov_h_rad_,
                                        camera_max_dist_, &ptcloud, &colors);
    } else {
      world_.getNoisyPointcloudFromViewpoint(
          view_origin, view_direction, camera_resolution_, camera_fov_h_rad_,
          camera_max_dist_, noise_sigma_, &ptcloud, &colors);
    }

    // Step 3: integrate into the map.
    // Transform back into camera frame.
    transformPointcloud(T_G_C.inverse(), ptcloud, &ptcloud_C);
    voxblox_server_.integratePointcloud(T_G_C, ptcloud_C, colors);

    // Step 4: update mesh and ESDF. NewPoseCallback will mark unknown as
    // occupied and clear space otherwise.
    // voxblox_server_.newPoseCallback(T_G_C);
    // voxblox_server_.updateEsdf();
  }

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

  FloatingPoint min_separation_angle =
      skeleton_generator.getMinSeparationAngle();
  nh_private_.param("min_separation_angle", min_separation_angle,
                    min_separation_angle);
  skeleton_generator.setMinSeparationAngle(min_separation_angle);
  bool generate_by_layer_neighbors =
      skeleton_generator.getGenerateByLayerNeighbors();
  nh_private_.param("generate_by_layer_neighbors", generate_by_layer_neighbors,
                    generate_by_layer_neighbors);
  skeleton_generator.setGenerateByLayerNeighbors(generate_by_layer_neighbors);

  int num_neighbors_for_edge = skeleton_generator.getNumNeighborsForEdge();
  nh_private_.param("num_neighbors_for_edge", num_neighbors_for_edge,
                    num_neighbors_for_edge);
  skeleton_generator.setNumNeighborsForEdge(num_neighbors_for_edge);

  FloatingPoint min_gvd_distance = skeleton_generator.getMinGvdDistance();
  nh_private_.param("min_gvd_distance", min_gvd_distance, min_gvd_distance);
  skeleton_generator.setMinGvdDistance(min_gvd_distance);

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
  skeleton_generator.repairGraph();
  ROS_INFO("Finished generating sparse graph.");

  ROS_INFO_STREAM("Total Timings: " << std::endl << timing::Timing::Print());

  // Now visualize the graph.
  const SparseSkeletonGraph& graph = skeleton_generator.getSparseGraph();
  visualization_msgs::MarkerArray marker_array;
  visualizeSkeletonGraph(graph, frame_id_, &marker_array);
  sparse_graph_pub_.publish(marker_array);

  std::vector<int64_t> vertex_ids, edge_ids;
  skeleton_generator.getSparseGraph().getAllEdgeIds(&edge_ids);
  skeleton_generator.getSparseGraph().getAllVertexIds(&vertex_ids);

  // Now output some staaaaats.
  ROS_INFO("Map voxel size: %f Noise on? %d Noise level: %f",
           voxblox_server_.getTsdfMapPtr()->getTsdfLayerPtr()->voxel_size(),
           apply_noise_, noise_sigma_);
  ROS_INFO(
      "Diagram edges: %lu Diagram vertices: %lu Graph edges: %lu Graph "
      "vertices: %lu",
      skeleton_generator.getSkeleton().getEdgePoints().size(),
      skeleton_generator.getSkeleton().getVertexPoints().size(),
      edge_ids.size(), vertex_ids.size());
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
  //node.generateMapFromRobotPoses(200, 1, 0.0);

  node.generateMapFromGroundTruth();
  node.generateSkeleton();

  ros::spin();
  return 0;
}

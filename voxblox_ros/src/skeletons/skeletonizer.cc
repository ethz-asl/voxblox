#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/skeletons/skeleton_generator.h>

#include "voxblox_ros/conversions.h"
#include "voxblox_ros/esdf_server.h"
#include "voxblox_ros/mesh_pcl.h"
#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"
#include "voxblox_ros/skeleton_vis.h"

namespace voxblox {

class SkeletonizerNode {
 public:
  SkeletonizerNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : nh_(nh), nh_private_(nh_private), esdf_server_(nh_, nh_private_) {
    skeleton_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ> >(
        "skeleton", 1, true);
    sparse_graph_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
        "sparse_graph", 1, true);
  }

  // Initialize the node.
  void init();

  // Make a skeletor!!!
  void skeletonize(Layer<EsdfVoxel>* esdf_layer, voxblox::Pointcloud* skeleton,
                   std::vector<float>* distances);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher skeleton_pub_;
  ros::Publisher sparse_graph_pub_;

  EsdfServer esdf_server_;
};

void SkeletonizerNode::init() {
  // Load a file from the params.
  std::string input_filepath;
  nh_private_.param("input_filepath", input_filepath, input_filepath);

  if (input_filepath.empty()) {
    return;
  }

  esdf_server_.loadMap(input_filepath);

  // Visualize all parts.
  esdf_server_.updateMesh();
  esdf_server_.publishPointclouds();
  esdf_server_.publishMap();

  // Skeletonize????
  voxblox::Pointcloud pointcloud;
  std::vector<float> distances;
  skeletonize(esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr(), &pointcloud,
              &distances);

  // Publish the skeleton.
  pcl::PointCloud<pcl::PointXYZI> ptcloud_pcl;
  pointcloudToPclXYZI(pointcloud, distances, &ptcloud_pcl);
  ptcloud_pcl.header.frame_id = "world";
  skeleton_pub_.publish(ptcloud_pcl);

  // Optionally save back to file.
}

void SkeletonizerNode::skeletonize(Layer<EsdfVoxel>* esdf_layer,
                                   voxblox::Pointcloud* pointcloud,
                                   std::vector<float>* distances) {
  SkeletonGenerator skeleton_generator(esdf_layer);

  bool generate_by_layer_neighbors =
      skeleton_generator.getGenerateByLayerNeighbors();
  skeleton_generator.setMinSeparationAngle(-0.0);

  nh_private_.param("generate_by_layer_neighbors", generate_by_layer_neighbors,
                    generate_by_layer_neighbors);
  skeleton_generator.setGenerateByLayerNeighbors(generate_by_layer_neighbors);

  int num_neighbors_for_edge = skeleton_generator.getNumNeighborsForEdge();
  nh_private_.param("num_neighbors_for_edge", num_neighbors_for_edge,
                    num_neighbors_for_edge);
  skeleton_generator.setNumNeighborsForEdge(num_neighbors_for_edge);


  skeleton_generator.generateSkeleton();
  skeleton_generator.getSkeleton().getEdgePointcloudWithDistances(pointcloud,
                                                              distances);
  ROS_INFO("Finished generating skeleton.");

  skeleton_generator.generateSparseGraph();
  ROS_INFO("Finished generating sparse graph.");

  ROS_INFO_STREAM("Total Timings: " << std::endl << timing::Timing::Print());

  // Now visualize the graph.
  const SparseSkeletonGraph& graph = skeleton_generator.getSparseGraph();
  visualization_msgs::MarkerArray marker_array;
  visualizeSkeletonGraph(graph, "world", &marker_array);
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

  voxblox::SkeletonizerNode node(nh, nh_private);

  node.init();

  ros::spin();
  return 0;
}

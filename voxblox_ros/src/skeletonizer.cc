#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/esdf_map.h>

#include "voxblox_ros/esdf_server.h"
#include "voxblox_ros/mesh_pcl.h"
#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"

namespace voxblox {

class SkeletonizerNode {
 public:
  SkeletonizerNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : nh_(nh), nh_private_(nh_private), esdf_server_(nh_, nh_private_) {}

  // Initialize the node.
  void init();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

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

  // Optionally save back to file.
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

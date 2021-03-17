#include "voxblox_ros/simple_tsdf_visualizer.h"

#include <voxblox/io/layer_io.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "visualize_tsdf_node");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh_private("~");

  std::string tsdf_proto_path = "";
  nh_private.param("tsdf_proto_path", tsdf_proto_path, tsdf_proto_path);
  if (tsdf_proto_path.empty()) {
    ROS_FATAL_STREAM(
        "Please provide a TSDF proto file to visualize using the ros "
        << "parameter: tsdf_proto_path");
    ros::shutdown();
    return 1;
  }
  ROS_INFO_STREAM("Visualize TSDF grid from " << tsdf_proto_path);

  ROS_INFO_STREAM("Loading...");
  voxblox::Layer<voxblox::TsdfVoxel>::Ptr tsdf_layer;
  if (!voxblox::io::LoadLayer<voxblox::TsdfVoxel>(tsdf_proto_path,
                                                  &tsdf_layer)) {
    ROS_FATAL_STREAM("Unable to load a TSDF grid from: " << tsdf_proto_path);
    ros::shutdown();
    return 1;
  }
  CHECK(tsdf_layer);
  ROS_INFO_STREAM("Done.");

  ROS_INFO_STREAM("Visualizing...");
  voxblox::SimpleTsdfVisualizer visualizer(nh_private);
  visualizer.run(*tsdf_layer);
  ROS_INFO_STREAM("Done.");

  ros::spin();

  return 0;
}

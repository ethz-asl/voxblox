#include "voxblox_ros/tsdf_server.h"

#include <gflags/gflags.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  // google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  auto node = std::make_shared<rclcpp::Node>("tsdf_server");
  auto tsdf_server = std::make_shared<voxblox::TsdfServer>(node);
  rclcpp::spin(node);
  return 0;
}

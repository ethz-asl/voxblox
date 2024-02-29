#include "voxblox_ros/esdf_server.h"

#include <gflags/gflags.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  // google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  voxblox::EsdfServer::SharedPtr node = std::make_shared<voxblox::EsdfServer>();
  rclcpp::spin(node);
  return 0;
}

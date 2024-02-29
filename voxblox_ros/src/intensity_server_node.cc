#include "voxblox_ros/intensity_server.h"

#include <gflags/gflags.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  // google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  voxblox::IntensityServer::SharedPtr node =
      std::make_shared<voxblox::IntensityServer::SharedPtr>();
  rclcpp::spin(node);
  return 0;
}

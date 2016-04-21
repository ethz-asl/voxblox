#include "voxblox/core/block.h"
#include "voxblox/core/map.h"
#include "voxblox/integrator/ray_integrator.h"
#include "voxblox/io/sdf_ply.h"

#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace voxblox;

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  voxblox::TsdfBlock my_cool_block(Eigen::Vector3d::Zero(), 8, 0.1);
  voxblox::TsdfMap::Ptr my_cool_map(new voxblox::TsdfMap(8, 0.1));
  /*my_cool_map->allocateBlockPtrByIndex(voxblox::BlockIndex(1, 2, 3));

  std::cout << "Started putting lots of boxes in.\n";
  int box_size = 1;
  for (int i = -box_size; i < box_size; ++i) {
    for (int j = -box_size; j < box_size; ++j) {
      for (int k = -box_size; k < box_size; ++k) {
        my_cool_map->allocateBlockPtrByIndex(voxblox::BlockIndex(i, j, k));
      }
    }
  }
  std::cout << "Finished putting lots of boxes in.\n"; */



  voxblox::Coordinates my_coordinate(1, 2, 3);
  std::cout << "Output ply to test_tsdf.ply\n";

  voxblox::Integrator my_cool_integrator(my_cool_map,
                                         Integrator::IntegratorConfig());

  Point sensor_origin(1, 0.4, 2.3);
  Transformation transform(sensor_origin, Eigen::Quaterniond::Identity());

  Pointcloud measurements;
  measurements.push_back(transform.inverse() * Point(4.4, 6.8, 1.11));
  Colors colors;
  colors.push_back(Color());

  my_cool_integrator.integratePointCloud(transform, measurements, colors);

  // Now output the ply file.
  voxblox::io::outputMapAsPly(*my_cool_map, "test_tsdf.ply",
                              voxblox::io::kSdfDistanceColor);

  return 0;
}

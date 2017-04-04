#include "voxblox/core/tsdf_map.h"
#include "voxblox/integrator/tsdf_integrator.h"
#include "voxblox/integrator/merge_integrator.h"
#include "voxblox/io/sdf_ply.h"
#include "voxblox/mesh/mesh_integrator.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace voxblox;  // NOLINT

// TODO(mfehr, helenol): Replace this playground with proper unit tests.
int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  TsdfMap::Config config;
  config.tsdf_voxel_size = 0.2;
  config.tsdf_voxels_per_side = 16;

  TsdfIntegrator::Config integrator_config;
  integrator_config.default_truncation_distance = 0.1;
  integrator_config.max_weight = 1000.0;
  integrator_config.voxel_carving_enabled = true;

  TsdfMap::Ptr my_cool_map = std::make_shared<TsdfMap>(config);
  TsdfIntegrator my_cool_integrator(integrator_config,
                                    my_cool_map->getTsdfLayerPtr());

  Point sensor_origin(1, 0.4, 2.3);
  Transformation transform(sensor_origin, Rotation::Implementation::Identity());

  Pointcloud measurements;
  measurements.push_back(transform.inverse() * Point(4.4, 6.8, 1.11));
  Colors colors;
  colors.push_back(Color());

  my_cool_integrator.integratePointCloud(transform, measurements, colors);

  // Now output the ply file.
  LOG(INFO) << "Output ply to test_tsdf.ply\n";
  voxblox::io::outputLayerAsPly(my_cool_map->getTsdfLayer(), "test_tsdf.ply",
                                voxblox::io::kSdfDistanceColor);

  return 0;
}

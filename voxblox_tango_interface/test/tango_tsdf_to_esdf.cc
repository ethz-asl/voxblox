#include <iostream>  // NOLINT

#include <voxblox/core/esdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>

#include "voxblox_tango_interface/io/tango_layer_io.h"
#include "voxblox_tango_interface/core/tango_layer_interface.h"

using namespace voxblox;  // NOLINT

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  if (argc != 7) {
    throw std::runtime_error(
        std::string("Args: filename to load, filename to save to") +
        ", min weight, min fixed distance" +
        ", max esdf distance, default esdf distance");
  }

  const std::string file = argv[1];
  FloatingPoint min_weight = std::stof(argv[3]);
  FloatingPoint min_distance_m = std::stof(argv[4]);
  FloatingPoint max_distance_m = std::stof(argv[5]);
  FloatingPoint default_distance_m = std::stof(argv[6]);

  TangoLayerInterface::Ptr layer_from_file;
  io::TangoLoadLayer(file, &layer_from_file);

  // TODO(mereweth@jpl.nasa.gov) - can we get truncation distance from ntsdf
  // proto?
  if (min_distance_m > layer_from_file->block_size()) {
    // Make sure that this is the same as the truncation distance OR SMALLER!
    min_distance_m = layer_from_file->block_size();
  }

  LOG(WARNING) << "Layer memory size: " << layer_from_file->getMemorySize()
               << "\n";
  LOG(WARNING) << "Layer voxel size: " << layer_from_file->voxel_size() << "\n";
  LOG(WARNING) << "Layer voxels per side: "
               << layer_from_file->voxels_per_side() << "\n";

  // ESDF maps.
  EsdfMap::Config esdf_config;
  // Same number of voxels per side for ESDF as with TSDF
  esdf_config.esdf_voxels_per_side = layer_from_file->voxels_per_side();
  // Same voxel size for ESDF as with TSDF
  esdf_config.esdf_voxel_size = layer_from_file->voxel_size();

  EsdfIntegrator::Config esdf_integrator_config;
  esdf_integrator_config.min_weight = min_weight;
  esdf_integrator_config.min_distance_m = min_distance_m;
  esdf_integrator_config.max_distance_m = max_distance_m;
  esdf_integrator_config.default_distance_m = default_distance_m;

  EsdfMap esdf_map(esdf_config);
  EsdfIntegrator esdf_integrator(esdf_integrator_config, layer_from_file.get(),
                                 esdf_map.getEsdfLayerPtr());

  esdf_integrator.updateFromTsdfLayerBatchFullEuclidean();

  bool esdfSuccess = io::SaveLayer(esdf_map.getEsdfLayer(), argv[2]);

  if (esdfSuccess == false) {
    throw std::runtime_error("Failed to save ESDF");
  }

  return 0;
}

#include <iostream>  // NOLINT
#include <string>  // NOLINT

#include <voxblox/integrator/merge_integrator.h>

#include "voxblox/io/layer_io.h"
#include "voxblox_tango_interface/io/tango_layer_io.h"
#include "voxblox_tango_interface/core/tango_layer_interface.h"

using namespace voxblox;  // NOLINT

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  if (argc != 5) {
    throw std::runtime_error(std::string("Args: filename to load, filename to save to")
                             + ", new voxel size, new # voxels per block side");
  }

  const std::string file = argv[1];
  FloatingPoint voxel_size = std::stof(argv[3]);
  size_t voxels_per_side = std::stoi(argv[4]);

  TangoLayerInterface::Ptr layer_from_file;
  io::TangoLoadLayer(file, &layer_from_file);

  LOG(WARNING) << "Layer memory size: " << layer_from_file->getMemorySize() << "\n";
  LOG(WARNING) << "Old layer voxel size: " << layer_from_file->voxel_size() << "\n";
  LOG(WARNING) << "Old layer voxels per side: " << layer_from_file->voxels_per_side() << "\n";

  Layer<TsdfVoxel>::Ptr resized_layer =
            std::make_shared<Layer<TsdfVoxel> >(voxel_size, voxels_per_side);
  LOG(WARNING) << "New layer voxel size: " << resized_layer->voxel_size() << "\n";
  LOG(WARNING) << "New layer voxels per side: " << resized_layer->voxels_per_side() << "\n";
  MergeIntegrator::MergeLayerAintoLayerB<TsdfVoxel>(*layer_from_file,
                                                    resized_layer.get());
  LOG(WARNING) << "New layer memory size after merge: " << resized_layer->getMemorySize() << "\n";

  bool tsdfSuccess = io::SaveLayer(*resized_layer, argv[2]);

  if (tsdfSuccess == false) {
    throw std::runtime_error("Failed to save TSDF");
  }

  return 0;
}

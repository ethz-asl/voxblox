#include <iostream>  // NOLINT
#include <string>    // NOLINT

#include <voxblox/integrator/merge_integration.h>

#include "voxblox/io/layer_io.h"
#include "voxblox_tango_interface/core/tango_layer_interface.h"
#include "voxblox_tango_interface/io/tango_layer_io.h"

using namespace voxblox;  // NOLINT

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  if ((argc != 5) && (argc != 3)) {
    throw std::runtime_error(
        std::string("Args: filename to load, filename to save to\n") +
        "Optional args: new voxel size, new # voxels per block side");
  }

  const std::string file = argv[1];
  bool tsdf_success = false;

  TangoLayerInterface::Ptr layer_from_file;
  io::TangoLoadLayer(file, &layer_from_file);

  LOG(INFO) << "Layer memory size: " << layer_from_file->getMemorySize();
  LOG(INFO) << "Old layer voxel size: " << layer_from_file->voxel_size();
  LOG(INFO) << "Old layer voxels per side: "
            << layer_from_file->voxels_per_side();

  if (argc == 5) {
    const FloatingPoint voxel_size = std::stof(argv[3]);
    const size_t voxels_per_side = std::stoi(argv[4]);

    Layer<TsdfVoxel>::Ptr resized_layer =
        std::make_shared<Layer<TsdfVoxel> >(voxel_size, voxels_per_side);
    LOG(INFO) << "New layer voxel size: " << resized_layer->voxel_size();
    LOG(INFO) << "New layer voxels per side: "
              << resized_layer->voxels_per_side();
    mergeLayerAintoLayerB<TsdfVoxel>(*layer_from_file, resized_layer.get());
    LOG(INFO) << "New layer memory size after merge: "
              << resized_layer->getMemorySize();

    tsdf_success = io::SaveLayer(*resized_layer, argv[2]);
  } else {
    tsdf_success = io::SaveLayer(*layer_from_file, argv[2]);
  }

  if (tsdf_success == false) {
    throw std::runtime_error("Failed to save TSDF");
  }

  return 0;
}

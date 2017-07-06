#include <iostream>  // NOLINT
#include <string>  // NOLINT

#include "voxblox/io/layer_io.h"
#include "voxblox_tango_interface/io/tango_layer_io.h"
#include "voxblox_tango_interface/core/tango_layer_interface.h"

using namespace voxblox;  // NOLINT

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  if (argc != 2) {
    throw std::runtime_error(std::string("Args: filename to load"));
  }

  const std::string file = argv[1];

  TangoLayerInterface::Ptr layer_from_file;
  io::TangoLoadLayer(file, &layer_from_file);

  TangoLayerInterface test_null_layer(nullptr);
  TangoLayerInterface test_layer(layer_from_file);
  LOG(WARNING) << "Layer memory size: " << test_layer.getMemorySize() << "\n";
  LOG(WARNING) << "Layer voxel size: " << test_layer.voxel_size() << "\n";
  LOG(WARNING) << "Layer voxels per side: " << test_layer.voxels_per_side() << "\n";

  return 0;
}

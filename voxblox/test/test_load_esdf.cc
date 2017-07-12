#include <iostream>  // NOLINT

#include "./Block.pb.h"
#include "./Layer.pb.h"
#include "voxblox/core/block.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/io/layer_io.h"
#include "voxblox/test/layer_test_utils.h"

#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>

using namespace voxblox;  // NOLINT

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  if (argc != 2) {
    throw std::runtime_error("Args: filename to load");
  }

  const std::string file = argv[1];

  Layer<EsdfVoxel>::Ptr layer_from_file;
  io::LoadLayer<EsdfVoxel>(file, &layer_from_file);

  std::cout << "Layer memory size: " << layer_from_file->getMemorySize()
            << "\n";

  return 0;
}

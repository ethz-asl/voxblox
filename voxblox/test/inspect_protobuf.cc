#include <iostream>  // NOLINT

#include "./Block.pb.h"
#include "./Layer.pb.h"
#include "voxblox/core/block.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/io/layer_io.h"
#include "voxblox/test/layer_test_utils.h"

using namespace voxblox;  // NOLINT

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  const std::string file = argv[1];

  Layer<TangoTsdfVoxel>::Ptr layer_from_file;
  io::LoadLayer<TangoTsdfVoxel>(file, &layer_from_file);

  return 0;
}

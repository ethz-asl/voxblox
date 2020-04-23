#include <memory>
#include <string>

#include <glog/logging.h>

#include "voxblox/core/block.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/io/layer_io.h"

using namespace voxblox;  // NOLINT

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  if (argc != 2) {
    throw std::runtime_error("Args: filename to load");
  }

  const std::string file = argv[1];

  Layer<EsdfVoxel>::Ptr layer_from_file;
  io::LoadLayer<EsdfVoxel>(file, &layer_from_file);

  LOG(INFO) << "Layer memory size: " << layer_from_file->getMemorySize();

  return 0;
}

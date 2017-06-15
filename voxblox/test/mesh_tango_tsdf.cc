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

  if (argc != 3) {
    throw std::runtime_error("Args: filename to load, followed by filename to save to");
  }

  const std::string file = argv[1];

  Layer<TangoTsdfVoxel>::Ptr layer_from_file;
  /* io::LoadLayerHeader<TangoTsdfVoxel>(file, &layer_from_file);
   * io::LoadBlocksFromFile<TangoTsdfVoxel>(file,
   *     Layer<TangoTsdfVoxel>::BlockMergingStrategy::kReplace, layer_from_file.get());
   */

  io::LoadLayer<TangoTsdfVoxel>(file, &layer_from_file);

  std::cout << "Layer memory size: " << layer_from_file->getMemorySize() << "\n";

  // Mesh accessories.
  MeshIntegrator<TangoTsdfVoxel>::Config mesh_config;
  std::shared_ptr<MeshLayer> mesh_layer_;
  std::unique_ptr<MeshIntegrator<TangoTsdfVoxel> > mesh_integrator_;

  mesh_layer_.reset(new MeshLayer(layer_from_file->block_size()));
  mesh_integrator_.reset(new MeshIntegrator<TangoTsdfVoxel>(
      mesh_config, layer_from_file.get(), mesh_layer_.get()));

  mesh_integrator_->generateWholeMesh();
  std::cout << "Number of meshes: " << mesh_layer_->getNumberOfAllocatedMeshes() << "\n";
  bool meshSuccess = outputMeshLayerAsPly(argv[2], *mesh_layer_);

  if (meshSuccess == false) {
    throw std::runtime_error("Failed to save mesh");
  }

  return 0;
}

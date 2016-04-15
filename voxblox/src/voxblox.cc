#include "voxblox/core/block.h"
#include "voxblox/core/map.h"
#include "voxblox/integrator/ray_integrator.h"
#include "voxblox/io/sdf_ply.h"

#include <iostream>

int main(int argc, char* argv[]) {
  voxblox::TsdfBlock my_cool_block(Eigen::Vector3d::Zero(), 8, 0.1);
  voxblox::TsdfMap my_cool_map(8, 0.2);
  my_cool_map.allocateBlockPtrByIndex(voxblox::BlockIndex(1, 2, 3));

  std::cout << "Started putting lots of boxes in.\n";
  int box_size = 1;
  for (int i = -box_size; i < box_size; ++i) {
    for (int j = -box_size; j < box_size; ++j) {
      for (int k = -box_size; k < box_size; ++k) {
        my_cool_map.allocateBlockPtrByIndex(voxblox::BlockIndex(i, j, k));
      }
    }
  }
  std::cout << "Finished putting lots of boxes in.\n";

  // Now output the ply file.
  voxblox::io::outputMapAsPly(my_cool_map, "test_tsdf.ply",
                              voxblox::io::kSdfDistanceColor);

  std::cout << "Output ply to test_tsdf.ply\n";

  return 0;
}

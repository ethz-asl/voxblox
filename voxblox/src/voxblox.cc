#include "voxblox/core/block.h"
#include "voxblox/core/map.h"

#include <iostream>

int main(int argc, char* argv[]) {
  voxblox::TsdfBlock my_cool_block(Eigen::Vector3d::Zero(), 8, 0.1);
  voxblox::TsdfMap my_cool_map(8, 0.2);
  my_cool_map.allocateBlockPtrByIndex(voxblox::BlockIndex(1, 2, 3));

  std::cout << "Started putting lots of boxes in.\n";
  for (int i = -10; i < 10; ++i) {
    for (int j = -10; j < 10; ++j) {
      for (int k = -10; k < 10; ++k) {
        my_cool_map.allocateBlockPtrByIndex(voxblox::BlockIndex(i, j, k));
      }
    }
  }
  std::cout << "Finished putting lots of boxes in.\n";
  return 0;
}

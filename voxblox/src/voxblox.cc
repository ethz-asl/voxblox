#include <voxblox/block.h>
#include "voxblox/map.h"

int main(int argc, char* argv[]) {

  voxblox::TsdfBlock my_cool_block(Eigen::Vector3d::Zero(), 8, 0.1);
  voxblox::TsdfMap my_cool_map(0.2, 10);
}


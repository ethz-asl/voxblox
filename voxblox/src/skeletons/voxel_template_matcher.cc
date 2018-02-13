#include "voxblox/skeletons/voxel_template_matcher.h"

namespace voxblox {

VoxelTemplateMatcher::VoxelTemplateMatcher() {}

void VoxelTemplateMatcher::addTemplate(const VoxelTemplate& voxel_template) {
  templates_.push_back(voxel_template);
}

void VoxelTemplateMatcher::addIntegerTemplate(int32_t neighbor_mask_dec,
                                              int32_t neighbor_template_dec) {
  VoxelTemplate voxel_template;
  voxel_template.neighbor_mask = neighbor_mask_dec;
  voxel_template.neighbor_template = neighbor_template_dec;
}

bool VoxelTemplateMatcher::fitsTemplates(
    const std::bitset<27>& voxel_neighbors) {
  for (const VoxelTemplate& temp : templates_) {
    if (((voxel_neighbors ^ temp.neighbor_template) & temp.neighbor_mask)
            .none()) {
      return true;
    }
  }
  return false;
}

void VoxelTemplateMatcher::setDefaultTemplates() {
  // These are the templates from "Improved 3D Thinning Algorithms for
  // Skeleton Extraction" by She 2009.
  // Template A.
  addIntegerTemplate(1904135, 65536);
  addIntegerTemplate(4194815, 4194304);
  addIntegerTemplate(19190345, 16384);
  addIntegerTemplate(76699940, 4096);
  addIntegerTemplate(117671360, 1024);
  addIntegerTemplate(133955600, 16);

  // Template B.
  addIntegerTemplate(2971147, 81920);
  addIntegerTemplate(4248283, 4210688);
  addIntegerTemplate(4263487, 4259840);
  addIntegerTemplate(4348342, 4198400);
  addIntegerTemplate(4425208, 4195328);
  addIntegerTemplate(10050598, 69632);
  addIntegerTemplate(16584208, 65552);
  addIntegerTemplate(52548808, 17408);
  addIntegerTemplate(57463312, 16400);
  addIntegerTemplate(109270432, 5120);
  addIntegerTemplate(114972688, 4112);
  addIntegerTemplate(132350992, 1040);

  // Template C.
  addIntegerTemplate(7165456, 81936);
  addIntegerTemplate(4314328, 4211712);
  addIntegerTemplate(4283446, 4263936);
  addIntegerTemplate(4281883, 4276224);
  addIntegerTemplate(4412848, 4199424);
  addIntegerTemplate(14244880, 69648);
  addIntegerTemplate(56742928, 17424);
  addIntegerTemplate(113464336, 5136);

  // Template D.
  addIntegerTemplate(253440, 512);
  addIntegerTemplate(253440, 2048);
  addIntegerTemplate(253440, 32768);
  addIntegerTemplate(253440, 131072);
  addIntegerTemplate(14700600, 8);
  addIntegerTemplate(14700600, 32);
  addIntegerTemplate(14700600, 2097152);
  addIntegerTemplate(14700600, 8388608);
  addIntegerTemplate(38339730, 2);
  addIntegerTemplate(38339730, 128);
  addIntegerTemplate(38339730, 524288);
  addIntegerTemplate(38339730, 33554432);
}

}  // namespace voxblox

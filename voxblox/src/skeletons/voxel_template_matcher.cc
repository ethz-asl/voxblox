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
  templates_.push_back(voxel_template);
}

bool VoxelTemplateMatcher::fitsTemplates(
    const std::bitset<27>& voxel_neighbors) const {
  for (const VoxelTemplate& temp : templates_) {
    if (((voxel_neighbors ^ temp.neighbor_template) & temp.neighbor_mask)
            .none()) {
      return true;
    }
  }
  return false;
}

void VoxelTemplateMatcher::setDeletionTemplates() {
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

void VoxelTemplateMatcher::setConnectivityTemplates() {
  // Template G, for end-points
  addIntegerTemplate(4281360, 16);
  addIntegerTemplate(4281360, 1024);
  addIntegerTemplate(4281360, 4096);
  addIntegerTemplate(4281360, 16384);
  addIntegerTemplate(4281360, 65536);
  addIntegerTemplate(4281360, 4194304);

  // Template H, possibly for matching straight lines.
  addIntegerTemplate(87226, 16);
  addIntegerTemplate(4742674, 1024);
  addIntegerTemplate(6395416, 4096);
  addIntegerTemplate(12799024, 16384);
  addIntegerTemplate(37998736, 65536);
  addIntegerTemplate(48845824, 4194304);
}

void VoxelTemplateMatcher::setCornerTemplates() {
  // New templates:
  addIntegerTemplate(20766719, 16384);
  addIntegerTemplate(20815871, 65536);
  addIntegerTemplate(24944639, 4194304);
  addIntegerTemplate(77488127, 4096);
  addIntegerTemplate(120050687, 16384);
  addIntegerTemplate(127127039, 1024);
  addIntegerTemplate(127130111, 4096);
  addIntegerTemplate(124228607, 4194304);
  addIntegerTemplate(134012495, 16384);
  addIntegerTemplate(134106935, 16);
  addIntegerTemplate(134111015, 4096);
  addIntegerTemplate(134061647, 65536);
  addIntegerTemplate(134190041, 16);
  addIntegerTemplate(134191049, 1024);
  addIntegerTemplate(134203892, 16);
  addIntegerTemplate(134204900, 1024);
  addIntegerTemplate(134207972, 4096);
  addIntegerTemplate(134206409, 16384);
  addIntegerTemplate(134172455, 65536);
  addIntegerTemplate(131320319, 4194304);
}

std::bitset<27> VoxelTemplateMatcher::get6ConnNeighborMask() const {
  std::bitset<27> conn_mask(4281360);
  return conn_mask;
}

std::bitset<27> VoxelTemplateMatcher::get18ConnNeighborMask() const {
  std::bitset<27> conn_mask(49012410);
  return conn_mask;
}

}  // namespace voxblox

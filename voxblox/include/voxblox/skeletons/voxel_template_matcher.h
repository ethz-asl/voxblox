#ifndef VOXBLOX_SKELETONS_VOXEL_TEMPLATE_MATCHER_H_
#define VOXBLOX_SKELETONS_VOXEL_TEMPLATE_MATCHER_H_

#include <bitset>
#include <vector>

#include "voxblox/core/common.h"

namespace voxblox {

// Always enforces 26-connectivity; to use lesser connectivities, just set
// the mask appropriately.
struct VoxelTemplate {
  std::bitset<27> neighbor_mask;
  std::bitset<27> neighbor_template;
};

// Does binary matching against a 3D (3x3x3) voxel template. The template has
// 2 parts: an attention mask and the actual template values.
class VoxelTemplateMatcher {
 public:
  VoxelTemplateMatcher();

  void addTemplate(const VoxelTemplate& voxel_template);
  // Converts the template to a bitmask.
  void addIntegerTemplate(int32_t neighbor_mask_dec,
                          int32_t neighbor_template_dec);

  // Returns true if ANY template is matched.
  bool fitsTemplates(const std::bitset<27>& voxel_neighbors);


  void setDefaultTemplates();


 private:
  std::vector<VoxelTemplate> templates_;
};

}  // namespace voxblox

#endif  // VOXBLOX_SKELETONS_VOXEL_TEMPLATE_MATCHER_H_

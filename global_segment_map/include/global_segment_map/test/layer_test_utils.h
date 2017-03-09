#ifndef GLOBAL_SEGMENT_MAP_LAYER_TEST_UTILS_H_
#define GLOBAL_SEGMENT_MAP_LAYER_TEST_UTILS_H_

#include <gtest/gtest.h>

#include <voxblox/test/layer_test_utils.h>

namespace voxblox {
namespace test {

template <>
void LayerTest<LabelVoxel>::CompareVoxel(const LabelVoxel& voxel_A,
                                         const LabelVoxel& voxel_B) const {
  CHECK_EQ(voxel_A.label, voxel_B.label);
  CHECK_EQ(voxel_A.label_confidence, voxel_B.label_confidence);
}

}  // namespace test
}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_LAYER_TEST_UTILS_H_

#include "voxblox/test/layer_test_utils.h"

namespace voxblox {
namespace test {

template <>
void fillVoxelWithTestData(size_t x, size_t y, size_t z, TsdfVoxel* voxel) {
  CHECK_NOTNULL(voxel);
  voxel->distance = x * y * 0.66 + z;
  voxel->weight = y * z * 0.33 + x;
  voxel->color.r = static_cast<uint8_t>(x % 255);
  voxel->color.g = static_cast<uint8_t>(y % 255);
  voxel->color.b = static_cast<uint8_t>(z % 255);
  voxel->color.a = static_cast<uint8_t>(x + y % 255);
}

template <>
void fillVoxelWithTestData(size_t x, size_t y, size_t z, EsdfVoxel* voxel) {
  CHECK_NOTNULL(voxel);
  voxel->distance = x * y * 0.66 + z;
  voxel->parent.x() = x % 255;
  voxel->parent.y() = y % 255;
  voxel->parent.z() = z % 255;

  voxel->observed = true;
  voxel->in_queue = true;
  voxel->fixed = true;
}

template <>
void fillVoxelWithTestData(size_t x, size_t y, size_t z,
                           OccupancyVoxel* voxel) {
  CHECK_NOTNULL(voxel);
  voxel->probability_log = x * y * 0.66 + z;
  voxel->observed = true;
}

}  // namespace test
}  // namespace voxblox

#ifndef VOXBLOX_TEST_LAYER_TEST_UTILS_H_
#define VOXBLOX_TEST_LAYER_TEST_UTILS_H_

#include <gtest/gtest.h>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

namespace test {

/**
 * Holds functions for comparing layers, blocks, voxels, etc for testing
 */
template <typename VoxelType>
class LayerTest {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void CompareLayers(const Layer<VoxelType>& layer_A,
                     const Layer<VoxelType>& layer_B) const {
    EXPECT_NEAR(layer_A.voxel_size(), layer_B.voxel_size(), kTolerance);
    EXPECT_NEAR(layer_A.block_size(), layer_B.block_size(), kTolerance);
    EXPECT_EQ(layer_A.voxels_per_side(), layer_B.voxels_per_side());
    EXPECT_EQ(layer_A.getNumberOfAllocatedBlocks(),
              layer_B.getNumberOfAllocatedBlocks());

    BlockIndexList blocks_A, blocks_B;
    layer_A.getAllAllocatedBlocks(&blocks_A);
    layer_B.getAllAllocatedBlocks(&blocks_B);
    EXPECT_EQ(blocks_A.size(), blocks_B.size());
    for (const BlockIndex& index_A : blocks_A) {
      BlockIndexList::const_iterator it =
          std::find(blocks_B.begin(), blocks_B.end(), index_A);
      if (it != blocks_B.end()) {
        const Block<VoxelType>& block_A = layer_A.getBlockByIndex(index_A);
        const Block<VoxelType>& block_B = layer_B.getBlockByIndex(*it);
        CompareBlocks(block_A, block_B);
      } else {
        ADD_FAILURE();
        LOG(ERROR) << "Block at index [" << index_A.transpose()
                   << "] in layer_A does not exists in layer_B";
      }
    }
    for (const BlockIndex& index_B : blocks_B) {
      BlockIndexList::const_iterator it =
          std::find(blocks_A.begin(), blocks_A.end(), index_B);
      if (it != blocks_A.end()) {
        const Block<VoxelType>& block_B = layer_A.getBlockByIndex(index_B);
        const Block<VoxelType>& block_A = layer_B.getBlockByIndex(*it);
        CompareBlocks(block_B, block_A);
      } else {
        ADD_FAILURE();
        LOG(ERROR) << "Block at index [" << index_B.transpose()
                   << "] in layer_B does not exists in layer_A";
      }
    }

    EXPECT_EQ(layer_A.getMemorySize(), layer_B.getMemorySize());
  }

  void CompareBlocks(const Block<VoxelType>& block_A,
                     const Block<VoxelType>& block_B) const {
    EXPECT_NEAR(block_A.voxel_size(), block_B.voxel_size(), kTolerance);
    EXPECT_NEAR(block_A.block_size(), block_B.block_size(), kTolerance);
    EXPECT_EQ(block_A.voxels_per_side(), block_B.voxels_per_side());

    EXPECT_NEAR(block_A.origin().x(), block_B.origin().x(), kTolerance);
    EXPECT_NEAR(block_A.origin().y(), block_B.origin().y(), kTolerance);
    EXPECT_NEAR(block_A.origin().z(), block_B.origin().z(), kTolerance);

    EXPECT_EQ(block_A.num_voxels(), block_B.num_voxels());
    for (size_t voxel_idx = 0u; voxel_idx < block_A.num_voxels(); ++voxel_idx) {
      CompareVoxel(block_A.getVoxelByLinearIndex(voxel_idx),
                   block_B.getVoxelByLinearIndex(voxel_idx));
    }
  }

  void CompareVoxel(const VoxelType& voxel_A, const VoxelType& voxel_B) const;

  static constexpr double kTolerance = 1e-10;
};

template <typename VoxelType>
void LayerTest<VoxelType>::CompareVoxel(const VoxelType& /* voxel_A */,
                                        const VoxelType& /* voxel_B */) const {
  LOG(FATAL) << "Not implemented for this voxel type!";
}

template <>
void LayerTest<EsdfVoxel>::CompareVoxel(const EsdfVoxel& voxel_A,
                                        const EsdfVoxel& voxel_B) const {
  constexpr double kTolerance = 1e-10;

  EXPECT_NEAR(voxel_A.distance, voxel_B.distance, kTolerance);

  // Flags
  EXPECT_EQ(voxel_A.observed, voxel_B.observed);
  EXPECT_EQ(voxel_A.hallucinated, voxel_B.hallucinated);
  EXPECT_EQ(voxel_A.in_queue, voxel_B.in_queue);
  EXPECT_EQ(voxel_A.fixed, voxel_B.fixed);

  EXPECT_EQ(voxel_A.parent.x(), voxel_B.parent.x());
  EXPECT_EQ(voxel_A.parent.y(), voxel_B.parent.y());
  EXPECT_EQ(voxel_A.parent.z(), voxel_B.parent.z());
}

template <>
void LayerTest<OccupancyVoxel>::CompareVoxel(
    const OccupancyVoxel& voxel_A, const OccupancyVoxel& voxel_B) const {
  constexpr double kTolerance = 1e-10;

  EXPECT_NEAR(voxel_A.probability_log, voxel_B.probability_log, kTolerance);
  EXPECT_EQ(voxel_A.observed, voxel_B.observed);
}

template <>
void LayerTest<TsdfVoxel>::CompareVoxel(const TsdfVoxel& voxel_A,
                                        const TsdfVoxel& voxel_B) const {
  EXPECT_NEAR(voxel_A.distance, voxel_B.distance, kTolerance);
  EXPECT_NEAR(voxel_A.weight, voxel_B.weight, kTolerance);
  EXPECT_EQ(voxel_A.color.r, voxel_B.color.r);
  EXPECT_EQ(voxel_A.color.g, voxel_B.color.g);
  EXPECT_EQ(voxel_A.color.b, voxel_B.color.b);
  EXPECT_EQ(voxel_A.color.a, voxel_B.color.a);
}

template <>
void LayerTest<IntensityVoxel>::CompareVoxel(
    const IntensityVoxel& voxel_A, const IntensityVoxel& voxel_B) const {
  EXPECT_NEAR(voxel_A.intensity, voxel_B.intensity, kTolerance);
  EXPECT_NEAR(voxel_A.weight, voxel_B.weight, kTolerance);
}

template <typename VoxelType>
void fillVoxelWithTestData(size_t x, size_t y, size_t z, VoxelType* voxel);

template <typename VoxelType>
void SetUpTestLayer(const IndexElement block_volume_diameter,
                    const IndexElement block_volume_offset,
                    Layer<VoxelType>* layer) {
  CHECK_NOTNULL(layer);

  const IndexElement half_idx_range = block_volume_diameter / 2;

  // For better readability.
  const IndexElement& min_idx = -half_idx_range + block_volume_offset;
  const IndexElement& max_idx = half_idx_range + block_volume_offset;

  for (IndexElement x = min_idx; x <= max_idx; ++x) {
    for (IndexElement y = min_idx; y <= max_idx; ++y) {
      for (IndexElement z = min_idx; z <= max_idx; ++z) {
        BlockIndex block_idx = {x, y, z};
        typename Block<VoxelType>::Ptr block =
            layer->allocateBlockPtrByIndex(block_idx);
        VoxelType& voxel = block->getVoxelByLinearIndex(
            (x * z + y) % layer->voxels_per_side());

        fillVoxelWithTestData(x, y, z, &voxel);

        block->has_data() = true;
      }
    }
  }
  const double size_in_MB = static_cast<double>(layer->getMemorySize()) * 1e-6;
  std::cout << std::endl
            << "Set up a test layer of size " << size_in_MB << " MB";
}

template <typename VoxelType>
void SetUpTestLayer(const IndexElement block_volume_diameter,
                    Layer<VoxelType>* layer) {
  constexpr IndexElement kBlockVolumeOffset = 0u;
  SetUpTestLayer(block_volume_diameter, kBlockVolumeOffset, layer);
}

template <>
inline void fillVoxelWithTestData(size_t x, size_t y, size_t z,
                                  TsdfVoxel* voxel) {
  CHECK_NOTNULL(voxel);
  voxel->distance = x * y * 0.66 + z;
  voxel->weight = y * z * 0.33 + x;
  voxel->color.r = static_cast<uint8_t>(x % 255);
  voxel->color.g = static_cast<uint8_t>(y % 255);
  voxel->color.b = static_cast<uint8_t>(z % 255);
  voxel->color.a = static_cast<uint8_t>(x + y % 255);
}

template <>
inline void fillVoxelWithTestData(size_t x, size_t y, size_t z,
                                  EsdfVoxel* voxel) {
  CHECK_NOTNULL(voxel);
  voxel->distance = x * y * 0.66 + z;
  voxel->parent.x() = x % INT8_MAX;
  voxel->parent.y() = y % INT8_MAX;
  voxel->parent.z() = z % INT8_MAX;

  voxel->observed = true;
  voxel->in_queue = true;
  voxel->fixed = true;
}

template <>
inline void fillVoxelWithTestData(size_t x, size_t y, size_t z,
                                  OccupancyVoxel* voxel) {
  CHECK_NOTNULL(voxel);
  voxel->probability_log = x * y * 0.66 + z;
  voxel->observed = true;
}

template <>
inline void fillVoxelWithTestData(size_t x, size_t y, size_t z,
                                  IntensityVoxel* voxel) {
  CHECK_NOTNULL(voxel);
  voxel->intensity = x * y * 0.66 + z;
  voxel->weight = y * z * 0.33 + x;
}

}  // namespace test
}  // namespace voxblox

#endif  // VOXBLOX_TEST_LAYER_TEST_UTILS_H_

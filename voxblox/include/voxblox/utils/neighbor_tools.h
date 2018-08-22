#ifndef VOXBLOX_UTILS_NEIGHBOR_TOOLS_H_
#define VOXBLOX_UTILS_NEIGHBOR_TOOLS_H_

#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"

namespace voxblox {

// Define what connectivity you want to have in the voxels.
enum Connectivity : unsigned int {
  kSix = 6u,
  kEighteen = 18u,
  kTwentySix = 26u
};

template <typename VoxelType>
class NeighborTools {
 public:
  NeighborTools() : voxels_per_side_(0) {}
  explicit NeighborTools(const Layer<VoxelType>* layer) : layer_(layer) {
    CHECK_NOTNULL(layer);
    voxels_per_side_ = layer_->voxels_per_side();
    CHECK_NE(voxels_per_side_, 0u);
  }

  void setLayer(const Layer<VoxelType>* layer) {
    CHECK_NOTNULL(layer);
    layer_ = layer;
    voxels_per_side_ = layer_->voxels_per_side();
    CHECK_NE(voxels_per_side_, 0u);
  }

  void getNeighborIndexesAndDistances(
      const BlockIndex& block_index, const VoxelIndex& voxel_index,
      Connectivity connectivity, AlignedVector<VoxelKey>* neighbors,
      AlignedVector<float>* distances,
      AlignedVector<Eigen::Vector3i>* directions) const;

  void getNeighborIndex(const BlockIndex& block_index,
                        const VoxelIndex& voxel_index,
                        const SignedIndex& direction,
                        BlockIndex* neighbor_block_index,
                        VoxelIndex* neighbor_voxel_index) const;

  SignedIndex getOffsetBetweenVoxels(const BlockIndex& start_block_index,
                                     const VoxelIndex& start_voxel_index,
                                     const BlockIndex& end_block_index,
                                     const VoxelIndex& end_voxel_index) const;

 private:
  const Layer<VoxelType>* layer_;

  size_t voxels_per_side_;
};

// Uses specified connectivity and quasi-Euclidean distances.
// Directions is the direction that the neighbor voxel lives in. If you
// need the direction FROM the neighbor voxel TO the current voxel, take
// negative of the given direction.
template <typename VoxelType>
void NeighborTools<VoxelType>::getNeighborIndexesAndDistances(
    const BlockIndex& block_index, const VoxelIndex& voxel_index,
    Connectivity connectivity, AlignedVector<VoxelKey>* neighbors,
    AlignedVector<float>* distances,
    AlignedVector<SignedIndex>* directions) const {
  CHECK_NOTNULL(layer_);
  CHECK_NOTNULL(neighbors);
  CHECK_NOTNULL(distances);
  CHECK_NOTNULL(directions);

  static const float kSqrt2 = std::sqrt(2);
  static const float kSqrt3 = std::sqrt(3);

  neighbors->reserve(connectivity);
  distances->reserve(connectivity);
  directions->reserve(connectivity);

  VoxelKey neighbor;
  Eigen::Vector3i direction;
  direction.setZero();
  // Distance 1 set.
  for (unsigned int i = 0u; i < 3u; ++i) {
    for (int j = -1; j <= 1; j += 2) {
      direction(i) = j;
      getNeighborIndex(block_index, voxel_index, direction, &neighbor.first,
                       &neighbor.second);
      neighbors->emplace_back(neighbor);
      distances->emplace_back(1.0);
      directions->emplace_back(direction);
    }
    direction(i) = 0;
  }

  if (connectivity == Connectivity::kSix) {
    return;
  }

  // Distance sqrt(2) set.
  for (unsigned int i = 0u; i < 3u; ++i) {
    unsigned int next_i = (i + 1) % 3;
    for (int j = -1; j <= 1; j += 2) {
      direction(i) = j;
      for (int k = -1; k <= 1; k += 2) {
        direction(next_i) = k;
        getNeighborIndex(block_index, voxel_index, direction, &neighbor.first,
                         &neighbor.second);
        neighbors->emplace_back(neighbor);
        distances->emplace_back(kSqrt2);
        directions->emplace_back(direction);
      }
      direction(i) = 0;
      direction(next_i) = 0;
    }
  }

  if (connectivity == Connectivity::kEighteen) {
    return;
  }

  // Distance sqrt(3) set.
  for (int i = -1; i <= 1; i += 2) {
    direction(0) = i;
    for (int j = -1; j <= 1; j += 2) {
      direction(1) = j;
      for (int k = -1; k <= 1; k += 2) {
        direction(2) = k;
        getNeighborIndex(block_index, voxel_index, direction, &neighbor.first,
                         &neighbor.second);
        neighbors->emplace_back(neighbor);
        distances->emplace_back(kSqrt3);
        directions->emplace_back(direction);
      }
    }
  }

  if (connectivity != Connectivity::kTwentySix) {
    LOG(FATAL) << "Unknown neighborhood connectivity: "
               << static_cast<int>(connectivity);
  }
}

template <typename VoxelType>
void NeighborTools<VoxelType>::getNeighborIndex(
    const BlockIndex& block_index, const VoxelIndex& voxel_index,
    const SignedIndex& direction, BlockIndex* neighbor_block_index,
    VoxelIndex* neighbor_voxel_index) const {
  CHECK_NE(voxels_per_side_, 0u);
  CHECK_NOTNULL(neighbor_block_index);
  CHECK_NOTNULL(neighbor_voxel_index);

  *neighbor_block_index = block_index;
  *neighbor_voxel_index = voxel_index + direction;

  for (unsigned int i = 0; i < 3; ++i) {
    while ((*neighbor_voxel_index)(i) < 0) {
      (*neighbor_block_index)(i)--;
      (*neighbor_voxel_index)(i) += voxels_per_side_;
    }
    while ((*neighbor_voxel_index)(i) >=
           static_cast<IndexElement>(voxels_per_side_)) {
      (*neighbor_block_index)(i)++;
      (*neighbor_voxel_index)(i) -= voxels_per_side_;
    }
  }
}

template <typename VoxelType>
SignedIndex NeighborTools<VoxelType>::getOffsetBetweenVoxels(
    const BlockIndex& start_block_index, const VoxelIndex& start_voxel_index,
    const BlockIndex& end_block_index,
    const VoxelIndex& end_voxel_index) const {
  CHECK_NE(voxels_per_side_, 0u);
  BlockIndex current_block_index = end_block_index;
  SignedIndex voxel_offset = SignedIndex::Zero();

  // Line up the voxels so that they're in the same block.
  for (unsigned int i = 0u; i < 3u; ++i) {
    while (start_block_index(i) > current_block_index(i)) {
      current_block_index(i)++;
      voxel_offset(i) -= voxels_per_side_;
    }
    while (start_block_index(i) < current_block_index(i)) {
      current_block_index(i)--;
      voxel_offset(i) += voxels_per_side_;
    }

    // Then get the voxel distance, since they're in the same block.
    // This can be negative, but that's totally fine.
    voxel_offset(i) += end_voxel_index(i) - start_voxel_index(i);
  }
  return voxel_offset;
}

}  // namespace voxblox

#endif  // VOXBLOX_UTILS_NEIGHBOR_TOOLS_H_

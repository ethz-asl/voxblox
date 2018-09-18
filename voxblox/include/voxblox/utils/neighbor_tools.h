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

class NeighborhoodLookupTables {
 public:
  typedef Eigen::Matrix<LongIndexElement, 3, Connectivity::kTwentySix>
      LongIndexOffsets;
  typedef Eigen::Matrix<IndexElement, 3, Connectivity::kTwentySix> IndexOffsets;
  typedef Eigen::Matrix<float, 1, Connectivity::kTwentySix> Distances;

  // Stores the distances to the 6, 18, and 26 neighborhood, in that order.
  // These distances need to be scaled by the voxel distance to get metric
  // distances.
  static const Distances kDistances;

  // Lookup table for the offsets between a index and its 6, 18, and 26
  // neighborhood, in that order. These two offset tables are the same except
  // for the type, this saves casting the offset when used with either global
  // index (long) or local index (int) in the neighborhood lookup.
  static const IndexOffsets kOffsets;
  static const LongIndexOffsets kLongOffsets;
};

template <Connectivity kConnectivity>
class Neighborhood : public NeighborhoodLookupTables {
 public:
  typedef Eigen::Matrix<LongIndexElement, 3, kConnectivity> IndexMatrix;

  // Get the global index of all (6, 18, or 26) neighbors of the input index.
  static void getFromGlobalIndex(const GlobalIndex& global_index,
                                 IndexMatrix* neighbors) {
    CHECK_NOTNULL(neighbors);
    for (unsigned int i = 0u; i < kConnectivity; ++i) {
      neighbors->col(i) = global_index + kLongOffsets.col(i);
    }
  }

  // Get the hierarchical index (block idx, local voxel index) a voxel described
  // by its hierarchical index and a direction. The main purpose of this
  // function is to solve the cross-block indexing that happens when looking up
  // neighbors at the block boundaries.
  static void getFromHierarchicalIndexAndDirection(
      const BlockIndex& block_index, const VoxelIndex& voxel_index,
      const SignedIndex& direction, const size_t voxels_per_side,
      BlockIndex* neighbor_block_index, VoxelIndex* neighbor_voxel_index) {
    CHECK_GT(voxels_per_side, 0u);
    CHECK_NOTNULL(neighbor_block_index);
    CHECK_NOTNULL(neighbor_voxel_index);

    *neighbor_block_index = block_index;
    *neighbor_voxel_index = voxel_index + direction;

    for (unsigned int i = 0u; i < 3u; ++i) {
      while ((*neighbor_voxel_index)(i) < 0) {
        (*neighbor_block_index)(i)--;
        (*neighbor_voxel_index)(i) += voxels_per_side;
      }
      while ((*neighbor_voxel_index)(i) >=
             static_cast<IndexElement>(voxels_per_side)) {
        (*neighbor_block_index)(i)++;
        (*neighbor_voxel_index)(i) -= voxels_per_side;
      }
    }
  }

  // Get the hierarchical indices (block idx, local voxel index) for all
  // neighbors (6, 18, or 26 neighborhood) of a hierarcical index. This function
  // solves the cross-block indexing that happens when looking up neighbors at
  // the block boundary.
  static void getFromHierarchicalIndex(const BlockIndex& block_index,
                                       const VoxelIndex& voxel_index,
                                       const size_t voxels_per_side,
                                       AlignedVector<VoxelKey>* neighbors_ptr) {
    CHECK_NOTNULL(neighbors_ptr)->resize(kConnectivity);

    AlignedVector<VoxelKey>& neighbors = *neighbors_ptr;
    for (unsigned int i = 0u; i < kConnectivity; ++i) {
      VoxelKey& neighbor = neighbors[i];
      getFromHierarchicalIndexAndDirection(block_index, voxel_index,
                                           kOffsets.col(i), voxels_per_side,
                                           &neighbor.first, &neighbor.second);
    }
  }
};
}  // namespace voxblox

#endif  // VOXBLOX_UTILS_NEIGHBOR_TOOLS_H_

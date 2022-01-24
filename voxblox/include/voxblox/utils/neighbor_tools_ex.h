#ifndef VOXBLOX_UTILS_NEIGHBOR_TOOLS_EX_H_
#define VOXBLOX_UTILS_NEIGHBOR_TOOLS_EX_H_

#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"

namespace voxblox {

// 24 Neighborhood, mainly used for FIESTA ESDF mapping
class Neighborhood24LookupTables {
 public:
  typedef Eigen::Matrix<LongIndexElement, 3, 24> LongIndexOffsets;
  typedef Eigen::Matrix<IndexElement, 3, 24> IndexOffsets;
  typedef Eigen::Matrix<float, 1, 24> Distances;

  /*
   * Stores the distances to the 24 neighborhood, in that order.
   * These distances need to be scaled by the voxel distance to get metric
   * distances.
   */
  static const Distances kDistances;

  /*
   * Lookup table for the offsets between a index and its 24
   * neighborhood, in that order. These two offset tables are the same except
   * for the type, this saves casting the offset when used with either global
   * index (long) or local index (int) in the neighborhood lookup.
   */
  static const IndexOffsets kOffsets;
  static const LongIndexOffsets kLongOffsets;
};

class Neighborhood24 : public Neighborhood24LookupTables {
 public:
  typedef Eigen::Matrix<LongIndexElement, 3, 24> IndexMatrix;

  /// Get the global index of all 24 neighbors of the input index.
  static void getFromGlobalIndex(const GlobalIndex& global_index,
                                 IndexMatrix* neighbors) {
    CHECK_NOTNULL(neighbors);
    for (unsigned int i = 0u; i < 24; ++i) {
      neighbors->col(i) = global_index + kLongOffsets.col(i);
    }
  }

  
  static void getFromGlobalIndexAndObstacle(const GlobalIndex& global_index, 
                                            const GlobalIndex& coc_index,
                                            std::vector<int> &neighbors_idx) {
    
  // -1,  1,  0,  0,  0,  0, -1,  1,  0,  0, -1,  1, -1,  1,  0,  0,  1, -1, -2,  2,  0,  0,  0,  0, // dx
  //  0,  0, -1,  1,  0,  0, -1,  1, -1,  1,  0,  0,  1, -1, -1,  1,  0,  0,  0,  0, -2,  2,  0,  0, // dy
  //  0,  0,  0,  0, -1,  1,  0,  0, -1,  1, -1,  1,  0,  0,  1, -1, -1,  1,  0,  0,  0,  0, -2,  2; // dz
  //  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23; // index

    if (coc_index(2) <= global_index(2)){ // z >= obstz 
      neighbors_idx.push_back(5);
      neighbors_idx.push_back(23);
    }
    if (coc_index(2) >= global_index(2)){ // z <= obstz 
      neighbors_idx.push_back(4);
      neighbors_idx.push_back(22);
    }
    
    if(coc_index(1) <= global_index(1)){ // y >= obsty
      neighbors_idx.push_back(3);
      neighbors_idx.push_back(21);
      if (coc_index(2) <= global_index(2))
        neighbors_idx.push_back(9);
      if (coc_index(2) >= global_index(2))
        neighbors_idx.push_back(15);
    }

    if(coc_index(1) >= global_index(1)){ // y <= obsty
      neighbors_idx.push_back(2);
      neighbors_idx.push_back(20);
      if (coc_index(2) <= global_index(2))
        neighbors_idx.push_back(14);
      if (coc_index(2) >= global_index(2))
        neighbors_idx.push_back(8);
    }

    if(coc_index(0) <= global_index(0)){ // x >= obstx
      neighbors_idx.push_back(1);
      neighbors_idx.push_back(19);
      if (coc_index(2) <= global_index(2))
        neighbors_idx.push_back(11);
      if (coc_index(2) >= global_index(2))
        neighbors_idx.push_back(16);
      if(coc_index(1) >= global_index(1))
        neighbors_idx.push_back(7);
      if(coc_index(1) >= global_index(1))
        neighbors_idx.push_back(13);
    }

    if(coc_index(0) >= global_index(0)){ // x <= obstx
      neighbors_idx.push_back(0);
      neighbors_idx.push_back(18);
      if (coc_index(2) <= global_index(2))
        neighbors_idx.push_back(17);
      if (coc_index(2) >= global_index(2))
        neighbors_idx.push_back(10);
      if(coc_index(1) >= global_index(1))
        neighbors_idx.push_back(12);
      if(coc_index(1) >= global_index(1))
        neighbors_idx.push_back(6);
    }
  }


  /**
   * Get the block idx and local voxel index of a neighbor voxel. The neighbor
   * voxel is defined by providing a direction. The main purpose of this
   * function is to solve the cross-block indexing that happens when looking up
   * neighbors at the block boundaries.
   */
  static void getFromBlockAndVoxelIndexAndDirection(
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

  /**
   * Get the hierarchical indices (block idx, local voxel index) for all 24
   * neighbors of a hierarcical index. This function
   * solves the cross-block indexing that happens when looking up neighbors at
   * the block boundary.
   */
  static void getFromBlockAndVoxelIndex(
      const BlockIndex& block_index, const VoxelIndex& voxel_index,
      const size_t voxels_per_side, AlignedVector<VoxelKey>* neighbors_ptr) {
    CHECK_NOTNULL(neighbors_ptr)->resize(24);

    AlignedVector<VoxelKey>& neighbors = *neighbors_ptr;
    for (unsigned int i = 0u; i < 24; ++i) {
      VoxelKey& neighbor = neighbors[i];
      getFromBlockAndVoxelIndexAndDirection(block_index, voxel_index,
                                            kOffsets.col(i), voxels_per_side,
                                            &neighbor.first, &neighbor.second);
    }
  }

  /// Get the signed offset between the global indices of two voxels.
  static SignedIndex getOffsetBetweenVoxels(const BlockIndex& start_block_index,
                                            const VoxelIndex& start_voxel_index,
                                            const BlockIndex& end_block_index,
                                            const VoxelIndex& end_voxel_index,
                                            const size_t voxels_per_side) {
    CHECK_NE(voxels_per_side, 0u);
    return (end_voxel_index - start_voxel_index) +
           (end_block_index - start_block_index) * voxels_per_side;
  }
};

// TODO: find the way to extend it to something simialr to 6, 18, 26-nei
}  // namespace voxblox

#endif  // VOXBLOX_UTILS_NEIGHBOR_TOOLS_EX_H_

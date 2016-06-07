#ifndef VOXBLOX_INTEGRATOR_ESDF_INTEGRATOR_H_
#define VOXBLOX_INTEGRATOR_ESDF_INTEGRATOR_H_

#include <algorithm>
#include <vector>
#include <queue>
#include <utility>

#include <Eigen/Core>
#include <glog/logging.h>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/utils/timing.h"

namespace voxblox {

class EsdfIntegrator {
 public:
  struct Config {
    FloatingPoint max_distance_m = 1.5;
  };

  EsdfIntegrator(const Config& config, Layer<TsdfVoxel>* tsdf_layer,
                 Layer<EsdfVoxel>* esdf_layer)
      : config_(config), tsdf_layer_(tsdf_layer), esdf_layer_(esdf_layer) {
    CHECK(tsdf_layer_);
    CHECK(esdf_layer_);

    /*voxel_size_ = layer_->voxel_size();
    block_size_ = layer_->block_size();
    voxels_per_side_ = layer_->voxels_per_side();

    voxel_size_inv_ = 1.0 / voxel_size_;
    block_size_inv_ = 1.0 / block_size_;
    voxels_per_side_inv_ = 1.0 / voxels_per_side_; */
  }

  void updateFromTsdfLayer() {}

  void updateFromTsdfVoxels() {
    // Get a specific list of voxels in the TSDF layer, and propagate out from
    // there.
  }

  // Uses 26-connectivity and quasi-Euclidean distances.
  void getNeighborsAndDistances(
      const BlockIndex& block_index, const VoxelIndex& voxel_index,
      std::vector<std::pair<BlockIndex, VoxelIndex> >* neighbors,
      std::vector<float>* distances) {
    static const double kSqrt2 = std::sqrt(2);
    static const double kSqrt3 = std::sqrt(3);
    static const size_t kNumNeighbors = 26;

    neighbors->reserve(kNumNeighbors);
    distances->reserve(kNumNeighbors);

    std::pair<BlockIndex, VoxelIndex> neighbor;
    Eigen::Vector3i direction;
    direction.setZero();
    // Distance 1 set.
    for (unsigned int i = 0; i < 3; ++i) {
      for (int j = -1; j <= 1; j += 2) {
        direction(i) = j;
        getNeighbor(block_index, voxel_index, direction, &neighbor.first,
                    &neighbor.second);
        neighbors->emplace_back(neighbor);
        distances->emplace_back(1.0);
      }
      direction(i) = 0;
    }

    // Distance sqrt(2) set.
    for (unsigned int i = 0; i < 3; ++i) {
      next_i = (i + 1) % 3;
      for (int j = -1; j <= 1; j += 2) {
        direction(i) = j;
        for (int k = -1; k <= 1; k += 2) {
          direction(next_i) = k;
          getNeighbor(block_index, voxel_index, direction, &neighbor.first,
                      &neighbor.second);
          neighbors->emplace_back(neighbor);
          distances->emplace_back(kSqrt2);
        }
        direction(i) = 0;
        direction(next_i) = 0;
      }
    }

    // Distance sqrt(3) set.
    for (int i = -1; i <= 1; i += 2) {
      direction(0) = i;
      for (int j = -1; j <= 1; j += 2) {
        direction(1) = j;
        for (int k = -1; k <= 1; k += 2) {
          direction(2) = k;
          getNeighbor(block_index, voxel_index, direction, &neighbor.first,
                      &neighbor.second);
          neighbors->emplace_back(neighbor);
          distances->emplace_back(kSqrt3);
        }
      }
    }
  }

  void getNeighbor(const BlockIndex& block_index, const VoxelIndex& voxel_index,
                   const Eigen::Vector3i& direction,
                   BlockIndex* neighbor_block_index,
                   VoxelIndex* neighbor_voxel_index) {
    DCHECK_NOTNULL(neighbor_block_index);
    DCHECK_NOTNULL(neighbor_voxel_index);

    *neighbor_block_index = block_index;
    *neighbor_voxel_index += direction;

    for (unsigned int i = 0; i < 3; ++i) {
      if (*neighbor_voxel_index(i) < 0) {
        *neighbor_block_index(i)--;
        *neighbor_voxel_index += esdf_voxels_per_side_;
      } else if (*neighbor_voxel_index(i) >= esdf_voxels_per_side_) {
        *neighbor_block_index(i)++;
        *neighbor_voxel_index -= esdf_voxels_per_side_;
      }
    }
  }

 protected:
  Config config_;

  Layer<TsdfVoxel>* tsdf_layer_;
  Layer<EsdfVoxel>* esdf_layer_;

  // Open Queue for incremental updates. Contains global voxel indices
  // for the ESDF layer.
  std::queue<std::pair<BlockIndex, VoxelIndex> > open_;

  size_t esdf_voxels_per_side_;

  /*
  // Cached map config.
  FloatingPoint voxel_size_;
  FloatingPoint block_size_;

  // Derived types.
  FloatingPoint voxel_size_inv_;
  FloatingPoint voxels_per_side_inv_;
  FloatingPoint block_size_inv_; */
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_ESDF_INTEGRATOR_H_

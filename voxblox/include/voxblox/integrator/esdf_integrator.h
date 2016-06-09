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
    // Should mirror (or be smaller than) truncation distance in tsdf
    // integrator.
    FloatingPoint min_distance_m = 0.2;
    FloatingPoint default_distance_m = 2;
    float min_weight = 1e-6;
  };

  EsdfIntegrator(const Config& config, Layer<TsdfVoxel>* tsdf_layer,
                 Layer<EsdfVoxel>* esdf_layer)
      : config_(config), tsdf_layer_(tsdf_layer), esdf_layer_(esdf_layer) {
    CHECK(tsdf_layer_);
    CHECK(esdf_layer_);

    esdf_voxels_per_side_ = esdf_layer_->voxels_per_side();
    esdf_voxel_size_ = esdf_layer_->voxel_size();
    /*voxel_size_ = layer_->voxel_size();
    block_size_ = layer_->block_size();
    voxels_per_side_ = layer_->voxels_per_side();

    voxel_size_inv_ = 1.0 / voxel_size_;
    block_size_inv_ = 1.0 / block_size_;
    voxels_per_side_inv_ = 1.0 / voxels_per_side_; */
  }

  void updateFromTsdfLayer(bool clear_updated_flag) {
    // esdf_layer_->removeAllBlocks();

    BlockIndexList tsdf_blocks;

    tsdf_layer_->getAllAllocatedBlocks(&tsdf_blocks);
    tsdf_layer_->getAllUpdatedBlocks(&tsdf_blocks);
    updateFromTsdfBlocks(tsdf_blocks);

    if (clear_updated_flag) {
      for (const BlockIndex& block_index : tsdf_blocks) {
        tsdf_layer_->getBlockByIndex(block_index).updated() = false;
      }
    }
  }

  void updateFromTsdfBlocks(const BlockIndexList& tsdf_blocks) {
    timing::Timer esdf_timer("esdf");

    // Get a specific list of voxels in the TSDF layer, and propagate out from
    // there.
    // Go through all blocks in TSDF and copy their values for relevant voxels.
    timing::Timer propagate_timer("esdf/propagate_tsdf");
    LOG(INFO) << "[ESDF update]: Propagating " << tsdf_blocks.size()
              << " updated blocks from the TSDF.";
    for (const BlockIndex& block_index : tsdf_blocks) {
      const Block<TsdfVoxel>& tsdf_block =
          tsdf_layer_->getBlockByIndex(block_index);

      // Allocate the same block in the ESDF layer.
      // Block indices are the same across all layers.
      Block<EsdfVoxel>::Ptr esdf_block =
          esdf_layer_->allocateBlockPtrByIndex(block_index);

      // TODO(helenol): assumes that TSDF and ESDF layer are the same size.
      // This will not always be true...
      const size_t num_voxels_per_block = tsdf_block.num_voxels();

      for (size_t lin_index = 0; lin_index < num_voxels_per_block;
           ++lin_index) {
        const TsdfVoxel& tsdf_voxel =
            tsdf_block.getVoxelByLinearIndex(lin_index);

        if (tsdf_voxel.weight < config_.min_weight) {
          continue;
        }

        EsdfVoxel& esdf_voxel = esdf_block->getVoxelByLinearIndex(lin_index);

        if (std::abs(tsdf_voxel.distance) < config_.min_distance_m) {
          if (!esdf_voxel.observed ||
              tsdf_voxel.distance < esdf_voxel.distance) {
            esdf_voxel.distance = tsdf_voxel.distance;
            esdf_voxel.observed = true;

            esdf_voxel.in_queue = true;
            open_.push(std::make_pair(
                block_index,
                esdf_block->computeVoxelIndexFromLinearIndex(lin_index)));
          }
        } else if (!esdf_voxel.observed) {
          // Outside of truncation distance, but actually observed in the
          // original map.
          // Should we clear the distance to something?
          // Should we add it to open?
          esdf_voxel.distance =
              signum(tsdf_voxel.distance) * config_.default_distance_m;
          esdf_voxel.observed = true;
        }
      }
    }
    propagate_timer.Stop();

    timing::Timer update_timer("esdf/update_esdf");
    // Process the open set now.
    processOpenSet();
    update_timer.Stop();

    esdf_timer.Stop();
  }

  void processOpenSet() {
    size_t num_updates = 0;
    while (!open_.empty()) {
      VoxelKey kv = open_.front();
      open_.pop();

      Block<EsdfVoxel>::Ptr esdf_block =
          esdf_layer_->getBlockPtrByIndex(kv.first);
      EsdfVoxel& esdf_voxel = esdf_block->getVoxelByVoxelIndex(kv.second);

      // See if you can update the neighbors.
      std::vector<VoxelKey> neighbors;
      std::vector<float> distances;
      getNeighborsAndDistances(kv.first, kv.second, &neighbors, &distances);

      // Do NOT update unobserved distances.
      CHECK_EQ(neighbors.size(), distances.size());
      for (size_t i = 0; i < neighbors.size(); ++i) {
        BlockIndex neighbor_block_index = neighbors[i].first;
        VoxelIndex neighbor_voxel_index = neighbors[i].second;

        // Get the block for this voxel.
        Block<EsdfVoxel>::Ptr neighbor_block;
        if (neighbor_block_index == kv.first) {
          neighbor_block = esdf_block;
        } else {
          neighbor_block =
              esdf_layer_->getBlockPtrByIndex(neighbor_block_index);
        }
        if (!neighbor_block) {
          continue;
        }
        CHECK(neighbor_block->isValidVoxelIndex(neighbor_voxel_index))
            << "Neigbor voxel index: " << neighbor_voxel_index.transpose();

        EsdfVoxel& neighbor_voxel =
            neighbor_block->getVoxelByVoxelIndex(neighbor_voxel_index);

        if (!neighbor_voxel.observed) {
          continue;
        }

        const FloatingPoint distance_to_neighbor =
            distances[i] * esdf_voxel_size_;

        // This is probably an fixed voxel.
        if (std::abs(neighbor_voxel.distance) < config_.min_distance_m) {
          continue;
        }

        if (esdf_voxel.distance > 0 &&
            esdf_voxel.distance + distance_to_neighbor <
                neighbor_voxel.distance) {
          neighbor_voxel.distance = esdf_voxel.distance + distance_to_neighbor;
          if (!neighbor_voxel.in_queue) {
            open_.push(neighbors[i]);
            neighbor_voxel.in_queue = true;
          }
        }
        if (esdf_voxel.distance < 0 &&
            esdf_voxel.distance - distance_to_neighbor >
                neighbor_voxel.distance) {
          neighbor_voxel.distance = esdf_voxel.distance - distance_to_neighbor;
          if (!neighbor_voxel.in_queue) {
            open_.push(neighbors[i]);
            neighbor_voxel.in_queue = true;
          }
        }
      }

      num_updates++;
      esdf_voxel.in_queue = false;
    }

    LOG(INFO) << "[ESDF update]: made " << num_updates << " voxel updates.";
  }

  // Uses 26-connectivity and quasi-Euclidean distances.
  void getNeighborsAndDistances(const BlockIndex& block_index,
                                const VoxelIndex& voxel_index,
                                std::vector<VoxelKey>* neighbors,
                                std::vector<float>* distances) {
    static const double kSqrt2 = std::sqrt(2);
    static const double kSqrt3 = std::sqrt(3);
    static const size_t kNumNeighbors = 26;

    neighbors->reserve(kNumNeighbors);
    distances->reserve(kNumNeighbors);

    VoxelKey neighbor;
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
      unsigned int next_i = (i + 1) % 3;
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
    *neighbor_voxel_index = voxel_index + direction;

    for (unsigned int i = 0; i < 3; ++i) {
      if ((*neighbor_voxel_index)(i) < 0) {
        (*neighbor_block_index)(i)--;
        (*neighbor_voxel_index)(i) += esdf_voxels_per_side_;
      } else if ((*neighbor_voxel_index)(i) >= esdf_voxels_per_side_) {
        (*neighbor_block_index)(i)++;
        (*neighbor_voxel_index)(i) -= esdf_voxels_per_side_;
      }
    }
  }

 protected:
  Config config_;

  Layer<TsdfVoxel>* tsdf_layer_;
  Layer<EsdfVoxel>* esdf_layer_;

  // Open Queue for incremental updates. Contains global voxel indices
  // for the ESDF layer.
  std::queue<VoxelKey> open_;

  size_t esdf_voxels_per_side_;
  FloatingPoint esdf_voxel_size_;

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

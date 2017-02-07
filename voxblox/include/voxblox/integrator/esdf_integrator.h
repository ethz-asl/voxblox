#ifndef VOXBLOX_INTEGRATOR_ESDF_INTEGRATOR_H_
#define VOXBLOX_INTEGRATOR_ESDF_INTEGRATOR_H_

#include <algorithm>
#include <Eigen/Core>
#include <glog/logging.h>
#include <queue>
#include <utility>
#include <vector>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/utils/timing.h"
#include "voxblox/utils/bucket_queue.h"

namespace voxblox {

class EsdfIntegrator {
 public:
  struct Config {
    // Maximum distance to calculate the actual distance to.
    // Any values above this will be set to default_distance_m.
    FloatingPoint max_distance_m = 2.0;
    // Should mirror (or be smaller than) truncation distance in tsdf
    // integrator.
    FloatingPoint min_distance_m = 0.2;
    // Default distance set for unknown values and values > max_distance_m.
    FloatingPoint default_distance_m = 2.0;
    // For cheaper but less accurate map updates: the minimum difference in
    // a voxel distance to require adding it to the raise queue.
    FloatingPoint min_raise_diff_m = 0.0;
    // Minimum weight to consider a TSDF value seen at.
    float min_weight = 1e-6;
    // Number of buckets for the bucketed priority queue.
    int num_buckets = 20;
  };

  EsdfIntegrator(const Config& config, Layer<TsdfVoxel>* tsdf_layer,
                 Layer<EsdfVoxel>* esdf_layer)
      : config_(config), tsdf_layer_(tsdf_layer), esdf_layer_(esdf_layer) {
    CHECK(tsdf_layer_);
    CHECK(esdf_layer_);

    esdf_voxels_per_side_ = esdf_layer_->voxels_per_side();
    esdf_voxel_size_ = esdf_layer_->voxel_size();

    open_.setNumBuckets(config_.num_buckets, config_.max_distance_m);
  }

  inline bool isFixed(FloatingPoint dist_m) const {
    return std::abs(dist_m) < config_.min_distance_m || dist_m < 0.0;
  }

  void updateFromTsdfLayerBatch() {
    esdf_layer_->removeAllBlocks();
    BlockIndexList tsdf_blocks;
    tsdf_layer_->getAllAllocatedBlocks(&tsdf_blocks);
    constexpr bool push_neighbors = false;
    updateFromTsdfBlocks(tsdf_blocks, push_neighbors);
  }

  void updateFromTsdfLayer(bool clear_updated_flag) {
    BlockIndexList tsdf_blocks;
    tsdf_layer_->getAllUpdatedBlocks(&tsdf_blocks);
    updateFromTsdfBlocks(tsdf_blocks);

    if (clear_updated_flag) {
      for (const BlockIndex& block_index : tsdf_blocks) {
        tsdf_layer_->getBlockByIndex(block_index).updated() = false;
      }
    }
  }

  // Short-cut for pushing neighbors (i.e., incremental update) by default.
  // Not necessary in batch.
  void updateFromTsdfBlocks(const BlockIndexList& tsdf_blocks) {
    constexpr bool push_neighbors = true;
    updateFromTsdfBlocks(tsdf_blocks, push_neighbors);
  }

  void updateFromTsdfBlocks(const BlockIndexList& tsdf_blocks,
                            bool push_neighbors) {
    DCHECK_EQ(tsdf_layer_->voxels_per_side(), esdf_layer_->voxels_per_side());
    timing::Timer esdf_timer("esdf");

    // Get a specific list of voxels in the TSDF layer, and propagate out from
    // there.
    // Go through all blocks in TSDF and copy their values for relevant voxels.
    size_t num_lower = 0u;
    size_t num_raise = 0u;
    size_t num_new = 0u;
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

      for (size_t lin_index = 0u; lin_index < num_voxels_per_block;
           ++lin_index) {
        const TsdfVoxel& tsdf_voxel =
            tsdf_block.getVoxelByLinearIndex(lin_index);

        if (tsdf_voxel.weight < config_.min_weight) {
          continue;
        }

        EsdfVoxel& esdf_voxel = esdf_block->getVoxelByLinearIndex(lin_index);
        VoxelIndex voxel_index =
            esdf_block->computeVoxelIndexFromLinearIndex(lin_index);
        // Check for frontier voxels.
        // This is the check for the lower frontier.
        if (isFixed(tsdf_voxel.distance)) {
          if (!esdf_voxel.observed ||
              esdf_voxel.distance > tsdf_voxel.distance) {
            esdf_voxel.distance = tsdf_voxel.distance;
            esdf_voxel.observed = true;
            esdf_voxel.fixed = true;
            esdf_voxel.parent.setZero();

            esdf_voxel.in_queue = true;
            open_.push(std::make_pair(block_index, voxel_index),
                       esdf_voxel.distance);
            num_lower++;
          } else if (esdf_voxel.distance < tsdf_voxel.distance) {
            esdf_voxel.distance = tsdf_voxel.distance;
            esdf_voxel.parent.setZero();
            esdf_voxel.fixed = true;
            raise_.push(std::make_pair(block_index, voxel_index));
            open_.push(std::make_pair(block_index, voxel_index),
                       esdf_voxel.distance);
            esdf_voxel.in_queue = true;
            num_raise++;
          }
        } else {
          // If the tsdf voxel isn't fixed...
          // If it used to be, then this is a raise.
          if (esdf_voxel.observed && esdf_voxel.fixed) {
            esdf_voxel.distance =
                signum(tsdf_voxel.distance) * config_.default_distance_m;
            esdf_voxel.parent.setZero();
            esdf_voxel.fixed = false;
            raise_.push(std::make_pair(block_index, voxel_index));
            num_raise++;
          }
          // Then there's the case where it's a completely new voxel.
          if (!esdf_voxel.observed) {
            esdf_voxel.distance =
                signum(tsdf_voxel.distance) * config_.default_distance_m;
            esdf_voxel.observed = true;
            esdf_voxel.parent.setZero();
            if (push_neighbors) {
              pushNeighborsToOpen(block_index, voxel_index);
            }
            num_new++;
          }
        }
      }
    }
    propagate_timer.Stop();
    LOG(INFO) << "[ESDF update]: Lower: " << num_lower
              << " Raise: " << num_raise << " New: " << num_new;

    timing::Timer raise_timer("esdf/raise_esdf");
    // Process the open set now.
    processRaiseSet();
    raise_timer.Stop();

    timing::Timer update_timer("esdf/update_esdf");
    // Process the open set now.
    processOpenSet();
    update_timer.Stop();

    esdf_timer.Stop();
  }

  void pushNeighborsToOpen(const BlockIndex& block_index,
                           const VoxelIndex& voxel_index) {
    std::vector<VoxelKey> neighbors;
    std::vector<float> distances;
    std::vector<Eigen::Vector3i> directions;
    getNeighborsAndDistances(block_index, voxel_index, &neighbors, &distances,
                             &directions);

    for (const VoxelKey& neighbor : neighbors) {
      BlockIndex neighbor_block_index = neighbor.first;
      VoxelIndex neighbor_voxel_index = neighbor.second;

      // Get the block for this voxel.
      Block<EsdfVoxel>::Ptr neighbor_block =
          esdf_layer_->getBlockPtrByIndex(neighbor_block_index);
      if (!neighbor_block) {
        continue;
      }
      EsdfVoxel& neighbor_voxel =
          neighbor_block->getVoxelByVoxelIndex(neighbor_voxel_index);

      if (!neighbor_voxel.in_queue) {
        open_.push(neighbor, neighbor_voxel.distance);
        neighbor_voxel.in_queue = true;
      }
    }
  }

  void processRaiseSet() {
    size_t num_updates = 0u;
    // For the raise set, get all the neighbors, then:
    // 1. if the neighbor's parent is the current voxel, add it to the raise
    //    queue.
    // 2. if the neighbor's parent differs, add it to open (we will have to
    //    update our current weights, of course).
    while (!raise_.empty()) {
      VoxelKey kv = raise_.front();
      raise_.pop();

      Block<EsdfVoxel>::Ptr esdf_block =
          esdf_layer_->getBlockPtrByIndex(kv.first);

      // See if you can update the neighbors.
      std::vector<VoxelKey> neighbors;
      std::vector<float> distances;
      std::vector<Eigen::Vector3i> directions;
      getNeighborsAndDistances(kv.first, kv.second, &neighbors, &distances,
                               &directions);

      CHECK_EQ(neighbors.size(), distances.size());
      for (size_t i = 0u; i < neighbors.size(); ++i) {
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

        // Do NOT update unobserved distances.
        if (!neighbor_voxel.observed) {
          continue;
        }
        if (neighbor_voxel.parent == -directions[i]) {
          // This is the case where we are the parent of this one, so we
          // should clear it and raise it.
          neighbor_voxel.distance =
              signum(neighbor_voxel.distance) * config_.default_distance_m;
          neighbor_voxel.parent.setZero();
          raise_.push(neighbors[i]);
        } else {
          // If it's not in the queue, then add it to open so it can update
          // our weights back.
          if (!neighbor_voxel.in_queue) {
            open_.push(neighbors[i], neighbor_voxel.distance);
            neighbor_voxel.in_queue = true;
          }
        }
      }
      num_updates++;
    }
    LOG(INFO) << "[ESDF update]: raised " << num_updates << " voxels.";
  }

  void processOpenSet() {
    size_t num_updates = 0u;
    while (!open_.empty()) {
      VoxelKey kv = open_.front();
      open_.pop();

      Block<EsdfVoxel>::Ptr esdf_block =
          esdf_layer_->getBlockPtrByIndex(kv.first);
      EsdfVoxel& esdf_voxel = esdf_block->getVoxelByVoxelIndex(kv.second);

      if (!esdf_voxel.observed) {
        esdf_voxel.in_queue = false;
        continue;
      }

      // Don't bother propagating this -- can't make any active difference.
      if (esdf_voxel.distance >= config_.default_distance_m) {
        esdf_voxel.in_queue = false;
        continue;
      }
      // See if you can update the neighbors.
      std::vector<VoxelKey> neighbors;
      std::vector<float> distances;
      std::vector<Eigen::Vector3i> directions;
      getNeighborsAndDistances(kv.first, kv.second, &neighbors, &distances,
                               &directions);

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

        // Don't bother updating fixed voxels.
        if (neighbor_voxel.fixed) {
          continue;
        }

        if (esdf_voxel.distance > 0.0 &&
            esdf_voxel.distance + distance_to_neighbor <
                neighbor_voxel.distance) {
          neighbor_voxel.distance = esdf_voxel.distance + distance_to_neighbor;
          // Also update parent.
          neighbor_voxel.parent = -directions[i];
          // ONLY propagate this if we're below the max distance!
          if (neighbor_voxel.distance < config_.max_distance_m) {
            if (!neighbor_voxel.in_queue) {
              open_.push(neighbors[i], neighbor_voxel.distance);
              neighbor_voxel.in_queue = true;
            }
          }
        }
        if (esdf_voxel.distance < 0.0 &&
            esdf_voxel.distance - distance_to_neighbor >
                neighbor_voxel.distance) {
          neighbor_voxel.distance = esdf_voxel.distance - distance_to_neighbor;
          // Also update parent.
          neighbor_voxel.parent = -directions[i];
          if (!neighbor_voxel.in_queue) {
            open_.push(neighbors[i], neighbor_voxel.distance);
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
  // Directions is the direction that the neighbor voxel lives in. If you
  // need the direction FROM the neighbor voxel TO the current voxel, take
  // negative of the given direction.
  void getNeighborsAndDistances(const BlockIndex& block_index,
                                const VoxelIndex& voxel_index,
                                std::vector<VoxelKey>* neighbors,
                                std::vector<float>* distances,
                                std::vector<Eigen::Vector3i>* directions) {
    CHECK_NOTNULL(neighbors);
    CHECK_NOTNULL(distances);
    CHECK_NOTNULL(directions);

    static const double kSqrt2 = std::sqrt(2);
    static const double kSqrt3 = std::sqrt(3);
    static const size_t kNumNeighbors = 26;

    neighbors->reserve(kNumNeighbors);
    distances->reserve(kNumNeighbors);
    directions->reserve(kNumNeighbors);

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
        directions->emplace_back(direction);
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
          directions->emplace_back(direction);
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
          directions->emplace_back(direction);
        }
      }
    }

    CHECK_EQ(neighbors->size(), kNumNeighbors);
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
  BucketQueue<VoxelKey> open_;

  // Raise set for updates; these are values that used to be in the fixed
  // frontier and now have a higher value.
  std::queue<VoxelKey> raise_;

  size_t esdf_voxels_per_side_;
  FloatingPoint esdf_voxel_size_;
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_ESDF_INTEGRATOR_H_

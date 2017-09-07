#include "voxblox/integrator/esdf_integrator.h"

#include <iostream>

namespace voxblox {

EsdfIntegrator::EsdfIntegrator(const Config& config,
                               Layer<TsdfVoxel>* tsdf_layer,
                               Layer<EsdfVoxel>* esdf_layer)
    : config_(config), tsdf_layer_(tsdf_layer), esdf_layer_(esdf_layer) {
  CHECK(tsdf_layer_);
  CHECK(esdf_layer_);

  esdf_voxels_per_side_ = esdf_layer_->voxels_per_side();
  esdf_voxel_size_ = esdf_layer_->voxel_size();

  CHECK_EQ(esdf_layer_->voxels_per_side(), tsdf_layer_->voxels_per_side());
  CHECK_NEAR(esdf_layer_->voxel_size(), tsdf_layer_->voxel_size(), 1e-6);

  open_.setNumBuckets(config_.num_buckets, config_.max_distance_m);
}

void EsdfIntegrator::getSphereAroundPoint(
    const Point& center, FloatingPoint radius,
    BlockVoxelListMap* block_voxel_list) const {
  // search a cube with side length 2*radius
  for (FloatingPoint x = -radius; x <= radius; x += esdf_voxel_size_) {
    for (FloatingPoint y = -radius; y <= radius; y += esdf_voxel_size_) {
      for (FloatingPoint z = -radius; z <= radius; z += esdf_voxel_size_) {
        Point point(x, y, z);

        // check if point is inside the spheres radius
        if (point.squaredNorm() <= radius * radius) {
          // convert to global coordinate
          point += center;

          BlockIndex block_index =
              esdf_layer_->computeBlockIndexFromCoordinates(point);

          (*block_voxel_list)[block_index].push_back(
              esdf_layer_->allocateBlockPtrByIndex(block_index)
                  ->computeVoxelIndexFromCoordinates(point));
        }
      }
    }
  }
}

// Used for planning - allocates sphere around as observed but occupied,
// and clears space in a sphere around current position.
void EsdfIntegrator::addNewRobotPosition(const Point& position) {
  timing::Timer clear_timer("esdf/clear_radius");

  // First set all in inner sphere to free.
  BlockVoxelListMap block_voxel_list;
  getSphereAroundPoint(position, config_.clear_sphere_radius,
                       &block_voxel_list);
  for (const std::pair<BlockIndex, VoxelIndexList>& kv : block_voxel_list) {
    // Get block.
    Block<EsdfVoxel>::Ptr block_ptr =
        esdf_layer_->allocateBlockPtrByIndex(kv.first);

    for (const VoxelIndex& voxel_index : kv.second) {
      if (!block_ptr->isValidVoxelIndex(voxel_index)) {
        continue;
      }
      EsdfVoxel& esdf_voxel = block_ptr->getVoxelByVoxelIndex(voxel_index);
      if (!esdf_voxel.observed) {
        esdf_voxel.distance = config_.default_distance_m;
        esdf_voxel.observed = true;
        pushNeighborsToOpen(kv.first, voxel_index);
        updated_blocks_.insert(kv.first);
      }
    }
  }

  // Second set all remaining unknown to occupied.
  block_voxel_list.clear();
  getSphereAroundPoint(position, config_.occupied_sphere_radius,
                       &block_voxel_list);
  for (const std::pair<BlockIndex, VoxelIndexList>& kv : block_voxel_list) {
    // Get block.
    Block<EsdfVoxel>::Ptr block_ptr =
        esdf_layer_->allocateBlockPtrByIndex(kv.first);

    for (const VoxelIndex& voxel_index : kv.second) {
      if (!block_ptr->isValidVoxelIndex(voxel_index)) {
        continue;
      }
      EsdfVoxel& esdf_voxel = block_ptr->getVoxelByVoxelIndex(voxel_index);
      if (!esdf_voxel.observed) {
        esdf_voxel.distance = -config_.default_distance_m;
        esdf_voxel.observed = true;
        pushNeighborsToOpen(kv.first, voxel_index);
        updated_blocks_.insert(kv.first);
      }
    }
  }
  VLOG(3) << "Changed " << updated_blocks_.size()
          << " blocks from unknown to free or occupied near the robot.";
  clear_timer.Stop();
}

void EsdfIntegrator::updateFromTsdfLayerBatch() {
  esdf_layer_->removeAllBlocks();
  BlockIndexList tsdf_blocks;
  tsdf_layer_->getAllAllocatedBlocks(&tsdf_blocks);
  tsdf_blocks.insert(tsdf_blocks.end(), updated_blocks_.begin(),
                     updated_blocks_.end());
  updated_blocks_.clear();
  constexpr bool push_neighbors = false;
  updateFromTsdfBlocks(tsdf_blocks, push_neighbors);
}

void EsdfIntegrator::updateFromTsdfLayerBatchOccupancy() {
  esdf_layer_->removeAllBlocks();
  BlockIndexList tsdf_blocks;
  tsdf_layer_->getAllAllocatedBlocks(&tsdf_blocks);
  updateFromTsdfBlocksAsOccupancy(tsdf_blocks);
}
void EsdfIntegrator::updateFromTsdfLayerBatchFullEuclidean() {
  esdf_layer_->removeAllBlocks();
  BlockIndexList tsdf_blocks;
  tsdf_layer_->getAllAllocatedBlocks(&tsdf_blocks);
  updateFromTsdfBlocksFullEuclidean(tsdf_blocks);
}

void EsdfIntegrator::updateFromTsdfLayer(bool clear_updated_flag) {
  BlockIndexList tsdf_blocks;
  tsdf_layer_->getAllUpdatedBlocks(&tsdf_blocks);
  tsdf_blocks.insert(tsdf_blocks.end(), updated_blocks_.begin(),
                     updated_blocks_.end());
  updated_blocks_.clear();
  updateFromTsdfBlocks(tsdf_blocks);

  if (clear_updated_flag) {
    for (const BlockIndex& block_index : tsdf_blocks) {
      if (tsdf_layer_->hasBlock(block_index)) {
        tsdf_layer_->getBlockByIndex(block_index).updated() = false;
      }
    }
  }
}

// Short-cut for pushing neighbors (i.e., incremental update) by default.
// Not necessary in batch.
void EsdfIntegrator::updateFromTsdfBlocks(const BlockIndexList& tsdf_blocks) {
  constexpr bool push_neighbors = true;
  updateFromTsdfBlocks(tsdf_blocks, push_neighbors);
}

void EsdfIntegrator::updateFromTsdfBlocksFullEuclidean(
    const BlockIndexList& tsdf_blocks) {
  DCHECK_EQ(tsdf_layer_->voxels_per_side(), esdf_layer_->voxels_per_side());
  timing::Timer esdf_timer("esdf");

  // Get a specific list of voxels in the TSDF layer, and propagate out from
  // there.
  // Go through all blocks in TSDF and copy their values for relevant voxels.
  size_t num_lower = 0u;
  size_t num_raise = 0u;
  size_t num_new = 0u;
  timing::Timer propagate_timer("esdf/propagate_tsdf");
  VLOG(3) << "[ESDF update]: Propagating " << tsdf_blocks.size()
          << " updated blocks from the TSDF.";
  for (const BlockIndex& block_index : tsdf_blocks) {
    if (!tsdf_layer_->hasBlock(block_index)) {
      continue;
    }
    const Block<TsdfVoxel>& tsdf_block =
        tsdf_layer_->getBlockByIndex(block_index);

    // Allocate the same block in the ESDF layer.
    // Block indices are the same across all layers.
    Block<EsdfVoxel>::Ptr esdf_block =
        esdf_layer_->allocateBlockPtrByIndex(block_index);

    // TODO(helenol): assumes that TSDF and ESDF layer are the same size.
    // This will not always be true...
    const size_t num_voxels_per_block = tsdf_block.num_voxels();

    for (size_t lin_index = 0u; lin_index < num_voxels_per_block; ++lin_index) {
      const TsdfVoxel& tsdf_voxel = tsdf_block.getVoxelByLinearIndex(lin_index);

      if (tsdf_voxel.weight < config_.min_weight) {
        continue;
      }

      EsdfVoxel& esdf_voxel = esdf_block->getVoxelByLinearIndex(lin_index);
      VoxelIndex voxel_index =
          esdf_block->computeVoxelIndexFromLinearIndex(lin_index);
      // Check for frontier voxels.
      // This is the check for the lower frontier.
      if (isFixed(tsdf_voxel.distance)) {
        // This is if the distance has been lowered or the voxel is new.
        // Gets put into lower frontier (open_).
        esdf_voxel.distance = tsdf_voxel.distance;
        esdf_voxel.observed = true;
        esdf_voxel.fixed = true;
        esdf_voxel.parent.setZero();

        esdf_voxel.in_queue = true;
        open_.push(std::make_pair(block_index, voxel_index),
                   esdf_voxel.distance);
        num_lower++;
      } else {
        // If the tsdf voxel isn't fixed...
        esdf_voxel.distance =
            signum(tsdf_voxel.distance) * config_.default_distance_m;
        esdf_voxel.observed = true;
        esdf_voxel.fixed = false;
        esdf_voxel.parent.setZero();
        num_new++;
      }
    }
  }
  propagate_timer.Stop();
  VLOG(3) << "[ESDF update]: Lower: " << num_lower << " Raise: " << num_raise
          << " New: " << num_new;

  timing::Timer update_timer("esdf/update_esdf");
  // Process the open set now.
  processOpenSetFullEuclidean();
  update_timer.Stop();

  esdf_timer.Stop();
}

void EsdfIntegrator::updateFromTsdfBlocks(const BlockIndexList& tsdf_blocks,
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
  VLOG(3) << "[ESDF update]: Propagating " << tsdf_blocks.size()
          << " updated blocks from the TSDF.";
  for (const BlockIndex& block_index : tsdf_blocks) {
    if (!tsdf_layer_->hasBlock(block_index)) {
      continue;
    }
    const Block<TsdfVoxel>& tsdf_block =
        tsdf_layer_->getBlockByIndex(block_index);

    // Allocate the same block in the ESDF layer.
    // Block indices are the same across all layers.
    Block<EsdfVoxel>::Ptr esdf_block =
        esdf_layer_->allocateBlockPtrByIndex(block_index);

    // TODO(helenol): assumes that TSDF and ESDF layer are the same size.
    // This will not always be true...
    const size_t num_voxels_per_block = tsdf_block.num_voxels();

    for (size_t lin_index = 0u; lin_index < num_voxels_per_block; ++lin_index) {
      const TsdfVoxel& tsdf_voxel = tsdf_block.getVoxelByLinearIndex(lin_index);

      if (tsdf_voxel.weight < config_.min_weight) {
        continue;
      }

      EsdfVoxel& esdf_voxel = esdf_block->getVoxelByLinearIndex(lin_index);
      VoxelIndex voxel_index =
          esdf_block->computeVoxelIndexFromLinearIndex(lin_index);
      // Check for frontier voxels.
      // This is the check for the lower frontier.
      if (isFixed(tsdf_voxel.distance)) {
        // This is if the distance has been lowered or the voxel is new.
        // Gets put into lower frontier (open_).
        if (!esdf_voxel.observed ||
            (tsdf_voxel.distance >= 0.0 &&
             esdf_voxel.distance >
                 (tsdf_voxel.distance + config_.min_diff_m)) ||
            (tsdf_voxel.distance < 0.0 &&
             esdf_voxel.distance <
                 (tsdf_voxel.distance - config_.min_diff_m))) {
          esdf_voxel.distance = tsdf_voxel.distance;
          esdf_voxel.observed = true;
          esdf_voxel.fixed = true;
          esdf_voxel.parent.setZero();

          esdf_voxel.in_queue = true;
          open_.push(std::make_pair(block_index, voxel_index),
                     esdf_voxel.distance);
          num_lower++;
        } else if ((tsdf_voxel.distance >= 0.0 &&
                    esdf_voxel.distance <
                        (tsdf_voxel.distance - config_.min_diff_m)) ||
                   (tsdf_voxel.distance < 0.0 &&
                    esdf_voxel.distance >
                        (tsdf_voxel.distance + config_.min_diff_m))) {
          // In case the fixed voxel has a HIGHER distance than the esdf
          // voxel. Need to raise it, and burn its children.
          esdf_voxel.distance = tsdf_voxel.distance;
          esdf_voxel.parent.setZero();
          esdf_voxel.fixed = true;
          raise_.push(std::make_pair(block_index, voxel_index));
          // We need to also make sure this voxel ends up in the lower
          // frontier, as it is fixed.
          open_.push(std::make_pair(block_index, voxel_index),
                     esdf_voxel.distance);
          esdf_voxel.in_queue = true;
          num_raise++;
        }
      } else {
        // If the tsdf voxel isn't fixed...
        // If it used to be, then this is a raise.
        // Or sign is flipped...

        if (esdf_voxel.observed &&
            (esdf_voxel.fixed ||
             signum(esdf_voxel.distance) != signum(tsdf_voxel.distance))) {
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
          esdf_voxel.fixed = false;
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
  VLOG(3) << "[ESDF update]: Lower: " << num_lower << " Raise: " << num_raise
          << " New: " << num_new;

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

void EsdfIntegrator::updateFromTsdfBlocksAsOccupancy(
    const BlockIndexList& tsdf_blocks) {
  DCHECK_EQ(tsdf_layer_->voxels_per_side(), esdf_layer_->voxels_per_side());
  timing::Timer esdf_timer("esdf");

  // Get a specific list of voxels in the TSDF layer, and propagate out from
  // there.
  // Go through all blocks in TSDF and copy their values for relevant voxels.
  size_t num_lower = 0u;
  size_t num_raise = 0u;
  size_t num_new = 0u;
  timing::Timer propagate_timer("esdf/propagate_tsdf");
  VLOG(3) << "[ESDF update]: Propagating " << tsdf_blocks.size()
          << " updated blocks from the TSDF.";
  for (const BlockIndex& block_index : tsdf_blocks) {
    if (!tsdf_layer_->hasBlock(block_index)) {
      continue;
    }
    const Block<TsdfVoxel>& tsdf_block =
        tsdf_layer_->getBlockByIndex(block_index);

    // Allocate the same block in the ESDF layer.
    // Block indices are the same across all layers.
    Block<EsdfVoxel>::Ptr esdf_block =
        esdf_layer_->allocateBlockPtrByIndex(block_index);

    // TODO(helenol): assumes that TSDF and ESDF layer are the same size.
    // This will not always be true...
    const size_t num_voxels_per_block = tsdf_block.num_voxels();

    for (size_t lin_index = 0u; lin_index < num_voxels_per_block; ++lin_index) {
      const TsdfVoxel& tsdf_voxel = tsdf_block.getVoxelByLinearIndex(lin_index);

      if (tsdf_voxel.weight < config_.min_weight) {
        continue;
      }

      EsdfVoxel& esdf_voxel = esdf_block->getVoxelByLinearIndex(lin_index);
      VoxelIndex voxel_index =
          esdf_block->computeVoxelIndexFromLinearIndex(lin_index);
      // Check for frontier voxels.
      // This is the check for the lower frontier.
      if (isFixedOccupancy(tsdf_voxel.distance)) {
        // This is if the distance has been lowered or the voxel is new.
        // Gets put into lower frontier (open_).
        if (!esdf_voxel.observed || !esdf_voxel.fixed) {
          esdf_voxel.distance = tsdf_voxel.distance;
          esdf_voxel.observed = true;
          esdf_voxel.fixed = true;
          esdf_voxel.parent.setZero();

          esdf_voxel.in_queue = true;
          open_.push(std::make_pair(block_index, voxel_index),
                     esdf_voxel.distance);
          num_lower++;
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
          esdf_voxel.fixed = false;
          esdf_voxel.parent.setZero();
          num_new++;
        }
      }
    }
  }
  propagate_timer.Stop();
  VLOG(3) << "[ESDF occ update]: Lower: " << num_lower
          << " Raise: " << num_raise << " New: " << num_new;

  timing::Timer update_timer("esdf/update_esdf");
  // Process the open set now.
  // processOpenSetFullEuclidean();
  processOpenSet();
  update_timer.Stop();

  esdf_timer.Stop();
}

void EsdfIntegrator::pushNeighborsToOpen(const BlockIndex& block_index,
                                         const VoxelIndex& voxel_index) {
  AlignedVector<VoxelKey> neighbors;
  AlignedVector<float> distances;
  AlignedVector<Eigen::Vector3i> directions;
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

    if (!neighbor_voxel.observed) {
      continue;
    }

    if (!neighbor_voxel.in_queue) {
      open_.push(neighbor, neighbor_voxel.distance);
      neighbor_voxel.in_queue = true;
    }
  }
}

// The raise set is always empty in batch operations.
void EsdfIntegrator::processRaiseSet() {
  size_t num_updates = 0u;
  // For the raise set, get all the neighbors, then:
  // 1. if the neighbor's parent is the current voxel, add it to the raise
  //    queue.
  // 2. if the neighbor's parent differs, add it to open (we will have to
  //    update our current distances, of course).
  while (!raise_.empty()) {
    VoxelKey kv = raise_.front();
    raise_.pop();

    Block<EsdfVoxel>::Ptr esdf_block =
        esdf_layer_->getBlockPtrByIndex(kv.first);

    // See if you can update the neighbors.
    AlignedVector<VoxelKey> neighbors;
    AlignedVector<float> distances;
    AlignedVector<Eigen::Vector3i> directions;
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
        neighbor_block = esdf_layer_->getBlockPtrByIndex(neighbor_block_index);
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
      // This will never update fixed voxels as they are their own parents.
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
  VLOG(3) << "[ESDF update]: raised " << num_updates << " voxels.";
}

void EsdfIntegrator::processOpenSet() {
  size_t num_updates = 0u;
  while (!open_.empty()) {
    VoxelKey kv = open_.front();
    open_.pop();

    Block<EsdfVoxel>::Ptr esdf_block =
        esdf_layer_->getBlockPtrByIndex(kv.first);
    if (!esdf_block) {
      continue;
    }

    EsdfVoxel& esdf_voxel = esdf_block->getVoxelByVoxelIndex(kv.second);

    // Again, no point updating unobserved voxels.
    if (!esdf_voxel.observed) {
      esdf_voxel.in_queue = false;
      continue;
    }

    // Don't bother propagating this -- can't make any active difference.
    if (esdf_voxel.distance >= config_.max_distance_m ||
        esdf_voxel.distance <= -config_.max_distance_m) {
      esdf_voxel.in_queue = false;
      continue;
    }
    // See if you can update the neighbors.
    AlignedVector<VoxelKey> neighbors;
    AlignedVector<float> distances;
    AlignedVector<Eigen::Vector3i> directions;
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
        neighbor_block = esdf_layer_->getBlockPtrByIndex(neighbor_block_index);
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

      // Everything outside the surface.
      // I think this can easily be combined with that below...
      if (esdf_voxel.distance + distance_to_neighbor >= 0.0 &&
          neighbor_voxel.distance >= 0.0 && esdf_voxel.distance >= 0.0 &&
          esdf_voxel.distance + distance_to_neighbor + config_.min_diff_m <
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

      // Everything inside the surface.
      if (esdf_voxel.distance - distance_to_neighbor < 0.0 &&
          neighbor_voxel.distance <= 0.0 && esdf_voxel.distance <= 0.0 &&
          esdf_voxel.distance - distance_to_neighbor - config_.min_diff_m >
              neighbor_voxel.distance) {
        neighbor_voxel.distance = esdf_voxel.distance - distance_to_neighbor;
        // Also update parent.
        neighbor_voxel.parent = -directions[i];
        if (neighbor_voxel.distance > -config_.max_distance_m) {
          if (!neighbor_voxel.in_queue) {
            open_.push(neighbors[i], neighbor_voxel.distance);
            neighbor_voxel.in_queue = true;
          }
        }
      }

      // If there's a sign flippy flip.
      if (signum(esdf_voxel.distance) != signum(neighbor_voxel.distance)) {
        if (esdf_voxel.fixed &&
            std::abs(neighbor_voxel.distance + esdf_voxel.distance) >
                distance_to_neighbor) {
          neighbor_voxel.distance =
              esdf_voxel.distance -
              signum(esdf_voxel.distance) * distance_to_neighbor;
          neighbor_voxel.parent = -directions[i];
          if (!neighbor_voxel.in_queue) {
            open_.push(neighbors[i], neighbor_voxel.distance);
            neighbor_voxel.in_queue = true;
          }
        } else if (std::abs(neighbor_voxel.distance + esdf_voxel.distance) <
                   distance_to_neighbor) {
          // Do nothing, we good.
        } else if (neighbor_voxel.distance < 0.0 &&
                   esdf_voxel.distance > distance_to_neighbor) {
          // OK now the other case is if it's 2 totally different signs...
          neighbor_voxel.distance = -distance_to_neighbor;
          neighbor_voxel.parent = -directions[i];
          if (!neighbor_voxel.in_queue) {
            open_.push(neighbors[i], neighbor_voxel.distance);
            neighbor_voxel.in_queue = true;
          }
        } else if (neighbor_voxel.distance >= distance_to_neighbor &&
                   esdf_voxel.distance < -distance_to_neighbor) {
          // OK now the other case is if it's 2 totally different signs...
          neighbor_voxel.distance = distance_to_neighbor;
          neighbor_voxel.parent = -directions[i];
          if (!neighbor_voxel.in_queue) {
            open_.push(neighbors[i], neighbor_voxel.distance);
            neighbor_voxel.in_queue = true;
          }
        }
      }
    }

    num_updates++;
    esdf_voxel.in_queue = false;
  }

  VLOG(3) << "[ESDF update]: made " << num_updates << " voxel updates.";
}

void EsdfIntegrator::processOpenSetFullEuclidean() {
  size_t num_updates = 0u;
  while (!open_.empty()) {
    VoxelKey kv = open_.front();
    open_.pop();

    Block<EsdfVoxel>::Ptr esdf_block =
        esdf_layer_->getBlockPtrByIndex(kv.first);
    EsdfVoxel& esdf_voxel = esdf_block->getVoxelByVoxelIndex(kv.second);

    // Again, no point updating unobserved voxels.
    if (!esdf_voxel.observed) {
      esdf_voxel.in_queue = false;
      continue;
    }

    // Don't bother propagating this -- can't make any active difference.
    if (esdf_voxel.distance >= config_.max_distance_m) {
      esdf_voxel.in_queue = false;
      continue;
    }

    // Figure out what the parent distance would have been.
    FloatingPoint parent_distance =
        esdf_voxel.distance - esdf_voxel.parent.norm() * esdf_voxel_size_;

    // See if you can update the neighbors.
    AlignedVector<VoxelKey> neighbors;
    AlignedVector<float> distances;
    AlignedVector<Eigen::Vector3i> directions;
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
        neighbor_block = esdf_layer_->getBlockPtrByIndex(neighbor_block_index);
      }
      if (!neighbor_block) {
        continue;
      }
      CHECK(neighbor_block->isValidVoxelIndex(neighbor_voxel_index))
          << "Neigbor voxel index: " << neighbor_voxel_index.transpose();

      EsdfVoxel& neighbor_voxel =
          neighbor_block->getVoxelByVoxelIndex(neighbor_voxel_index);

      if (!neighbor_voxel.observed || neighbor_voxel.fixed) {
        continue;
      }

      // + or - direction?? Maybe minus...
      const FloatingPoint neighbor_distance =
          parent_distance +
          (-esdf_voxel.parent.cast<FloatingPoint>() +
           directions[i].cast<FloatingPoint>())
                  .norm() *
              esdf_voxel_size_;

      if (neighbor_distance >= 0.0 &&
          neighbor_distance < neighbor_voxel.distance) {
        neighbor_voxel.distance = neighbor_distance;
        // Also update parent.
        neighbor_voxel.parent = esdf_voxel.parent - directions[i];
        // ONLY propagate this if we're below the max distance!
        if (neighbor_voxel.distance < config_.max_distance_m) {
          if (!neighbor_voxel.in_queue) {
            open_.push(neighbors[i], neighbor_voxel.distance);
            neighbor_voxel.in_queue = true;
          }
        }
      }

      if (neighbor_distance < 0.0 &&
          neighbor_distance > neighbor_voxel.distance) {
        neighbor_voxel.distance = neighbor_distance;
        // Also update parent.
        neighbor_voxel.parent = esdf_voxel.parent - directions[i];
        if (!neighbor_voxel.in_queue) {
          open_.push(neighbors[i], neighbor_voxel.distance);
          neighbor_voxel.in_queue = true;
        }
      }
    }

    num_updates++;
    esdf_voxel.in_queue = false;
  }

  VLOG(3) << "[ESDF update]: [Euclidean] made " << num_updates
          << " voxel updates.";
}

// Uses 26-connectivity and quasi-Euclidean distances.
// Directions is the direction that the neighbor voxel lives in. If you
// need the direction FROM the neighbor voxel TO the current voxel, take
// negative of the given direction.
void EsdfIntegrator::getNeighborsAndDistances(
    const BlockIndex& block_index, const VoxelIndex& voxel_index,
    AlignedVector<VoxelKey>* neighbors, AlignedVector<float>* distances,
    AlignedVector<Eigen::Vector3i>* directions) const {
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

void EsdfIntegrator::getNeighbor(const BlockIndex& block_index,
                                 const VoxelIndex& voxel_index,
                                 const Eigen::Vector3i& direction,
                                 BlockIndex* neighbor_block_index,
                                 VoxelIndex* neighbor_voxel_index) const {
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

}  // namespace voxblox

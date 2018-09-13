#include "voxblox/integrator/esdf_integrator_new.h"

namespace voxblox {

EsdfIntegratorNew::EsdfIntegratorNew(const Config& config,
                                     Layer<TsdfVoxel>* tsdf_layer,
                                     Layer<EsdfVoxel>* esdf_layer)
    : config_(config), tsdf_layer_(tsdf_layer), esdf_layer_(esdf_layer) {
  CHECK(tsdf_layer_);
  CHECK(esdf_layer_);

  voxels_per_side_ = esdf_layer_->voxels_per_side();
  voxel_size_ = esdf_layer_->voxel_size();

  CHECK_EQ(esdf_layer_->voxels_per_side(), tsdf_layer_->voxels_per_side());
  CHECK_NEAR(esdf_layer_->voxel_size(), tsdf_layer_->voxel_size(), 1e-6);

  open_.setNumBuckets(config_.num_buckets, config_.max_distance_m);

  neighbor_tools_.setLayer(esdf_layer);
}

// Used for planning - allocates sphere around as observed but occupied,
// and clears space in a sphere around current position.
void EsdfIntegratorNew::addNewRobotPosition(const Point& position) {
  timing::Timer clear_timer("esdf/clear_radius");
  /*
  // First set all in inner sphere to free.
  HierarchicalIndexMap block_voxel_list;
  timing::Timer sphere_timer("esdf/clear_radius/get_sphere");
  utils::getAndAllocateSphereAroundPoint(position, config_.clear_sphere_radius,
                                         esdf_layer_, &block_voxel_list);
  sphere_timer.Stop();
  for (const std::pair<BlockIndex, VoxelIndexList>& kv : block_voxel_list) {
    // Get block.
    Block<EsdfVoxel>::Ptr block_ptr = esdf_layer_->getBlockPtrByIndex(kv.first);

    for (const VoxelIndex& voxel_index : kv.second) {
      if (!block_ptr->isValidVoxelIndex(voxel_index)) {
        continue;
      }
      EsdfVoxel& esdf_voxel = block_ptr->getVoxelByVoxelIndex(voxel_index);
      // We can clear unobserved or hallucinated voxels.
      if (!esdf_voxel.observed || esdf_voxel.hallucinated) {
        esdf_voxel.distance = config_.default_distance_m;
        esdf_voxel.observed = true;
        esdf_voxel.hallucinated = true;
        updated_blocks_.insert(kv.first);
        pushNeighborsToOpen(kv.first, voxel_index);
      }
    }
  }

  // Second set all remaining unknown to occupied.
  HierarchicalIndexMap block_voxel_list_occ;
  timing::Timer outer_sphere_timer("esdf/clear_radius/get_outer_sphere");
  utils::getAndAllocateSphereAroundPoint(position,
                                         config_.occupied_sphere_radius,
                                         esdf_layer_, &block_voxel_list_occ);
  outer_sphere_timer.Stop();
  for (const std::pair<BlockIndex, VoxelIndexList>& kv : block_voxel_list_occ) {
    // Get block.
    Block<EsdfVoxel>::Ptr block_ptr = esdf_layer_->getBlockPtrByIndex(kv.first);

    for (const VoxelIndex& voxel_index : kv.second) {
      if (!block_ptr->isValidVoxelIndex(voxel_index)) {
        continue;
      }
      EsdfVoxel& esdf_voxel = block_ptr->getVoxelByVoxelIndex(voxel_index);
      if (!esdf_voxel.observed) {
        esdf_voxel.distance = -config_.default_distance_m;
        esdf_voxel.observed = true;
        esdf_voxel.hallucinated = true;
        updated_blocks_.insert(kv.first);
        pushNeighborsToOpen(kv.first, voxel_index);
      }
    }
  }

  VLOG(3) << "Changed " << updated_blocks_.size()
          << " blocks from unknown to free or occupied near the robot."; */
  clear_timer.Stop();
}

void EsdfIntegratorNew::updateFromTsdfLayerBatch() {
  esdf_layer_->removeAllBlocks();
  BlockIndexList tsdf_blocks;
  tsdf_layer_->getAllAllocatedBlocks(&tsdf_blocks);
  tsdf_blocks.insert(tsdf_blocks.end(), updated_blocks_.begin(),
                     updated_blocks_.end());
  updated_blocks_.clear();
  updateFromTsdfBlocks(tsdf_blocks);
}

void EsdfIntegratorNew::updateFromTsdfLayer(bool clear_updated_flag) {
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

void EsdfIntegratorNew::updateFromTsdfBlocks(
    const BlockIndexList& tsdf_blocks) {
  CHECK_EQ(tsdf_layer_->voxels_per_side(), esdf_layer_->voxels_per_side());
  timing::Timer esdf_timer("esdf");

  // Go through all blocks in TSDF and copy their values for relevant voxels.
  size_t num_lower = 0u;
  size_t num_raise = 0u;
  size_t num_new = 0u;
  timing::Timer propagate_timer("esdf/propagate_tsdf");
  VLOG(3) << "[ESDF update]: Propagating " << tsdf_blocks.size()
          << " updated blocks from the TSDF.";
  for (const BlockIndex& block_index : tsdf_blocks) {
    Block<TsdfVoxel>::ConstPtr tsdf_block =
        tsdf_layer_->getBlockPtrByIndex(block_index);
    if (!tsdf_block) {
      continue;
    }

    // Allocate the same block in the ESDF layer.
    // Block indices are the same across all layers.
    Block<EsdfVoxel>::Ptr esdf_block =
        esdf_layer_->allocateBlockPtrByIndex(block_index);
    esdf_block->set_updated(true);

    const size_t num_voxels_per_block = tsdf_block->num_voxels();
    for (size_t lin_index = 0u; lin_index < num_voxels_per_block; ++lin_index) {
      const TsdfVoxel& tsdf_voxel =
          tsdf_block->getVoxelByLinearIndex(lin_index);
      // If this voxel is unobserved in the original map, skip it.
      if (tsdf_voxel.weight < config_.min_weight) {
        continue;
      }
      EsdfVoxel& esdf_voxel = esdf_block->getVoxelByLinearIndex(lin_index);
      VoxelIndex voxel_index =
          esdf_block->computeVoxelIndexFromLinearIndex(lin_index);
      GlobalIndex global_index = getGlobalVoxelIndexFromBlockAndVoxelIndex(
          block_index, voxel_index, voxels_per_side_);

      // If there was nothing there before:
      if (!esdf_voxel.observed) {
        // Two options: ESDF is in the fixed truncation band, or outside.
        if (isFixed(tsdf_voxel.distance)) {
          // In fixed band, just add and lock it.
          esdf_voxel.distance = tsdf_voxel.distance;
          esdf_voxel.fixed = true;
          // Also add it to open so it can update the neighbors.
          open_.push(global_index, tsdf_voxel.distance);
        } else {
          // Not in the fixed band. Just copy the sign.
          esdf_voxel.distance =
              signum(tsdf_voxel.distance) * config_.default_distance_m;
          esdf_voxel.fixed = false;
        }
        // Both of these are set to the correct value no matter what.
        esdf_voxel.observed = true;
        esdf_voxel.parent.setZero();
        num_new++;
      } else {
        // If this voxel DID exist before.
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

// The raise set is always empty in batch operations.
void EsdfIntegratorNew::processRaiseSet() {
  size_t num_updates = 0u;
  // For the raise set, get all the neighbors, then:
  // 1. if the neighbor's parent is the current voxel, add it to the raise
  //    queue.
  // 2. if the neighbor's parent differs, add it to open (we will have to
  //    update our current distances, of course).
  /*  while (!raise_.empty()) {
    VoxelKey kv = raise_.front();
    raise_.pop();

    Block<EsdfVoxel>::Ptr esdf_block =
        esdf_layer_->getBlockPtrByIndex(kv.first);

    // See if you can update the neighbors.
    AlignedVector<VoxelKey> neighbors;
    AlignedVector<float> distances;
    AlignedVector<Eigen::Vector3i> directions;
    neighbor_tools_.getNeighborIndexesAndDistances(
        kv.first, kv.second, Connectivity::kTwentySix, &neighbors,
  &distances,
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
  }*/
  VLOG(3) << "[ESDF update]: raised " << num_updates << " voxels.";
}

void EsdfIntegratorNew::processOpenSet() {
  size_t num_updates = 0u;
  GlobalIndexVector neighbors;

  while (!open_.empty()) {
    GlobalIndex global_index = open_.front();
    open_.pop();

    EsdfVoxel* voxel = esdf_layer_->getVoxelPtrByGlobalIndex(global_index);
    CHECK_NOTNULL(voxel);

    // Skip voxels that are unobserved or outside the ranges we care about.
    if (!voxel->observed || voxel->distance >= config_.max_distance_m ||
        voxel->distance <= -config_.max_distance_m) {
      voxel->in_queue = false;
      continue;
    }

    // Get the global indices of neighbors.
    neighbor_tools_.getNeighborsByGlobalIndex(
        global_index, Connectivity::kTwentySix, &neighbors);

    // Go through the neighbors and see if we can update any of them.
    for (const GlobalIndex& neighbor_index : neighbors) {
      EsdfVoxel* neighbor_voxel =
          esdf_layer_->getVoxelPtrByGlobalIndex(neighbor_index);
      if (neighbor_voxel == nullptr) {
        continue;
      }

      // Don't touch unobserved voxels and can't do anything with fixed voxels.
      if (!neighbor_voxel->observed || neighbor_voxel->fixed) {
        continue;
      }

      SignedIndex direction = (neighbor_index - global_index).cast<int>();
      FloatingPoint distance =
          direction.cast<FloatingPoint>().norm() * voxel_size_;

      // Both are OUTSIDE the surface.
      if (voxel->distance > 0 && neighbor_voxel->distance > 0) {
        if (voxel->distance + distance + config_.min_diff_m <
            neighbor_voxel->distance) {
          num_updates++;
          neighbor_voxel->distance = voxel->distance + distance;
          // Also update parent.
          neighbor_voxel->parent = -direction;
          // Push into the queue if necessary.
          if (config_.multi_queue || !neighbor_voxel->in_queue) {
            open_.push(neighbor_index, neighbor_voxel->distance);
            neighbor_voxel->in_queue = true;
          }
        }
        // Next case is both INSIDE the surface.
      } else if (voxel->distance <= 0 && neighbor_voxel->distance <= 0) {
        if (voxel->distance - distance - config_.min_diff_m >
            neighbor_voxel->distance) {
          num_updates++;
          neighbor_voxel->distance = voxel->distance - distance;
          // Also update parent.
          neighbor_voxel->parent = -direction;
          // Push into the queue if necessary.
          if (config_.multi_queue || !neighbor_voxel->in_queue) {
            open_.push(neighbor_index, neighbor_voxel->distance);
            neighbor_voxel->in_queue = true;
          }
        }
        // Final case is if the signs are different.
      } else {
        FloatingPoint potential_distance =
            voxel->distance - signum(voxel->distance) * distance;
        if (std::abs(potential_distance - neighbor_voxel->distance) >
            distance) {
          if (signum(potential_distance) == neighbor_voxel->distance) {
            num_updates++;
            neighbor_voxel->distance = potential_distance;
            // Also update parent.
            neighbor_voxel->parent = -direction;
            // Push into the queue if necessary.
            if (config_.multi_queue || !neighbor_voxel->in_queue) {
              open_.push(neighbor_index, neighbor_voxel->distance);
              neighbor_voxel->in_queue = true;
            }
          } else {
            num_updates++;
            neighbor_voxel->distance =
                signum(neighbor_voxel->distance) * distance;
            // Also update parent.
            neighbor_voxel->parent = -direction;
            // Push into the queue if necessary.
            if (config_.multi_queue || !neighbor_voxel->in_queue) {
              open_.push(neighbor_index, neighbor_voxel->distance);
              neighbor_voxel->in_queue = true;
            }
          }
        }
      }
    }

    voxel->in_queue = false;
  }

  VLOG(3) << "[ESDF update]: made " << num_updates << " voxel updates.";
}

}  // namespace voxblox

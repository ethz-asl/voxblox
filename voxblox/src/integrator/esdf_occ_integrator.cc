#include "voxblox/integrator/esdf_occ_integrator.h"

namespace voxblox {

EsdfOccIntegrator::EsdfOccIntegrator(const Config& config,
                                     Layer<OccupancyVoxel>* occ_layer,
                                     Layer<EsdfVoxel>* esdf_layer)
    : config_(config), occ_layer_(occ_layer), esdf_layer_(esdf_layer) {
  CHECK_NOTNULL(occ_layer_);
  CHECK_NOTNULL(esdf_layer_);

  esdf_voxels_per_side_ = esdf_layer_->voxels_per_side();
  esdf_voxel_size_ = esdf_layer_->voxel_size();

  open_.setNumBuckets(config_.num_buckets, config_.max_distance_m);
}

// Fixed is overloaded as occupied in this case.
void EsdfOccIntegrator::updateFromOccLayerBatch() {
  esdf_layer_->removeAllBlocks();
  BlockIndexList occ_blocks;
  occ_layer_->getAllAllocatedBlocks(&occ_blocks);
  updateFromOccBlocks(occ_blocks);
}

void EsdfOccIntegrator::updateFromOccBlocks(const BlockIndexList& occ_blocks) {
  DCHECK_EQ(occ_layer_->voxels_per_side(), esdf_layer_->voxels_per_side());
  timing::Timer esdf_timer("esdf_occ");

  // Get a specific list of voxels in the TSDF layer, and propagate out from
  // there.
  // Go through all blocks in TSDF and copy their values for relevant voxels.
  size_t num_lower = 0u;
  size_t num_raise = 0u;
  size_t num_new = 0u;
  timing::Timer propagate_timer("esdf_occ/propagate_tsdf");
  VLOG(3) << "[ESDF update]: Propagating " << occ_blocks.size()
          << " updated blocks from the TSDF.";
  for (const BlockIndex& block_index : occ_blocks) {
    const Block<OccupancyVoxel>& occ_block =
        occ_layer_->getBlockByIndex(block_index);

    // Allocate the same block in the ESDF layer.
    // Block indices are the same across all layers.
    Block<EsdfVoxel>::Ptr esdf_block =
        esdf_layer_->allocateBlockPtrByIndex(block_index);

    // TODO(helenol): assumes that TSDF and ESDF layer are the same size.
    // This will not always be true...
    const size_t num_voxels_per_block = occ_block.num_voxels();

    for (size_t lin_index = 0u; lin_index < num_voxels_per_block; ++lin_index) {
      const OccupancyVoxel& occ_voxel =
          occ_block.getVoxelByLinearIndex(lin_index);

      if (!occ_voxel.observed) {
        continue;
      }

      EsdfVoxel& esdf_voxel = esdf_block->getVoxelByLinearIndex(lin_index);
      VoxelIndex voxel_index =
          esdf_block->computeVoxelIndexFromLinearIndex(lin_index);
      // Check for frontier voxels.
      // This is the check for the lower frontier.
      // If the occupancy voxel is occupied... Count unknown as free.
      if (occ_voxel.probability_log > 0.0) {
        // Assume batch for now.
        esdf_voxel.distance = 0.0;
        esdf_voxel.observed = true;
        esdf_voxel.fixed = true;
        esdf_voxel.parent.setZero();

        esdf_voxel.in_queue = true;
        open_.push(std::make_pair(block_index, voxel_index),
                   esdf_voxel.distance);
        num_lower++;
      } else {
        esdf_voxel.distance = config_.default_distance_m;
        esdf_voxel.observed = true;
        esdf_voxel.fixed = false;
        esdf_voxel.parent.setZero();
        // No need to push to open...
        num_new++;
      }
    }
  }
  propagate_timer.Stop();
  VLOG(3) << "[ESDF update]: Lower: " << num_lower << " Raise: " << num_raise
          << " New: " << num_new;

  timing::Timer raise_timer("esdf_occ/raise_esdf");
  // Process the open set now.
  // processRaiseSet();
  raise_timer.Stop();

  timing::Timer update_timer("esdf_occ/update_esdf");
  // Process the open set now.
  processOpenSet();
  update_timer.Stop();

  esdf_timer.Stop();
}

void EsdfOccIntegrator::processOpenSet() {
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

      if (!neighbor_voxel.fixed && esdf_voxel.distance + distance_to_neighbor <
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

      if (neighbor_voxel.fixed && esdf_voxel.distance - distance_to_neighbor >
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

  VLOG(3) << "[ESDF update]: made " << num_updates << " voxel updates.";
}

// Uses 26-connectivity and quasi-Euclidean distances.
// Directions is the direction that the neighbor voxel lives in. If you
// need the direction FROM the neighbor voxel TO the current voxel, take
// negative of the given direction.
void EsdfOccIntegrator::getNeighborsAndDistances(
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

void EsdfOccIntegrator::getNeighbor(const BlockIndex& block_index,
                                    const VoxelIndex& voxel_index,
                                    const Eigen::Vector3i& direction,
                                    BlockIndex* neighbor_block_index,
                                    VoxelIndex* neighbor_voxel_index) const {
  DCHECK(neighbor_block_index != NULL);
  DCHECK(neighbor_voxel_index != NULL);

  *neighbor_block_index = block_index;
  *neighbor_voxel_index = voxel_index + direction;

  for (unsigned int i = 0; i < 3; ++i) {
    if ((*neighbor_voxel_index)(i) < 0) {
      (*neighbor_block_index)(i)--;
      (*neighbor_voxel_index)(i) += esdf_voxels_per_side_;
    } else if ((*neighbor_voxel_index)(i) >=
               static_cast<IndexElement>(esdf_voxels_per_side_)) {
      (*neighbor_block_index)(i)++;
      (*neighbor_voxel_index)(i) -= esdf_voxels_per_side_;
    }
  }
}

}  // namespace voxblox

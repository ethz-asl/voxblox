#include "voxblox/integrator/esdf_occ_fiesta_integrator.h"

namespace voxblox {

EsdfOccFiestaIntegrator::EsdfOccFiestaIntegrator(
    const Config& config, Layer<OccupancyVoxel>* occ_layer,
    Layer<EsdfVoxel>* esdf_layer)
    : config_(config), occ_layer_(occ_layer), esdf_layer_(esdf_layer) {
  CHECK_NOTNULL(occ_layer_);
  CHECK_NOTNULL(esdf_layer_);

  esdf_voxels_per_side_ = esdf_layer_->voxels_per_side();
  esdf_voxel_size_ = esdf_layer_->voxel_size();
}

// Main entrance
void EsdfOccFiestaIntegrator::updateFromOccLayer(bool clear_updated_flag) {
  BlockIndexList occ_blocks;
  occ_layer_->getAllUpdatedBlocks(Update::kEsdf, &occ_blocks);

  const bool kIncremental = true;
  updateFromOccBlocks(occ_blocks, kIncremental);

  if (clear_updated_flag) {
    for (const BlockIndex& block_index : occ_blocks) {
      if (occ_layer_->hasBlock(block_index)) {
        occ_layer_->getBlockByIndex(block_index)
            .setUpdated(Update::kEsdf, false);
      }
    }
  }
}

void EsdfOccFiestaIntegrator::updateFromOccBlocks(
    const BlockIndexList& occ_blocks, bool incremental) {
  CHECK_EQ(occ_layer_->voxels_per_side(), esdf_layer_->voxels_per_side());
  timing::Timer esdf_timer("esdf");

  // Go through all blocks in occupancy map (that are recently updated)
  // and copy their values for relevant voxels.
  timing::Timer allocate_timer("esdf/allocate_vox");
  VLOG(3) << "[ESDF update]: Propagating " << occ_blocks.size()
          << " updated blocks from the Occupancy.";

  for (const BlockIndex& block_index : occ_blocks) {
    Block<OccupancyVoxel>::ConstPtr occ_block =
        occ_layer_->getBlockPtrByIndex(block_index);
    if (!occ_block) {
      continue;
    }

    // Allocate the same block in the ESDF layer.
    // Block indices are the same across all layers.
    Block<EsdfVoxel>::Ptr esdf_block =
        esdf_layer_->allocateBlockPtrByIndex(block_index);
    esdf_block->setUpdatedAll();

    const size_t num_voxels_per_block = occ_block->num_voxels();
    for (size_t lin_index = 0u; lin_index < num_voxels_per_block; ++lin_index) {
      const OccupancyVoxel& occupancy_voxel =
          occ_block->getVoxelByLinearIndex(lin_index);
      // If this voxel is unobserved in the original map, skip it.
      // Initialization
      if (occupancy_voxel.observed) {
        EsdfVoxel& esdf_voxel = esdf_block->getVoxelByLinearIndex(lin_index);
        if (esdf_voxel.self_idx(0) == UNDEF)  // not yet assigned
        {
          // If the corresponding occ_voxel is observed,
          // then the esdf voxel is also observed
          esdf_voxel.observed = true;
          VoxelIndex voxel_index =
              esdf_block->computeVoxelIndexFromLinearIndex(lin_index);
          GlobalIndex global_index = getGlobalVoxelIndexFromBlockAndVoxelIndex(
              block_index, voxel_index, esdf_layer_->voxels_per_side());
          esdf_voxel.self_idx = global_index;
          esdf_voxel.behind = occupancy_voxel.behind;  // add signed
          esdf_voxel.distance = esdf_voxel.behind ? -config_.behind_surface_m
                                                  : config_.default_distance_m;
        }
      }
    }
  }

  getUpdateRange();
  setLocalRange();

  allocate_timer.Stop();
  updateESDF();
  esdf_timer.Stop();
}

void EsdfOccFiestaIntegrator::getUpdateRange() {
  // initialization
  update_range_min_ << UNDEF, UNDEF, UNDEF;
  update_range_max_ << -UNDEF, -UNDEF, -UNDEF;

  for (auto it = insert_list_.begin(); it != insert_list_.end(); it++) {
    GlobalIndex cur_vox_idx = *it;
    for (int j = 0; j <= 2; j++) {
      update_range_min_(j) = std::min(cur_vox_idx(j), update_range_min_(j));
      update_range_max_(j) = std::max(cur_vox_idx(j), update_range_max_(j));
    }
  }

  for (auto it = delete_list_.begin(); it != delete_list_.end(); it++) {
    GlobalIndex cur_vox_idx = *it;
    for (int j = 0; j <= 2; j++) {
      update_range_min_(j) = std::min(cur_vox_idx(j), update_range_min_(j));
      update_range_max_(j) = std::max(cur_vox_idx(j), update_range_max_(j));
    }
  }
}

void EsdfOccFiestaIntegrator::setLocalRange() {
  range_min_ = update_range_min_ - config_.range_boundary_offset;
  range_max_ = update_range_max_ + config_.range_boundary_offset;

  // Allocate memory for the local ESDF map
  // TODO: may have some issues here
  BlockIndex block_range_min, block_range_max;
  for (int i = 0; i <= 2; i++) {
    block_range_min(i) = range_min_(i) / esdf_voxels_per_side_;
    block_range_max(i) = range_max_(i) / esdf_voxels_per_side_;
  }

  for (int x = block_range_min(0); x <= block_range_max(0); x++) {
    for (int y = block_range_min(1); y <= block_range_max(1); y++) {
      for (int z = block_range_min(2); z <= block_range_max(2); z++) {
        BlockIndex cur_block_idx = BlockIndex(x, y, z);
        Block<EsdfVoxel>::Ptr esdf_block =
            esdf_layer_->allocateBlockPtrByIndex(cur_block_idx);
        esdf_block->setUpdatedAll();
      }
    }
  }
}

/* Delete idx from the doubly linked list
 * input:
 * occ_vox: head voxel of the list
 * cur_vox: the voxel need to be deleted
 */
void EsdfOccFiestaIntegrator::deleteFromList(EsdfVoxel* occ_vox,
                                             EsdfVoxel* cur_vox) {
  if (cur_vox->prev_idx(0) != UNDEF) {
    EsdfVoxel* prev_vox =
        esdf_layer_->getVoxelPtrByGlobalIndex(cur_vox->prev_idx);
    prev_vox->next_idx =
        cur_vox->next_idx;  // a <-> b <-> c , delete b, a <-> c
  } else
    occ_vox->head_idx = cur_vox->next_idx;  // b <-> c, b is already the head
  if (cur_vox->next_idx(0) != UNDEF) {
    EsdfVoxel* next_vox =
        esdf_layer_->getVoxelPtrByGlobalIndex(cur_vox->next_idx);
    next_vox->prev_idx = cur_vox->prev_idx;
  }
  cur_vox->next_idx = GlobalIndex(UNDEF, UNDEF, UNDEF);
  cur_vox->prev_idx = cur_vox->next_idx;
}

/* Insert idx to the doubly linked list at the head
 * input:
 * occ_vox: head voxel of the list
 * cur_vox: the voxel need to be insert
 */
void EsdfOccFiestaIntegrator::insertIntoList(EsdfVoxel* occ_vox,
                                             EsdfVoxel* cur_vox) {
  if (occ_vox->head_idx(0) == UNDEF)
    occ_vox->head_idx = cur_vox->self_idx;
  else {
    EsdfVoxel* head_occ_vox =
        esdf_layer_->getVoxelPtrByGlobalIndex(occ_vox->head_idx);
    head_occ_vox->prev_idx = cur_vox->self_idx;  // b <-> c to a <-> b <-> c
    cur_vox->next_idx = occ_vox->head_idx;
    occ_vox->head_idx = cur_vox->self_idx;
  }
}

// Main processing function of FIESTA
// Reference: Han. L, et al., Fast Incremental Euclidean Distance Fields for
// Online Motion Planning of Aerial Robots, IROS 2019 A mapping system called
// Fiesta is proposed to build global ESDF map incrementally. By introducing two
// independent updating queues for inserting and deleting obstacles separately,
// and using Indexing Data Structures and Doubly Linked Lists for map
// maintenance, our algorithm updates as few as possible nodes using a BFS
// framework. The ESDF mapping has high computational performance and produces
// near-optimal results. Code: https://github.com/HKUST-Aerial-Robotics/FIESTA

void EsdfOccFiestaIntegrator::updateESDF() {
  timing::Timer init_timer("esdf/update_init(alg2)");

  // TODO: Figure out how to write it into the config file
  // TODO: Change to 24 Neighborhood
  // const Connectivity k_connectivity = Connectivity::kTwentySix;

  // Algorithm 2: ESDF Updating Initialization
  while (!insert_list_.empty()) {
    GlobalIndex cur_vox_idx = *insert_list_.begin();
    insert_list_.erase(insert_list_.begin());
    // delete previous link & create a new linked-list
    EsdfVoxel* cur_vox = esdf_layer_->getVoxelPtrByGlobalIndex(cur_vox_idx);
    CHECK_NOTNULL(cur_vox);
    if (cur_vox->coc_idx(0) != UNDEF) {
      EsdfVoxel* coc_vox =
          esdf_layer_->getVoxelPtrByGlobalIndex(cur_vox->coc_idx);
      CHECK_NOTNULL(coc_vox);
      deleteFromList(coc_vox, cur_vox);
    }
    cur_vox->distance = 0.0f;
    cur_vox->coc_idx = cur_vox_idx;
    insertIntoList(cur_vox, cur_vox);
    update_queue_.push(cur_vox_idx);
  }

  while (!delete_list_.empty()) {
    GlobalIndex cur_vox_idx = *delete_list_.begin();
    delete_list_.erase(delete_list_.begin());
    EsdfVoxel* cur_vox = esdf_layer_->getVoxelPtrByGlobalIndex(cur_vox_idx);
    CHECK_NOTNULL(cur_vox);

    GlobalIndex next_vox_idx = GlobalIndex(UNDEF, UNDEF, UNDEF);
    // for each voxel in current voxel's doubly linked list
    for (GlobalIndex temp_vox_idx = cur_vox->head_idx; temp_vox_idx(0) != UNDEF;
         temp_vox_idx = next_vox_idx) {
      EsdfVoxel* temp_vox = esdf_layer_->getVoxelPtrByGlobalIndex(temp_vox_idx);
      CHECK_NOTNULL(temp_vox);
      deleteFromList(cur_vox, temp_vox);

      temp_vox->coc_idx = GlobalIndex(UNDEF, UNDEF, UNDEF);
      if (voxInRange(temp_vox_idx)) {
        // temp_vox->distance = config_.default_distance_m;
        temp_vox->distance = INF;

        // Get the global indices of neighbors.
        Neighborhood<>::IndexMatrix nbr_voxs_idx;
        Neighborhood<>::getFromGlobalIndex(temp_vox_idx, &nbr_voxs_idx);

        // Go through the neighbors and see if we can update any of them.
        for (unsigned int idx = 0u; idx < nbr_voxs_idx.cols(); ++idx) {
          GlobalIndex nbr_vox_idx = nbr_voxs_idx.col(idx);
          if (voxInRange(nbr_vox_idx)) {
            EsdfVoxel* nbr_vox =
                esdf_layer_->getVoxelPtrByGlobalIndex(nbr_vox_idx);
            CHECK_NOTNULL(nbr_vox);
            GlobalIndex nbr_coc_vox_idx = nbr_vox->coc_idx;
            if (nbr_vox->observed && nbr_coc_vox_idx(0) != UNDEF) {
              OccupancyVoxel* nbr_coc_occ_vox =
                  occ_layer_->getVoxelPtrByGlobalIndex(nbr_coc_vox_idx);
              CHECK_NOTNULL(nbr_coc_occ_vox);
              // Next line is neccessary, to check if the closest
              // occupied voxel is still occupied
              if (nbr_coc_occ_vox->occupied) {
                float temp_dist = dist(nbr_coc_vox_idx, temp_vox_idx);
                if (temp_dist < std::abs(temp_vox->distance)) {
                  temp_vox->distance =
                      temp_vox->behind ? -temp_dist : temp_dist;
                  temp_vox->coc_idx = nbr_coc_vox_idx;
                }
                // break; //TODO: may be fixed by the patch code (algorithm 3)
              }
            }
          }
        }
      }
      // Destroy the linked-list
      temp_vox->prev_idx = GlobalIndex(UNDEF, UNDEF, UNDEF);
      next_vox_idx = temp_vox->next_idx;
      temp_vox->next_idx = GlobalIndex(UNDEF, UNDEF, UNDEF);

      if (temp_vox->coc_idx(0) != UNDEF) {
        update_queue_.push(temp_vox_idx);
        EsdfVoxel* temp_coc_vox =
            esdf_layer_->getVoxelPtrByGlobalIndex(temp_vox->coc_idx);
        CHECK_NOTNULL(temp_coc_vox);
        insertIntoList(temp_coc_vox, temp_vox);
      }
    }
    cur_vox->head_idx = GlobalIndex(UNDEF, UNDEF, UNDEF);
  }
  init_timer.Stop();
  // End of Algorithm 2
  // LOG(INFO)<<"Update queue's size: ["<<update_queue_.size()<< "]";

  timing::Timer update_timer("esdf/update(alg1)");
  // Algorithm 1 ESDF updating (BFS based on priority queue)
  int times = 0, change_num = 0;
  while (!update_queue_.empty()) {
    GlobalIndex cur_vox_idx = update_queue_.front();
    update_queue_.pop();
    EsdfVoxel* cur_vox = esdf_layer_->getVoxelPtrByGlobalIndex(cur_vox_idx);
    CHECK_NOTNULL(cur_vox);
    // TODO: this is a dirty fix, figure out why there's the SEGFAULT later
    if (cur_vox->coc_idx(0) == UNDEF) continue;
    EsdfVoxel* coc_vox =
        esdf_layer_->getVoxelPtrByGlobalIndex(cur_vox->coc_idx);
    CHECK_NOTNULL(coc_vox);

    times++;
    total_expanding_times_++;
    // timing::Timer patch_timer("esdf/patch(alg3)");
    // Algorithm 3 Patch Code (why do we need it)
    bool change_flag = false;  // indicate if the patch works
    // Get the global indices of neighbors.
    Neighborhood<>::IndexMatrix nbr_voxs_idx;
    Neighborhood<>::getFromGlobalIndex(cur_vox_idx, &nbr_voxs_idx);

    // Go through the neighbors and see if we can update any of them.
    for (unsigned int idx = 0u; idx < nbr_voxs_idx.cols(); ++idx) {
      GlobalIndex nbr_vox_idx = nbr_voxs_idx.col(idx);
      if (voxInRange(nbr_vox_idx)) {
        EsdfVoxel* nbr_vox = esdf_layer_->getVoxelPtrByGlobalIndex(nbr_vox_idx);
        CHECK_NOTNULL(nbr_vox);
        if (nbr_vox->observed) {
          float temp_dist = dist(nbr_vox->coc_idx, cur_vox_idx);
          if (temp_dist < std::abs(cur_vox->distance)) {
            cur_vox->distance = cur_vox->behind ? -temp_dist : temp_dist;
            deleteFromList(coc_vox, cur_vox);
            cur_vox->coc_idx = nbr_vox->coc_idx;
            if (cur_vox->coc_idx(0) != UNDEF) {
              EsdfVoxel* cur_coc_vox =
                  esdf_layer_->getVoxelPtrByGlobalIndex(cur_vox->coc_idx);
              CHECK_NOTNULL(cur_coc_vox);
              insertIntoList(cur_coc_vox, cur_vox);
            }
            change_flag = true;
          }
        }
      }
    }
    if (change_flag) {
      change_num++;
      update_queue_.push(cur_vox_idx);
      continue;
    }
    // patch_timer.Stop();
    // End of Algorithm 3

    for (unsigned int idx = 0u; idx < nbr_voxs_idx.cols(); ++idx) {
      GlobalIndex nbr_vox_idx = nbr_voxs_idx.col(idx);
      // check if this index is in the range
      if (voxInRange(nbr_vox_idx)) {
        EsdfVoxel* nbr_vox = esdf_layer_->getVoxelPtrByGlobalIndex(nbr_vox_idx);
        CHECK_NOTNULL(nbr_vox);
        if (nbr_vox->observed) {
          float temp_dist = dist(cur_vox->coc_idx, nbr_vox_idx);
          if (temp_dist < std::abs(nbr_vox->distance)) {
            nbr_vox->distance = nbr_vox->behind ? -temp_dist : temp_dist;
            if (nbr_vox->coc_idx(0) != UNDEF) {
              EsdfVoxel* nbr_coc_vox =
                  esdf_layer_->getVoxelPtrByGlobalIndex(nbr_vox->coc_idx);
              CHECK_NOTNULL(nbr_coc_vox);
              deleteFromList(nbr_coc_vox, nbr_vox);
            }
            nbr_vox->coc_idx = cur_vox->coc_idx;
            insertIntoList(coc_vox, nbr_vox);
            update_queue_.push(nbr_vox_idx);
          }
        }
      }
    }
  }
  update_timer.Stop();
  // LOG(INFO)<<"Alg 1 done";
  // End of Algorithm 1

  // LOG(INFO) << "FIESTA: expanding [" << times << "] nodes,
  // with [" << change_num << "] changes by the patch,
  // up-to-now [" << total_expanding_times_<< "] nodes";
}

inline float EsdfOccFiestaIntegrator::dist(GlobalIndex vox_idx_a,
                                           GlobalIndex vox_idx_b) {
  return (vox_idx_b - vox_idx_a).cast<float>().norm() * esdf_voxel_size_;
  // TODO: may use square root & * resolution_ at last together to speed up
}

inline bool EsdfOccFiestaIntegrator::voxInRange(GlobalIndex vox_idx) {
  return (vox_idx(0) >= range_min_(0) && vox_idx(0) <= range_max_(0) &&
          vox_idx(1) >= range_min_(1) && vox_idx(1) <= range_max_(1) &&
          vox_idx(2) >= range_min_(2) && vox_idx(2) <= range_max_(2));
}

void EsdfOccFiestaIntegrator::loadInsertList(
    const GlobalIndexList& insert_list) {
  insert_list_ = insert_list;
}

void EsdfOccFiestaIntegrator::loadDeleteList(
    const GlobalIndexList& delete_list) {
  delete_list_ = delete_list;
}

}  // namespace voxblox

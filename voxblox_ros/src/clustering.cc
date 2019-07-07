#include <voxblox_ros/clustering.h>

namespace voxblox {

Clustering::Clustering(const std::shared_ptr<TsdfMap> input_map, float cluster_distance_threshold,
                       unsigned int cluster_match_vote_threshold, unsigned int cluster_min_size_threshold) {
  current_oneshot_map_.reset(new TsdfMap(input_map->getTsdfLayer()));
  current_clusters_.clear();
  map_clusters_.clear();
  used_indices_.clear();
  for(int i = 0; i<(colors.size()-1); i++) used_colors_[i] = 0;
  cluster_distance_threshold_ = cluster_distance_threshold * current_oneshot_map_->getTsdfLayerPtr()->voxel_size() / 2; //max positive tsdf distance for voxel to be in cluster
  cluster_match_vote_threshold_ = cluster_match_vote_threshold; //number of voxels which need to have voted for a cluster for a match
  cluster_min_size_threshold_ = cluster_min_size_threshold;
  voxels_per_side_ = current_oneshot_map_->getTsdfLayerPtr()->voxels_per_side();
  num_voxels_per_block_ = voxels_per_side_ * voxels_per_side_ * voxels_per_side_;
}

void Clustering::addCurrentMap(const std::shared_ptr<TsdfMap> input_map) {
  current_oneshot_map_.reset(new TsdfMap(input_map->getTsdfLayer()));
  current_clusters_.clear();
  Neighborhood<kEighteen>::IndexMatrix neighbors;

  BlockIndexList blocks;
  current_oneshot_map_->getTsdfLayerPtr()->getAllAllocatedBlocks(&blocks);

  for (const BlockIndex& index : blocks) {
    const Block<TsdfVoxel>& block = current_oneshot_map_->getTsdfLayerPtr()->getBlockByIndex(index);
    for (size_t linear_index = 0; linear_index < num_voxels_per_block_;
         ++linear_index) {
      TsdfVoxel voxel = block.getVoxelByLinearIndex(linear_index);
      if (voxel.weight != 0 && voxel.distance < cluster_distance_threshold_) {
        LongIndexSet current_cluster;
        current_cluster.clear();
        VoxelIndex voxel_index = block.computeVoxelIndexFromLinearIndex(linear_index);
        GlobalIndex global_voxel_index = getGlobalVoxelIndexFromBlockAndVoxelIndex(index, voxel_index, voxels_per_side_);
        current_cluster.insert(global_voxel_index);
        voxel.weight = 0;
        std::queue<GlobalIndex> global_index_voxel_queue;
        global_index_voxel_queue.push(global_voxel_index);
        while(!global_index_voxel_queue.empty()) {
          GlobalIndex cluster_voxel_gi = global_index_voxel_queue.front();
          Neighborhood<kEighteen>::getFromGlobalIndex(cluster_voxel_gi, &neighbors);
          for (unsigned int i = 0; i < Connectivity::kTwentySix; i++) {
            GlobalIndex neighbor_global_index = neighbors.col(i);
            TsdfVoxel* neighbor_voxel_ptr = current_oneshot_map_->getTsdfLayerPtr()->getVoxelPtrByGlobalIndex(neighbor_global_index);
            if (neighbor_voxel_ptr == nullptr) continue;
            if (neighbor_voxel_ptr->weight != 0 && neighbor_voxel_ptr->distance < cluster_distance_threshold_) {
              current_cluster.insert(neighbor_global_index);
              global_index_voxel_queue.push(neighbor_global_index);
              neighbor_voxel_ptr->weight = 0;
            }
          }
          global_index_voxel_queue.pop();
        }
        if (current_cluster.size() > cluster_min_size_threshold_) {
          ColoredDynamicCluster current_colored_cluster;
          current_colored_cluster = {current_cluster , Color::White(), false, 0};
          current_clusters_.push_back(current_colored_cluster);
        }
      }
    }
  }
  ROS_INFO("current cluster size = %u", current_clusters_.size());
}

void Clustering::matchCommunClusters() {
	const int size = map_clusters_.size();
  ColoredDynamicCluster* base_map_color_cluster = nullptr;
  std::list<int> seen_indices;
  seen_indices.clear();


	// looping through all current_clusters_
	for (ColoredDynamicCluster& current_color_cluster : current_clusters_) {
    unsigned int* map_cluster_vote_array = nullptr;
    LongIndexSet& current_cluster = current_color_cluster.cluster;
    if (size != 0){
      map_cluster_vote_array = new unsigned int[size];
	    for (int i = 0; i < size; i++) map_cluster_vote_array[i] = 0;
      //Vote of current voxels for a map_cluster
      for (auto current_voxel_global_index = current_cluster.begin(); current_voxel_global_index != current_cluster.end(); current_voxel_global_index++) {
        int cluster_count = 0;
        for (const ColoredDynamicCluster& map_color_cluster : map_clusters_) {
          const LongIndexSet& map_cluster = map_color_cluster.cluster;
          if (map_cluster.find(*current_voxel_global_index) != map_cluster.end()){
            map_cluster_vote_array[cluster_count] += 1;
            break;
          }
          cluster_count += 1;
        }
      }
    }

    //matching current_cluster to map_cluster
		std::list<ColoredDynamicCluster*> matched_map_clusters;
		matched_map_clusters.clear();
    unsigned int max_votes = 0;
    auto map_cluster_list_iterator = map_clusters_.begin();
		for (int i = 0; i < size; i++) {
			if (map_cluster_vote_array[i] >= cluster_match_vote_threshold_) {
				matched_map_clusters.push_back(&(*map_cluster_list_iterator));
        if (map_cluster_vote_array[i] > max_votes) {
          base_map_color_cluster = &(*map_cluster_list_iterator);
          max_votes = map_cluster_vote_array[i];
        }
			}
      map_cluster_list_iterator++;
		}

    ROS_INFO("matched_map_cluster_count: %u", matched_map_clusters.size());

    // If no matching has been made for this current_cluster
    if (matched_map_clusters.size() == 0) {
      ROS_INFO("no match to current cluster");
      current_color_cluster.index = findIndex();
      current_color_cluster.color = findColor();
      map_clusters_.push_back(current_color_cluster);
      for (const GlobalIndex current_voxel_global_index : current_cluster){
        current_FOV_voxels_.erase(current_voxel_global_index);
      }
      continue;
    }

    LongIndexSet& base_map_cluster = base_map_color_cluster->cluster;
    // In case the map cluster is devided in several clusters
    if (matched_map_clusters.size() > 1) {
      ROS_INFO("several map clusters matched to current cluster");
      for (auto matched_map_color_cluster = matched_map_clusters.begin(); matched_map_color_cluster != matched_map_clusters.end(); matched_map_color_cluster++ ) {
        if ((*matched_map_color_cluster)->index == base_map_color_cluster->index) continue;
        if ((*matched_map_color_cluster)->dynamic) {
            base_map_color_cluster->dynamic = true;
        }
        LongIndexSet* matched_map_cluster = &(*matched_map_color_cluster)->cluster;
        for (auto voxel_global_index = matched_map_cluster->begin(); voxel_global_index != matched_map_cluster->end(); voxel_global_index++) {
          base_map_cluster.insert(*voxel_global_index);
        }
        used_indices_.remove((*matched_map_color_cluster)->index);
        int counter = 0;
        for (auto color : colors){
          if ((*matched_map_color_cluster)->color == color) {
            used_colors_[counter]--;
            break;
          }
          counter++;
        }
        ColoredDynamicCluster temp_cluster = **matched_map_color_cluster;
        matched_map_color_cluster = matched_map_clusters.erase(matched_map_color_cluster);
        map_clusters_.remove(temp_cluster);
      }
    }

    //Checking if a matched_map_cluster has already been matched by another current_cluster
    /*bool double_match_xor_active = false;
    for (ColoredDynamicCluster& processed_current_cluster: current_clusters_) {
      if (processed_current_cluster == current_color_cluster) break;
      if (processed_current_cluster.index == base_map_color_cluster->index) {
        if (processed_current_cluster.dynamic != current_color_cluster.dynamic) { //XOR
          double_match_xor_active = true;
          ROS_INFO("double match XOR active");
          ColoredDynamicCluster* dynamic_cluster;
          ColoredDynamicCluster* static_cluster;
          if (current_color_cluster.dynamic) {
            dynamic_cluster = &current_color_cluster;
            static_cluster = &processed_current_cluster;
          }
          else {
            dynamic_cluster = &processed_current_cluster;
            static_cluster = &current_color_cluster;
          }
          static_cluster->index = base_map_color_cluster->index;
          static_cluster->color = base_map_color_cluster->color;
          for (GlobalIndex matched_map_voxel_idx : base_map_cluster) {
            if (dynamic_cluster->cluster.find(matched_map_voxel_idx) == dynamic_cluster->cluster.end()){
              static_cluster->cluster.insert(matched_map_voxel_idx);
            }
          }
          map_clusters_.push_back(*static_cluster);
          dynamic_cluster->index = findIndex();
          dynamic_cluster->color = findColor();
          map_clusters_.push_back(*dynamic_cluster);
          used_indices_.remove(base_map_color_cluster->index);
          int counter = 0;
          for (auto color : colors){
            if (base_map_color_cluster->color == color) {
              used_colors_[counter]--;
              break;
            }
            counter++;
          }

          map_clusters_.remove(*base_map_color_cluster);
          seen_indices.push_back(current_color_cluster.index);
          for (GlobalIndex current_voxel : current_color_cluster.cluster) {
            current_FOV_voxels_.erase(current_voxel);
          }
          delete map_cluster_vote_array;
          ROS_INFO("test 1");
          break;
        }
      }
    }
    ROS_INFO("test 2");
    if (double_match_xor_active) continue;
    // Combining clusters if a matched_current_cluster has already been matched by another map_cluster
    Color color_from_map = colors[0];
    bool dynamic_tag_from_map = false;
    while (!double_matched_current_clusters.empty()) {
      ColoredDynamicCluster* previously_computed_double_matched_current_cluster = double_matched_current_clusters.front();
      for (GlobalIndex previously_computed_current_voxel : previously_computed_double_matched_current_cluster->cluster) {
        base_current_cluster->insert(previously_computed_current_voxel);
      }
      if (previously_computed_double_matched_current_cluster->cluster.size() == max_double_matched_size) color_from_map = previously_computed_double_matched_current_cluster->color;
      color_from_map = previously_computed_double_matched_current_cluster->color;
      if (!dynamic_tag_from_map) dynamic_tag_from_map = previously_computed_double_matched_current_cluster->dynamic;
      current_clusters_.remove(*previously_computed_double_matched_current_cluster);
      ROS_INFO("double matched cluster removed");
      double_matched_current_clusters.pop();
    }*/


    seen_indices.push_back(base_map_color_cluster->index);
    current_color_cluster.index = base_map_color_cluster->index;
    if (!base_map_color_cluster->dynamic) base_map_color_cluster->dynamic = current_color_cluster.dynamic;
    for (GlobalIndex current_voxel : current_cluster) {
      base_map_color_cluster->cluster.insert(current_voxel);
      current_FOV_voxels_.erase(current_voxel);
    }
    delete map_cluster_vote_array;
    ROS_INFO("going to next current_Cluster");
	}
  for (GlobalIndex non_clustered_voxel : current_FOV_voxels_) {
    for (ColoredDynamicCluster& map_cluster : map_clusters_) {
      if(map_cluster.cluster.find(non_clustered_voxel) != map_cluster.cluster.end()){
        map_cluster.cluster.erase(non_clustered_voxel);
        break;
      }
    }
  }
  for (std::list<ColoredDynamicCluster>::iterator map_cluster = map_clusters_.begin(); map_cluster != map_clusters_.end(); map_cluster++) {
    ROS_INFO("map cluster: index %u, color r %u g %u b %u", map_cluster->index, map_cluster->color.r, map_cluster->color.g, map_cluster->color.b);
    // removing not seen dynamic map clusters
    if (map_cluster->dynamic) {
      bool seen = false;
      for (int seen_index : seen_indices) {
        if(map_cluster->index == seen_index) {
          seen = true;
          break;
        }
      }
      if (!seen) {
        used_indices_.remove(map_cluster->index);
        for (int i = 1; i < (colors.size() - 1); i++) {
          if (colors[i] == map_cluster->color) used_colors_[i]--;
        }
        map_cluster = map_clusters_.erase(map_cluster);
        continue;
      }
    }
    // removing too small map clusters
    if (map_cluster->cluster.size() < cluster_min_size_threshold_) {
      used_indices_.remove(map_cluster->index);
      for (int i = 1; i < (colors.size()-1); i++) {
        if (colors[i] == map_cluster->color) used_colors_[i]--;
      }
      map_cluster = map_clusters_.erase(map_cluster);
    }
  }
  ROS_INFO("map clusters size: %u", map_clusters_.size());
}

void Clustering::determineFOV() {
  BlockIndexList blocks_current;
  current_oneshot_map_->getTsdfLayerPtr()->getAllAllocatedBlocks(&blocks_current);
  for (const BlockIndex& index : blocks_current) {
    // Iterate over all voxels in current blocks.
    Block<TsdfVoxel>& block_current = current_oneshot_map_->getTsdfLayerPtr()->getBlockByIndex(index);
    for (size_t linear_index = 0; linear_index < num_voxels_per_block_; ++linear_index) {
      TsdfVoxel& voxel = block_current.getVoxelByLinearIndex(linear_index);
      if(voxel.weight != 0) {
        GlobalIndex voxel_global_idx = getGlobalVoxelIndexFromBlockAndVoxelIndex(index, block_current.computeVoxelIndexFromLinearIndex(linear_index), voxels_per_side_);
        current_FOV_voxels_.insert(voxel_global_idx);
      }
    }
  }
  ROS_INFO("current FOV voxels size: %u", current_FOV_voxels_.size());
  //erasing dynamic voxels currently not seen
  for (std::list<ColoredDynamicCluster>::iterator map_cluster = map_clusters_.begin(); map_cluster != map_clusters_.end(); map_cluster++) {
    if (map_cluster->dynamic){
      for (GlobalIndex voxel_idx : map_cluster->cluster) {
        if (current_FOV_voxels_.find(voxel_idx) == current_FOV_voxels_.end()) {
          map_cluster->cluster.erase(voxel_idx);
        }
      }
      if (map_cluster->cluster.size() < cluster_min_size_threshold_) {
        used_indices_.remove(map_cluster->index);
        for (int i = 1; i < (colors.size()-1); i++) {
          if (colors[i] == map_cluster->color) used_colors_[i]--;
        }
        map_cluster = map_clusters_.erase(map_cluster);
      }
    }
  }
}

pcl::PointCloud<pcl::PointXYZRGB> Clustering::matchedClusterVisualiser() {
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  pointcloud.clear();
  for (ColoredDynamicCluster colored_cluster : current_clusters_) {
    LongIndexSet cluster = colored_cluster.cluster;
    Color color = colored_cluster.color;
    for (auto voxel_global_index = cluster.begin(); voxel_global_index != cluster.end(); ++voxel_global_index) {
      BlockIndex block_index ;
      VoxelIndex voxel_index ;
      getBlockAndVoxelIndexFromGlobalVoxelIndex(*voxel_global_index, voxels_per_side_, &block_index, &voxel_index);
      const Block<TsdfVoxel>& block = current_oneshot_map_->getTsdfLayerPtr()->getBlockByIndex(block_index);
      Point voxel_coord = block.computeCoordinatesFromVoxelIndex(voxel_index);
      pcl::PointXYZRGB point;
      point.x = voxel_coord.x();
      point.y = voxel_coord.y();
      point.z = voxel_coord.z();
      point.r = color.r;
      point.g = color.g;
      point.b = color.b;
      pointcloud.push_back(point);
    }
  }
  return pointcloud;
}

void Clustering::mapClusterVisualiser(pcl::PointCloud<pcl::PointXYZRGB>* dynamic_pointcloud,
                                      pcl::PointCloud<pcl::PointXYZRGB>* static_pointcloud,
                                      std::shared_ptr<TsdfMap> tsdf_map){
  dynamic_pointcloud->clear();
  static_pointcloud->clear();
  for (const ColoredDynamicCluster& colored_cluster : map_clusters_) {
    const LongIndexSet& cluster = colored_cluster.cluster;
    Color color = colored_cluster.color;
    for (auto voxel_global_index = cluster.begin(); voxel_global_index != cluster.end(); ++voxel_global_index) {
      BlockIndex block_index ;
      VoxelIndex voxel_index ;
      //ROS_INFO("test_2");
      getBlockAndVoxelIndexFromGlobalVoxelIndex(*voxel_global_index, voxels_per_side_, &block_index, &voxel_index);
      const Block<TsdfVoxel>& block = tsdf_map->getTsdfLayerPtr()->getBlockByIndex(block_index);
      Point voxel_coord = block.computeCoordinatesFromVoxelIndex(voxel_index);
      pcl::PointXYZRGB point;
      point.x = voxel_coord.x();
      point.y = voxel_coord.y();
      point.z = voxel_coord.z();
      point.r = color.r;
      point.g = color.g;
      point.b = color.b;
      if(colored_cluster.dynamic){
        dynamic_pointcloud->push_back(point);
      } else {
        static_pointcloud->push_back(point);
      }
    }
  }
}


int Clustering::findIndex() {
  int index_counter = 1;
  if (!used_indices_.empty()){
    bool to_use = true;
    while(to_use){
      for (auto used_index : used_indices_) {
        if(used_index == index_counter) {
          to_use = false;
        }
      }
      if (!to_use) {
        index_counter++;
        to_use = true;
      } else break;
    }
  }
  used_indices_.push_back(index_counter);
  used_indices_.sort();
  return index_counter;
}

Color Clustering::findColor() {
  int min_val = 999;
  int found_iterator = 0;
  for (int i = 1; i<colors.size() ; i++){
    if (used_colors_[i]< min_val) {
      min_val = used_colors_[i];
      found_iterator = i;
    }
  }
  used_colors_[found_iterator]++;
  return colors[found_iterator];
}

} //end of namespace

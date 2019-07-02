#include <voxblox_ros/clustering.h>

namespace voxblox {

Clustering::Clustering(const std::shared_ptr<TsdfMap> input_map, float cluster_distance_threshold, 
                       unsigned int cluster_match_vote_threshold, unsigned int cluster_min_size_threshold,
                       unsigned int clustering_queue_size) {
  current_map_.reset(new TsdfMap(input_map->getTsdfLayer()));
  cluster_distance_threshold_ = cluster_distance_threshold * current_map_->getTsdfLayerPtr()->voxel_size() / 2;
  cluster_match_vote_threshold_ = cluster_match_vote_threshold; //number of voxels which need to have voted for a cluster for a match
  cluster_min_size_threshold_ = cluster_min_size_threshold;
  clustering_queue_size_ = clustering_queue_size;
  voxels_per_side_ = current_map_->getTsdfLayerPtr()->voxels_per_side();
  num_voxels_per_block_ = voxels_per_side_ * voxels_per_side_ * voxels_per_side_;
}

void Clustering::addCurrentMap(const std::shared_ptr<TsdfMap> input_map) {
  current_map_.reset(new TsdfMap(input_map->getTsdfLayer()));
  std::list<ColoredDynamicCluster> cluster_list;
  cluster_list.clear();
  Neighborhood<kEighteen>::IndexMatrix neighbors;

  BlockIndexList blocks;
  current_map_->getTsdfLayerPtr()->getAllAllocatedBlocks(&blocks);

  for (const BlockIndex& index : blocks) {
    const Block<TsdfVoxel>& block = current_map_->getTsdfLayerPtr()->getBlockByIndex(index);
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
            TsdfVoxel* neighbor_voxel_ptr = current_map_->getTsdfLayerPtr()->getVoxelPtrByGlobalIndex(neighbor_global_index);
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
          if (cluster_queue_.size() < (clustering_queue_size_ - 1)) {
            Color color = colors[cluster_list.size() + 1];
            current_colored_cluster = {current_cluster , color, false};
          }
          else current_colored_cluster = {current_cluster , Color::White(), false};

          cluster_list.push_back(current_colored_cluster);
        }
      }
    }
  }
  cluster_queue_.push(cluster_list);
}

void Clustering::matchCommunClusters() {
 std::list<ColoredDynamicCluster>* current_clusters = &(cluster_queue_.back());
 std::list<ColoredDynamicCluster>* old_clusters = &(cluster_queue_.front());

	const int size = current_clusters->size();

  bool* used_colors = new bool[9];
  for (int i = 0; i<9; i++) used_colors[i] = false;
	
	for (ColoredDynamicCluster old_color_cluster : *old_clusters) {
    unsigned int* current_cluster_vote_array = new unsigned int[size];
	  for (int i = 0; i < size; i++) current_cluster_vote_array[i] = 0;
    LongIndexSet old_cluster = old_color_cluster.cluster;
    //Vote of old voxels for a current_cluster
		for (auto old_voxel_global_index = old_cluster.begin(); old_voxel_global_index != old_cluster.end(); old_voxel_global_index++) {
			//Point voxel_coord = voxel_element.coord;
			int cluster_count = 0;
			for (const ColoredDynamicCluster current_color_cluster : *current_clusters) {
        LongIndexSet current_cluster = current_color_cluster.cluster;
        if (current_cluster.find(*old_voxel_global_index) != current_cluster.end()){
          current_cluster_vote_array[cluster_count] += 1;
          break;
        }
				cluster_count += 1;
			}
		}
    //matching current_cluster to old_cluster
		std::list<ColoredDynamicCluster*> matched_current_clusters;
		matched_current_clusters.clear();
    int max_votes = 0;
    ColoredDynamicCluster* base_current_color_cluster;
    auto current_cluster_list_iterator = current_clusters->begin();
		for (int i = 0; i < size; i++) {
			if (current_cluster_vote_array[i] >= cluster_match_vote_threshold_) {
				matched_current_clusters.push_back(&(*current_cluster_list_iterator));
        if (current_cluster_vote_array[i] > max_votes) {
          base_current_color_cluster = &(*current_cluster_list_iterator);
          max_votes = current_cluster_vote_array[i];
        }
			}
      current_cluster_list_iterator++;
		}
    LongIndexSet* base_current_cluster = &(base_current_color_cluster->cluster);

    ROS_INFO("matched_current_cluster_count: %u", matched_current_clusters.size());

    // If no matching has been made for this old_cluster
    if (matched_current_clusters.size() == 0) continue;

    // Checking if a matched_current_cluster has already been matched by another old_cluster
    std::queue<ColoredDynamicCluster*> double_matched_current_clusters;
    unsigned int max_double_matched_size = 0;
    for (auto matched_current_cluster = matched_current_clusters.begin(); matched_current_cluster != matched_current_clusters.end(); matched_current_cluster++ ) {
      GlobalIndex matched_current_cluster_voxel = *((*matched_current_cluster)->cluster).begin();
      for (auto current_color_cluster = current_clusters->begin(); current_color_cluster != current_clusters->end(); current_color_cluster++) {
        if (current_color_cluster->color != colors[0] && current_color_cluster->cluster.find(matched_current_cluster_voxel) != current_color_cluster->cluster.end()) {
          double_matched_current_clusters.push(&(*current_color_cluster));
          if (current_color_cluster->cluster.size() > max_double_matched_size) max_double_matched_size = current_color_cluster->cluster.size();
          ROS_INFO("double match activated");
        }
      }
    }

    // In case the current cluster is devided in several clusters
    if (matched_current_clusters.size() > 1) {
      for (auto matched_current_color_cluster = matched_current_clusters.begin(); matched_current_color_cluster != matched_current_clusters.end(); matched_current_color_cluster++ ) {
        if (*matched_current_color_cluster == base_current_color_cluster) continue;
        LongIndexSet* matched_current_cluster = &(*matched_current_color_cluster)->cluster;
        for (auto voxel_global_index = matched_current_cluster->begin(); voxel_global_index != matched_current_cluster->end(); voxel_global_index++) {
          base_current_cluster->insert(*voxel_global_index);
        }
        current_clusters->remove(**matched_current_color_cluster); //erase(iterator)
      }
    }
    // Combining clusters if a matched_current_cluster has already been matched by another old_cluster
    Color color_from_old = colors[0];
    bool dynamic_tag_from_old = false;
    while (!double_matched_current_clusters.empty()) {
      ColoredDynamicCluster* previously_computed_double_matched_current_cluster = double_matched_current_clusters.front();
      for (GlobalIndex previously_computed_current_voxel : previously_computed_double_matched_current_cluster->cluster) {
        base_current_cluster->insert(previously_computed_current_voxel);
      }
      if (previously_computed_double_matched_current_cluster->cluster.size() > max_double_matched_size) color_from_old = previously_computed_double_matched_current_cluster->color;
      color_from_old = previously_computed_double_matched_current_cluster->color;
      if (!dynamic_tag_from_old) dynamic_tag_from_old = previously_computed_double_matched_current_cluster->dynamic; 
      current_clusters->remove(*previously_computed_double_matched_current_cluster);
      double_matched_current_clusters.pop();
    }
    if (color_from_old != colors[0]) {
      base_current_color_cluster->color = color_from_old;
      base_current_color_cluster->dynamic = dynamic_tag_from_old;
    } else {
      base_current_color_cluster->color = old_color_cluster.color;
      base_current_color_cluster->dynamic = old_color_cluster.dynamic;
    }
    if (base_current_color_cluster->color == Color::Red()){
      ROS_INFO("red");
    } else if (base_current_color_cluster->color == Color::Green()){
      ROS_INFO("green");
    } else if (base_current_color_cluster->color == Color::Blue()){
      ROS_INFO("blue");
    } else if (base_current_color_cluster->color == Color::White()){
      ROS_INFO("white");
    }
    for (int i = 0; i < 9; i++){
      if(base_current_color_cluster->color == colors[i]) used_colors[i] = true;
    }
    delete current_cluster_vote_array;
	}
  // check for current clusters with color white -> given them an unused color
  for (auto current_color_cluster = current_clusters->begin(); current_color_cluster != current_clusters->end(); current_color_cluster++) {
    if (current_color_cluster->color == Color::White()){
      for (int i = 1; i<9 ; i++){
        if(!used_colors[i]) {
          current_color_cluster->color = colors[i];
          used_colors[i] = true;
          break;
        }
      }
      if (current_color_cluster->color == Color::Red()){
        ROS_INFO("red");
      } else if (current_color_cluster->color == Color::Green()){
        ROS_INFO("green");
      } else if (current_color_cluster->color == Color::Blue()){
        ROS_INFO("blue");
      } else if (current_color_cluster->color == Color::White()){
        ROS_INFO("white");
      }
    }
  }
  delete used_colors;
}

pcl::PointCloud<pcl::PointXYZRGB> Clustering::matchedClusterVisualiser() {
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  pointcloud.clear();
  std::list<ColoredDynamicCluster>* current_clusters = &(cluster_queue_.back());
  for (ColoredDynamicCluster colored_cluster : *current_clusters) {
    LongIndexSet cluster = colored_cluster.cluster;
    Color color = colored_cluster.color;
    for (auto voxel_global_index = cluster.begin(); voxel_global_index != cluster.end(); ++voxel_global_index) {
      BlockIndex block_index ;
      VoxelIndex voxel_index ;
      getBlockAndVoxelIndexFromGlobalVoxelIndex(*voxel_global_index, voxels_per_side_, &block_index, &voxel_index);
      const Block<TsdfVoxel>& block = current_map_->getTsdfLayerPtr()->getBlockByIndex(block_index);
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
} //end of namespace
#include <voxblox_ros/dynamic_recognizer.h>

namespace voxblox {

DynamicRecognizer::DynamicRecognizer(const std::shared_ptr<TsdfMap> input_map, float delta_distance_threshold, float dynamic_share_threshold){
  voxels_per_side_ = input_map->getTsdfLayerPtr()->voxels_per_side();
  num_voxels_per_block_ = voxels_per_side_ * voxels_per_side_ * voxels_per_side_;
  delta_distance_threshold_ = delta_distance_threshold;
  dynamic_share_threshold_ = dynamic_share_threshold;
  for(int i = 0; i<(colors.size()-1); i++) used_colors_[i] = 0;
}

void DynamicRecognizer::addCurrentMap(const std::shared_ptr<TsdfMap> input_map) {
  tsdf_ptr_queue_.push(input_map);
}

void DynamicRecognizer::dynamicRecognizing(std::list<ColoredDynamicCluster>* input_clusters, std::shared_ptr<TsdfMap> tsdf_map_delta_distance){
  std::shared_ptr<TsdfMap> current_map = tsdf_ptr_queue_.back();
  std::shared_ptr<TsdfMap> old_map = tsdf_ptr_queue_.front();
  current_clusters_ = input_clusters;
  float voxels_per_side_inv = 1 / voxels_per_side_;

  BlockIndexList blocks_d_dist;
  tsdf_map_delta_distance->getTsdfLayerPtr()->getAllAllocatedBlocks(&blocks_d_dist);
  for (const BlockIndex& index : blocks_d_dist) {
    // Iterate over all voxels in current blocks.
    Block<TsdfVoxel>& block_current = tsdf_map_delta_distance->getTsdfLayerPtr()->getBlockByIndex(index);
    for (size_t linear_index = 0; linear_index < num_voxels_per_block_; ++linear_index) {
      TsdfVoxel& delta_distance_voxel = block_current.getVoxelByLinearIndex(linear_index);
      delta_distance_voxel.distance = 0;
    }
  }

  for (auto current_color_cluster = current_clusters_->begin(); current_color_cluster != current_clusters_->end(); current_color_cluster++){
    const LongIndexSet& current_cluster = current_color_cluster->cluster;
    float dynamic_counter = 0;
    float total_count = 0;
    for (GlobalIndex global_voxel_index : current_cluster){
      TsdfVoxel* old_voxel = old_map->getTsdfLayerPtr()->getVoxelPtrByGlobalIndex(global_voxel_index);
      TsdfVoxel* current_voxel = current_map->getTsdfLayerPtr()->getVoxelPtrByGlobalIndex(global_voxel_index);
      TsdfVoxel* delta_distance_voxel = tsdf_map_delta_distance->getTsdfLayerPtr()->getVoxelPtrByGlobalIndex(global_voxel_index);
      float delta_distance;
      if (old_voxel != nullptr) {
        if (old_voxel->weight != 0) {
          delta_distance = std::abs(current_voxel->distance - old_voxel->distance);
          delta_distance_voxel->distance = delta_distance;
          ROS_INFO("delta_distance = %f, threshold = %f", delta_distance, delta_distance_threshold_);
          //ROS_INFO("dynamic count = %u, total count = %u", dynamic_counter, total_count);
        } else {
          delta_distance = std::abs(current_voxel->distance);
          delta_distance_voxel->distance = delta_distance;
          ROS_INFO("weight 0; delta_distance = %f, threshold = %f", delta_distance, delta_distance_threshold_);
        }
      } else {
        delta_distance = std::abs(current_voxel->distance);
        delta_distance_voxel->distance = delta_distance;
        ROS_INFO("nullptr; delta_distance = %f, threshold = %f", delta_distance, delta_distance_threshold_);
      }
      total_count++;
      if (delta_distance > delta_distance_threshold_) {
        dynamic_counter++;
      }
    }
    if (total_count > 0){
      float dynamic_share = dynamic_counter/total_count;
      ROS_INFO("dynamic_share = %f, threshold = %f", dynamic_share, dynamic_share_threshold_);
      if (dynamic_share > dynamic_share_threshold_) current_color_cluster->dynamic = true;
    }
  }
}

void DynamicRecognizer::dynamicClusterVisualiser(pcl::PointCloud<pcl::PointXYZRGB>* dynamic_pointcloud,
                                                 pcl::PointCloud<pcl::PointXYZRGB>* static_pointcloud) {
  dynamic_pointcloud->clear();
  static_pointcloud->clear();
  for (const ColoredDynamicCluster& colored_cluster : *current_clusters_) {
    LongIndexSet cluster = colored_cluster.cluster;
    Color color = findColor();
    for (auto voxel_global_index = cluster.begin(); voxel_global_index != cluster.end(); ++voxel_global_index) {
      BlockIndex block_index ;
      VoxelIndex voxel_index ;
      //ROS_INFO("test_2");
      getBlockAndVoxelIndexFromGlobalVoxelIndex(*voxel_global_index, voxels_per_side_, &block_index, &voxel_index);
      const Block<TsdfVoxel>& block = tsdf_ptr_queue_.back()->getTsdfLayerPtr()->getBlockByIndex(block_index);
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

Color DynamicRecognizer::findColor() {
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

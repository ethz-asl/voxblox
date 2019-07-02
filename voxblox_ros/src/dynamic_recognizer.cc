#include <voxblox_ros/dynamic_recognizer.h>

namespace voxblox {

DynamicRecognizer::DynamicRecognizer(const std::shared_ptr<TsdfMap> input_map, float delta_distance_threshold, float dynamic_share_threshold){
  voxels_per_side_ = input_map->getTsdfLayerPtr()->voxels_per_side();
  num_voxels_per_block_ = voxels_per_side_ * voxels_per_side_ * voxels_per_side_;
  delta_distance_threshold_ = delta_distance_threshold;
  dynamic_share_threshold_ = dynamic_share_threshold;
}

void DynamicRecognizer::addCurrentMap(const std::shared_ptr<TsdfMap> input_map) {
  tsdf_ptr_queue_.push(input_map);
}

void DynamicRecognizer::dynamicRecognizing(std::list<ColoredDynamicCluster>* input_clusters){
  std::shared_ptr<TsdfMap> current_map = tsdf_ptr_queue_.back();
  std::shared_ptr<TsdfMap> old_map = tsdf_ptr_queue_.front();
  current_clusters_ = input_clusters;
  float voxels_per_side_inv = 1 / voxels_per_side_;

  for (auto current_color_cluster = current_clusters_->begin(); current_color_cluster != current_clusters_->end(); current_color_cluster++){
    if (!current_color_cluster->dynamic){
      LongIndexSet current_cluster = current_color_cluster->cluster;
      int dynamic_counter = 0;
      int total_count = 0;
      for (GlobalIndex global_voxel_index : current_cluster){
        TsdfVoxel* current_voxel = current_map->getTsdfLayerPtr()->getVoxelPtrByGlobalIndex(global_voxel_index);
        BlockIndex block_index = getBlockIndexFromGlobalVoxelIndex(global_voxel_index, voxels_per_side_inv);
        if (old_map->getTsdfLayerPtr()->hasBlock(block_index)) {
          TsdfVoxel* old_voxel = old_map->getTsdfLayerPtr()->getVoxelPtrByGlobalIndex(global_voxel_index);
          if (old_voxel != nullptr) {
            if (old_voxel->weight != 0) {
              float delta_distance = std::abs(current_voxel->distance - old_voxel->distance);
              //ROS_INFO("delta_distance = %f", delta_distance);
              total_count++;
              if (delta_distance > delta_distance_threshold_) dynamic_counter++;
            }
          }
        }
      }
      if (total_count > 0){
        float dynamic_share = dynamic_counter/total_count;
        //ROS_INFO("dynamic_share = %f", dynamic_share);
        if (dynamic_share > dynamic_share_threshold_) current_color_cluster->dynamic = true;      
      }
    }
  }
}

void DynamicRecognizer::dynamicClusterVisualiser(pcl::PointCloud<pcl::PointXYZRGB>* dynamic_pointcloud, 
                                                 pcl::PointCloud<pcl::PointXYZRGB>* static_pointcloud) {
  dynamic_pointcloud->clear();
  static_pointcloud->clear();
  for (const ColoredDynamicCluster colored_cluster : *current_clusters_) {
    LongIndexSet cluster = colored_cluster.cluster;
    Color color = colored_cluster.color;
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
} //end of namespace
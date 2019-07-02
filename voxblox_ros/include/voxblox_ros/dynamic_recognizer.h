#include <voxblox/core/block_hash.h>
#include <voxblox/core/tsdf_map.h>
#include <pcl_ros/point_cloud.h>

namespace voxblox {

class DynamicRecognizer{
  public:
    DynamicRecognizer(const std::shared_ptr<TsdfMap> input_map, float delta_distance_threshold, float dynamic_share_threshold);
    void addCurrentMap(const std::shared_ptr<TsdfMap> input_map);
    void dynamicRecognizing(std::list<ColoredDynamicCluster>* input_clusters);
    void dynamicClusterVisualiser(pcl::PointCloud<pcl::PointXYZRGB>* dynamic_pointcloud, pcl::PointCloud<pcl::PointXYZRGB>* static_pointcloud);
    int getMapQueueSize(){
      return tsdf_ptr_queue_.size();
    }
    void popfromQueue(){
      tsdf_ptr_queue_.pop();
    }
  private:
    size_t voxels_per_side_ ;
    size_t num_voxels_per_block_ ;
    float delta_distance_threshold_;
    float dynamic_share_threshold_;
    std::queue<std::shared_ptr<TsdfMap>> tsdf_ptr_queue_;
    std::list<ColoredDynamicCluster>* current_clusters_;
};
} //end of namespace
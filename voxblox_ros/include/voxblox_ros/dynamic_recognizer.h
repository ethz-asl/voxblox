#include <voxblox/core/block_hash.h>
#include <voxblox/core/tsdf_map.h>
#include <pcl_ros/point_cloud.h>

namespace voxblox {

class DynamicRecognizer{
  public:
    DynamicRecognizer(const std::shared_ptr<TsdfMap> input_map, float delta_distance_threshold, float dynamic_share_threshold);
    void addCurrentMap(const std::shared_ptr<TsdfMap> input_map);
    void dynamicRecognizing(std::list<ColoredDynamicCluster>* input_clusters, std::shared_ptr<TsdfMap> tsdf_map_delta_distance);
    void dynamicClusterVisualiser(pcl::PointCloud<pcl::PointXYZRGB>* dynamic_pointcloud, pcl::PointCloud<pcl::PointXYZRGB>* static_pointcloud);
    int getMapQueueSize(){
      return tsdf_ptr_queue_.size();
    }
    void popfromQueue(){
      tsdf_ptr_queue_.pop();
    }
    Color findColor();
    const std::vector<Color> colors = {Color::White(), Color::Red(), Color::Green(), Color::Blue(), 
                                       Color::Yellow(), Color::Orange(), Color::Purple(), Color::Teal(), Color::Pink()};
  private:
    size_t voxels_per_side_ ;
    size_t num_voxels_per_block_ ;
    float delta_distance_threshold_;
    float dynamic_share_threshold_;
    std::queue<std::shared_ptr<TsdfMap>> tsdf_ptr_queue_;
    std::list<ColoredDynamicCluster>* current_clusters_;
    int used_colors_ [8]; //colors.size - 1
};
} //end of namespace
#include <voxblox/core/block_hash.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/utils/neighbor_tools.h>
#include <pcl_ros/point_cloud.h>

namespace voxblox {

class Clustering {
  public:
    Clustering(const std::shared_ptr<TsdfMap> input_map, float cluster_distance_threshold, 
               unsigned int cluster_match_vote_threshold, unsigned int cluster_min_size_threshold,
               unsigned int clustering_queue_size);
    void addCurrentMap(const std::shared_ptr<TsdfMap> input_map);
    void matchCommunClusters();
    pcl::PointCloud<pcl::PointXYZRGB> extractedClusterVisualiser();
    pcl::PointCloud<pcl::PointXYZRGB> matchedClusterVisualiser();
    std::list<ColoredDynamicCluster>* getCurrentClustersPointer() {
      return &(cluster_queue_.back());
    }

    int getClusterQueueSize(){
      return cluster_queue_.size();
    }

    void popfromQueue(){
      cluster_queue_.pop();
    }

    Color colors[9] = {Color::White(), Color::Red(), Color::Green(), Color::Blue(), 
                  Color::Yellow(), Color::Orange(), Color::Purple(), Color::Teal(), Color::Pink()};

  private:
    int cluster_distance_threshold_ ;
    unsigned int cluster_match_vote_threshold_ ;
    unsigned int cluster_min_size_threshold_;
    unsigned int clustering_queue_size_;
    size_t voxels_per_side_ ;
    size_t num_voxels_per_block_ ;

    std::shared_ptr<TsdfMap> current_map_ ;
    std::queue<std::list<ColoredDynamicCluster>> cluster_queue_ ;
};
} // end of namespace
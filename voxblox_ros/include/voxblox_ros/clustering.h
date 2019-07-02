#include <voxblox/core/block_hash.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/utils/neighbor_tools.h>
#include <pcl_ros/point_cloud.h>

namespace voxblox {

class Clustering {
  public:
    Clustering(const std::shared_ptr<TsdfMap> input_map, float cluster_distance_threshold, 
               unsigned int cluster_match_vote_threshold, unsigned int cluster_min_size_threshold);
    void addCurrentMap(const std::shared_ptr<TsdfMap> input_map);
    void matchCommunClusters();
    pcl::PointCloud<pcl::PointXYZRGB> extractedClusterVisualiser();
    pcl::PointCloud<pcl::PointXYZRGB> matchedClusterVisualiser();
    std::list<ColoredDynamicCluster>* getCurrentClustersPointer() {
      return &(current_clusters_);
    }
    bool isOldClustersEmpty(){
      return (old_clusters_.size() == 0);
    }

    Color colors[9] = {Color::White(), Color::Red(), Color::Green(), Color::Blue(), 
                  Color::Yellow(), Color::Orange(), Color::Purple(), Color::Teal(), Color::Pink()};

  private:
    int cluster_distance_threshold_ ;
    unsigned int cluster_match_vote_threshold_ ;
    unsigned int cluster_min_size_threshold_;
    size_t voxels_per_side_ ;
    size_t num_voxels_per_block_ ;

    std::shared_ptr<TsdfMap> current_map_ ;
    std::list<ColoredDynamicCluster> current_clusters_;
    std::list<ColoredDynamicCluster> old_clusters_;
    //std::queue<std::list<ColoredDynamicCluster>> cluster_queue_ ;
};
} // end of namespace
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
    void determineFOV();
    pcl::PointCloud<pcl::PointXYZRGB> extractedClusterVisualiser();
    pcl::PointCloud<pcl::PointXYZRGB> matchedClusterVisualiser();
    void mapClusterVisualiser(pcl::PointCloud<pcl::PointXYZRGB>* dynamic_pointcloud,  
                              pcl::PointCloud<pcl::PointXYZRGB>* static_pointcloud,
                              std::shared_ptr<TsdfMap> tsdf_map);

    std::list<ColoredDynamicCluster>* getCurrentClustersPointer() {
      return &(current_clusters_);
    }
    bool isMapClustersEmpty(){
      return (map_clusters_.size() == 0);
    }
    int findIndex();
    Color findColor();
    
    const std::vector<Color> colors = {Color::White(), Color::Red(), Color::Green(), Color::Blue(), 
                  Color::Yellow(), Color::Orange(), Color::Purple(), Color::Teal(), Color::Pink()};

  private:
    int cluster_distance_threshold_ ;
    unsigned int cluster_match_vote_threshold_ ;
    unsigned int cluster_min_size_threshold_;
    size_t voxels_per_side_ ;
    size_t num_voxels_per_block_ ;

    std::shared_ptr<TsdfMap> current_oneshot_map_ ;
    std::list<ColoredDynamicCluster> current_clusters_;
    std::list<ColoredDynamicCluster> map_clusters_;
    std::list<int> used_indices_;
    int used_colors_ [8]; //colors.size - 1
    LongIndexSet current_FOV_voxels_;
    //std::queue<std::list<ColoredDynamicCluster>> cluster_queue_ ;
};
} // end of namespace
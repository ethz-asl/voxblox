#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include "voxblox_ros/conversions.h"
#include "voxblox_ros/ros_params.h"

#include "voxblox_ros/tsdf_server.h"

namespace voxblox {

TsdfServer::TsdfServer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private)
    : TsdfServer(nh, nh_private, getTsdfMapConfigFromRosParam(nh_private),
                 getTsdfIntegratorConfigFromRosParam(nh_private),
                 getMeshIntegratorConfigFromRosParam(nh_private)) {}

TsdfServer::TsdfServer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private,
                       const TsdfMap::Config& config,
                       const TsdfIntegratorBase::Config& integrator_config,
                       const MeshIntegratorConfig& mesh_config)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(true),
      world_frame_("world"),
      icp_corrected_frame_("icp_corrected"),
      pose_corrected_frame_("pose_corrected"),
      max_block_distance_from_body_(std::numeric_limits<FloatingPoint>::max()),
      slice_level_(0.5),
      use_freespace_pointcloud_(false),
      color_map_(new RainbowColorMap()),
      publish_pointclouds_on_update_(false),
      publish_slices_(false),
      publish_pointclouds_(false),
      publish_tsdf_map_(false),
      cache_mesh_(false),
      enable_icp_(false),
      accumulate_icp_corrections_(true),
      pointcloud_queue_size_(1),
      num_subscribers_tsdf_map_(0),
      transformer_(nh, nh_private),
      cluster_distance_threshold_(1),
      cluster_match_vote_threshold_(5),
      cluster_min_size_threshold_(5),
      clustering_queue_size_(5)
       {
  getServerConfigFromRosParam(nh_private);

  // Advertise topics.
  surface_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
          "surface_pointcloud", 1, true);
  tsdf_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >("tsdf_pointcloud",
                                                              1, true);
  tsdf_newly_occupied_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >("tsdf_newly_occupied_slice",
                                                              1, true);
  tsdf_newly_occupied_distance_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >("tsdf_newly_occupied_distance_slice",
                                                              1, true);
  clustered_pointcloud_pub_ = 
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("clustered_pointcloud", 1, true);

  occupancy_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("occupied_nodes",
                                                             1, true);
  tsdf_slice_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "tsdf_slice", 1, true);

  nh_private_.param("pointcloud_queue_size", pointcloud_queue_size_,
                    pointcloud_queue_size_);
  pointcloud_sub_ = nh_.subscribe("pointcloud", pointcloud_queue_size_,
                                  &TsdfServer::insertPointcloud, this);

  mesh_pub_ = nh_private_.advertise<voxblox_msgs::Mesh>("mesh", 1, true);

  // Publishing/subscribing to a layer from another node (when using this as
  // a library, for example within a planner).
  tsdf_map_pub_ =
      nh_private_.advertise<voxblox_msgs::Layer>("tsdf_map_out", 1, false);
  tsdf_map_sub_ = nh_private_.subscribe("tsdf_map_in", 1,
                                        &TsdfServer::tsdfMapCallback, this);
  nh_private_.param("publish_tsdf_map", publish_tsdf_map_, publish_tsdf_map_);

  if (use_freespace_pointcloud_) {
    // points that are not inside an object, but may also not be on a surface.
    // These will only be used to mark freespace beyond the truncation distance.
    freespace_pointcloud_sub_ =
        nh_.subscribe("freespace_pointcloud", pointcloud_queue_size_,
                      &TsdfServer::insertFreespacePointcloud, this);
  }

  if (enable_icp_) {
    icp_transform_pub_ = nh_private_.advertise<geometry_msgs::TransformStamped>(
        "icp_transform", 1, true);
    nh_private_.param("icp_corrected_frame", icp_corrected_frame_,
                      icp_corrected_frame_);
    nh_private_.param("pose_corrected_frame", pose_corrected_frame_,
                      pose_corrected_frame_);
  }

  // Initialize TSDF Map and integrator.
  tsdf_map_.reset(new TsdfMap(config));

  std::string method("merged");
  nh_private_.param("method", method, method);
  if (method.compare("simple") == 0) {
    tsdf_integrator_.reset(new SimpleTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else if (method.compare("merged") == 0) {
    tsdf_integrator_.reset(new MergedTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else if (method.compare("fast") == 0) {
    tsdf_integrator_.reset(new FastTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else {
    tsdf_integrator_.reset(new SimpleTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  }

  //Vinz
  clustering_.reset(new Clustering(tsdf_map_, cluster_distance_threshold_, cluster_match_vote_threshold_, cluster_min_size_threshold_, clustering_queue_size_));

  mesh_layer_.reset(new MeshLayer(tsdf_map_->block_size()));

  mesh_integrator_.reset(new MeshIntegrator<TsdfVoxel>(
     mesh_config, tsdf_map_->getTsdfLayerPtr(), mesh_layer_.get()));

  icp_.reset(new ICP(getICPConfigFromRosParam(nh_private)));

  // Advertise services.
  generate_mesh_srv_ = nh_private_.advertiseService(
      "generate_mesh", &TsdfServer::generateMeshCallback, this);
  clear_map_srv_ = nh_private_.advertiseService(
      "clear_map", &TsdfServer::clearMapCallback, this);
  save_map_srv_ = nh_private_.advertiseService(
      "save_map", &TsdfServer::saveMapCallback, this);
  load_map_srv_ = nh_private_.advertiseService(
      "load_map", &TsdfServer::loadMapCallback, this);
  publish_pointclouds_srv_ = nh_private_.advertiseService(
      "publish_pointclouds", &TsdfServer::publishPointcloudsCallback, this);
  publish_tsdf_map_srv_ = nh_private_.advertiseService(
      "publish_map", &TsdfServer::publishTsdfMapCallback, this);

  // If set, use a timer to progressively integrate the mesh.
  double update_mesh_every_n_sec = 1.0;
  nh_private_.param("update_mesh_every_n_sec", update_mesh_every_n_sec,
                    update_mesh_every_n_sec);

  if (update_mesh_every_n_sec > 0.0) {
    update_mesh_timer_ =
        nh_private_.createTimer(ros::Duration(update_mesh_every_n_sec),
                                &TsdfServer::updateMeshEvent, this);
  }

  double publish_map_every_n_sec = 1.0;
  nh_private_.param("publish_map_every_n_sec", publish_map_every_n_sec,
                    publish_map_every_n_sec);

  if (publish_map_every_n_sec > 0.0) {
    publish_map_timer_ =
        nh_private_.createTimer(ros::Duration(publish_map_every_n_sec),
                                &TsdfServer::publishMapEvent, this);
  }
}

void TsdfServer::getServerConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  // Before subscribing, determine minimum time between messages.
  // 0 by default.
  double min_time_between_msgs_sec = 0.0;
  nh_private.param("min_time_between_msgs_sec", min_time_between_msgs_sec,
                   min_time_between_msgs_sec);
  min_time_between_msgs_.fromSec(min_time_between_msgs_sec);

  nh_private.param("max_block_distance_from_body",
                   max_block_distance_from_body_,
                   max_block_distance_from_body_);
  nh_private.param("slice_level", slice_level_, slice_level_);
  nh_private.param("world_frame", world_frame_, world_frame_);
  nh_private.param("publish_pointclouds_on_update",
                   publish_pointclouds_on_update_,
                   publish_pointclouds_on_update_);
  nh_private.param("publish_slices", publish_slices_, publish_slices_);
  nh_private.param("publish_pointclouds", publish_pointclouds_,
                   publish_pointclouds_);

  nh_private.param("use_freespace_pointcloud", use_freespace_pointcloud_,
                   use_freespace_pointcloud_);
  nh_private.param("pointcloud_queue_size", pointcloud_queue_size_,
                   pointcloud_queue_size_);
  nh_private.param("enable_icp", enable_icp_, enable_icp_);
  nh_private.param("accumulate_icp_corrections", accumulate_icp_corrections_,
                   accumulate_icp_corrections_);
  nh_private.param("cluster_distance_threshold",
                   cluster_distance_threshold_,
                   cluster_distance_threshold_);
  nh_private.param("cluster_match_vote_threshold",
                   cluster_match_vote_threshold_,
                   cluster_match_vote_threshold_);
  nh_private.param("cluster_min_size_threshold",
                   cluster_min_size_threshold_,
                   cluster_min_size_threshold_);
  nh_private.param("clustering_queue_size",
                   clustering_queue_size_, 
                   clustering_queue_size_);

  nh_private.param("verbose", verbose_, verbose_);

  // Mesh settings.
  nh_private.param("mesh_filename", mesh_filename_, mesh_filename_);
  std::string color_mode("");
  nh_private.param("color_mode", color_mode, color_mode);
  color_mode_ = getColorModeFromString(color_mode);

  // Color map for intensity pointclouds.
  std::string intensity_colormap("rainbow");
  float intensity_max_value = kDefaultMaxIntensity;
  nh_private.param("intensity_colormap", intensity_colormap,
                   intensity_colormap);
  nh_private.param("intensity_max_value", intensity_max_value,
                   intensity_max_value);

  // Default set in constructor.
  if (intensity_colormap == "rainbow") {
    color_map_.reset(new RainbowColorMap());
  } else if (intensity_colormap == "inverse_rainbow") {
    color_map_.reset(new InverseRainbowColorMap());
  } else if (intensity_colormap == "grayscale") {
    color_map_.reset(new GrayscaleColorMap());
  } else if (intensity_colormap == "inverse_grayscale") {
    color_map_.reset(new InverseGrayscaleColorMap());
  } else if (intensity_colormap == "ironbow") {
    color_map_.reset(new IronbowColorMap());
  } else {
    ROS_ERROR_STREAM("Invalid color map: " << intensity_colormap);
  }
  color_map_->setMaxValue(intensity_max_value);
}

Queue::Queue() {
  last = nullptr;
  queue_size = 0;
}

void Queue::push(TsdfMap::Ptr tsdf_map){
  Member* new_member = new Member;
  new_member->tsdf_ptr = tsdf_map;
  new_member->next = last;
  last = new_member;
  queue_size++;
}

TsdfMap::Ptr Queue::front() {
  Member* current = last;
  while(current->next != nullptr) {
    current = current->next;
  }
  return current->tsdf_ptr;
}

void Queue::pop() {
  Member* current = last;
  while(current->next->next != nullptr) {
    current = current->next;
  }
  delete current->next;
  current->next = nullptr;
  queue_size--;
}

int Queue::size() {
  return queue_size;
}

Clustering::Clustering(const std::shared_ptr<TsdfMap> input_map, float cluster_distance_threshold, 
                       unsigned int cluster_match_vote_threshold, unsigned int cluster_min_size_threshold,
                       unsigned int clustering_queue_size) {
  current_map_.reset(new TsdfMap(input_map->getTsdfLayer()));
  //layer_.reset(current_map_->getTsdfLayerPtr());
  cluster_distance_threshold_ = cluster_distance_threshold * current_map_->getTsdfLayerPtr()->voxel_size() / 2;
  cluster_match_vote_threshold_ = cluster_match_vote_threshold; //number of voxels which need to have voted for a cluster for a match
  cluster_min_size_threshold_ = cluster_min_size_threshold;
  clustering_queue_size_ = clustering_queue_size;
  voxels_per_side_ = current_map_->getTsdfLayerPtr()->voxels_per_side();
  num_voxels_per_block_ = voxels_per_side_ * voxels_per_side_ * voxels_per_side_;
}

void Clustering::addCurrentMap(const std::shared_ptr<TsdfMap> input_map) {
  current_map_.reset(new TsdfMap(input_map->getTsdfLayer()));
  std::list<ColoredCluster> cluster_list;
  cluster_list.clear();
  Neighborhood<>::IndexMatrix neighbors;

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
          Neighborhood<>::getFromGlobalIndex(cluster_voxel_gi, &neighbors);
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
          ColoredCluster current_colored_cluster;
          if (cluster_queue_.size() < (clustering_queue_size_ - 1)) {
            Color color = colors[cluster_list.size() + 1];
            current_colored_cluster = {current_cluster , color};
          }
          else current_colored_cluster = {current_cluster , Color::White()};
          
          /*if (current_colored_cluster.color == Color::Red()){
            ROS_INFO("red");
          } else if (current_colored_cluster.color == Color::Green()){
            ROS_INFO("green");
          } else if (current_colored_cluster.color == Color::Blue()){
            ROS_INFO("blue");
          } else if (current_colored_cluster.color == Color::White()){
            ROS_INFO("white");
          }*/
          cluster_list.push_back(current_colored_cluster);
        }
      }
    }
  }
  cluster_queue_.push(cluster_list);
  /*std::list<ColoredCluster> cluster_list_test = cluster_queue_.back();
  ColoredCluster cluster = cluster_list_test.front();
  if (cluster.color == Color::Red()){
    ROS_INFO("red");
  } else if (cluster.color == Color::Green()){
    ROS_INFO("green");
  } else if (cluster.color == Color::Blue()){
    ROS_INFO("blue");
  } else if (cluster.color == Color::White()){
    ROS_INFO("white");
  }*/
}

/*std::shared_ptr<std::list<LongIndexSet>> Clustering::extractClusters() {
  cluster_list_.reset(new std::list<LongIndexSet>);
  cluster_list_->clear();
  //current_cluster_.reset(new std::list<GlobalIndex>);
  Neighborhood<>::IndexMatrix neighbors;
  const size_t num_voxels_per_block = voxels_per_side_ * voxels_per_side_ * voxels_per_side_;
  BlockIndexList blocks;
  current_map_->getTsdfLayerPtr()->getAllAllocatedBlocks(&blocks);

  for (const BlockIndex& index : blocks) {
    const Block<TsdfVoxel>& block = current_map_->getTsdfLayerPtr()->getBlockByIndex(index);
    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
         ++linear_index) {
      TsdfVoxel voxel = block.getVoxelByLinearIndex(linear_index);
      if (voxel.weight != 0 && voxel.distance < distance_threshold_) {
        LongIndexSet current_cluster_;
        current_cluster_.clear();
        VoxelIndex voxel_index = block.computeVoxelIndexFromLinearIndex(linear_index);
        GlobalIndex global_voxel_index = getGlobalVoxelIndexFromBlockAndVoxelIndex(index, voxel_index, voxels_per_side_);
        current_cluster_.insert(global_voxel_index);
        voxel.weight = 0;
        std::queue<GlobalIndex> global_index_voxel_queue;
        global_index_voxel_queue.push(global_voxel_index);
        while(!global_index_voxel_queue.empty()) {
          GlobalIndex cluster_voxel_gi = global_index_voxel_queue.front();
          Neighborhood<>::getFromGlobalIndex(cluster_voxel_gi, &neighbors);
          for (unsigned int i = 0; i < Connectivity::kTwentySix; i++) {
            GlobalIndex neighbor_global_index = neighbors.col(i);
            TsdfVoxel* neighbor_voxel_ptr = current_map_->getTsdfLayerPtr()->getVoxelPtrByGlobalIndex(neighbor_global_index);
            if (neighbor_voxel_ptr == nullptr) continue;
            if (neighbor_voxel_ptr->weight != 0 && neighbor_voxel_ptr->distance < distance_threshold_) {
              current_cluster_.insert(neighbor_global_index);
              global_index_voxel_queue.push(neighbor_global_index);
              neighbor_voxel_ptr->weight = 0;
            }
          }
          global_index_voxel_queue.pop();
        }
        if (current_cluster_.size() > cluster_match_vote_threshold_) {
          cluster_list_->push_back(current_cluster_);
        }
      }
    }
  }
  return cluster_list_;
}*/

void Clustering::matchCommunClusters() {
 std::list<ColoredCluster>* current_clusters = &(cluster_queue_.back());
 std::list<ColoredCluster>* old_clusters = &(cluster_queue_.front());

	const int size = current_clusters->size();

  bool* used_colors = new bool[9];
  for (int i = 0; i<9; i++) used_colors[i] = false;
	
	for (ColoredCluster old_color_cluster : *old_clusters) {
    unsigned int* current_cluster_vote_array = new unsigned int[size];
	  for (int i = 0; i < size; i++) current_cluster_vote_array[i] = 0;
    LongIndexSet old_cluster = old_color_cluster.cluster;
    //Vote of old voxels for a current_cluster
		for (auto old_voxel_global_index = old_cluster.begin(); old_voxel_global_index != old_cluster.end(); old_voxel_global_index++) {
			//Point voxel_coord = voxel_element.coord;
			int cluster_count = 0;
			for (const ColoredCluster current_color_cluster : *current_clusters) {
        LongIndexSet current_cluster = current_color_cluster.cluster;
        if (current_cluster.find(*old_voxel_global_index) != current_cluster.end()){
          current_cluster_vote_array[cluster_count] += 1;
          break;
        }
				cluster_count += 1;
			}
		}
    //matching current_cluster to old_cluster
		std::list<ColoredCluster*> matched_current_clusters;
		matched_current_clusters.clear();
    int max_votes = 0;
    ColoredCluster* base_current_color_cluster;
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
    std::queue<ColoredCluster*> double_matched_current_clusters;
    for (auto matched_current_cluster = matched_current_clusters.begin(); matched_current_cluster != matched_current_clusters.end(); matched_current_cluster++ ) {
      /*if ((*matched_current_cluster)->color == Color::Red()){
        ROS_INFO("red");
      } else if ((*matched_current_cluster)->color == Color::Green()){
        ROS_INFO("green");
      } else if ((*matched_current_cluster)->color == Color::Blue()){
        ROS_INFO("blue");
      } else if ((*matched_current_cluster)->color == Color::White()){
        ROS_INFO("white");
      }*/
      GlobalIndex matched_current_cluster_voxel = *((*matched_current_cluster)->cluster).begin();
      for (auto current_color_cluster = current_clusters->begin(); current_color_cluster != current_clusters->end(); current_color_cluster++) {
        if (current_color_cluster->color != colors[0] && current_color_cluster->cluster.find(matched_current_cluster_voxel) != current_color_cluster->cluster.end()) {
          double_matched_current_clusters.push(&(*current_color_cluster));
          ROS_INFO("double match activated");
        }
      }
    }

    // In case the current cluster is devided in several clusters
    if (matched_current_clusters.size() > 1) {
      //matched_current_clusters.erase(matched_current_clusters.begin());
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
    while (!double_matched_current_clusters.empty()) {
      ColoredCluster* previously_computed_double_matched_current_cluster = double_matched_current_clusters.front();
      for (GlobalIndex previously_computed_current_voxel : previously_computed_double_matched_current_cluster->cluster) {
        base_current_cluster->insert(previously_computed_current_voxel);
      }
      color_from_old = previously_computed_double_matched_current_cluster->color;
      /*for (auto current_cluster = current_clusters->begin(); current_cluster != current_clusters->end(); current_cluster++){
        ROS_INFO("test 13");
        if (current_cluster->cluster.find(*(previously_computed_double_matched_current_cluster->cluster.begin())) != current_cluster->cluster.end()) {
            ROS_INFO("test 11");
            current_clusters->erase(current_cluster);
            ROS_INFO("test 10");
            break;
        }
      }*/
      current_clusters->remove(*previously_computed_double_matched_current_cluster);
      double_matched_current_clusters.pop();
    }
    if (color_from_old != colors[0]) {
      base_current_color_cluster->color = color_from_old;
    } else {
      base_current_color_cluster->color = old_color_cluster.color;
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

/* pcl::PointCloud<pcl::PointXYZRGB> Clustering::extractedClusterVisualiser() {
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  pointcloud.clear();
  Color color;
  Color color_array[] = {color.Red(), color.Green(), color.Blue(), color.Yellow(), color.Orange(), color.Purple(), color.Teal(), color.Pink()};
  int color_counter = 0;
  for (LongIndexSet cluster : *cluster_ _) {
    color = color_array[color_counter % 8];
    if (color_counter > 8) ROS_WARN ("overflowing colors for cluster pub");

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
    color_counter++;
  }
  return pointcloud;
} */

pcl::PointCloud<pcl::PointXYZRGB> Clustering::matchedClusterVisualiser() {
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  pointcloud.clear();
  std::list<ColoredCluster>* current_clusters = &(cluster_queue_.back());
  for (ColoredCluster colored_cluster : *current_clusters) {
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

/*pcl::PointCloud<pcl::PointXYZRGB> matchedClusterVisualiser(std::list<std::pair<LongIndexSet, LongIndexSet>> matched_clusters_list) {
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  pointcloud.clear();
  Color color;
  Color color_array[] = {color.Red(), color.Green(), color.Blue(), color.Yellow(), color.Orange(), color.Purple(), color.Teal(), color.Pink()};
  int color_counter = 0;
  for (LongIndexSet cluster : *cluster_list_) {
    color = color_array[color_counter % 8];
    if (color_counter > 8) ROS_WARN ("overflowing colors for cluster pub");

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
    color_counter++;
  }
  return pointcloud;
}*/

void TsdfServer::createNewlyOccupiedMap(const TsdfMap::Ptr current_map, 
      TsdfMap::Ptr old_map, TsdfMap::Ptr newly_occupied_map, TsdfMap::Ptr newly_occupied_map_distance){
  
  const size_t vps = current_map->getTsdfLayerPtr()->voxels_per_side();
  const size_t num_voxels_per_block = vps * vps * vps;
  const float distance_threshold = 0.1;

  BlockIndexList blocks_current;
  current_map->getTsdfLayerPtr()->getAllAllocatedBlocks(&blocks_current);

  for (const BlockIndex& index : blocks_current) {
    // Iterate over all voxels in current blocks.
    const Block<TsdfVoxel>& block_current = current_map->getTsdfLayerPtr()->getBlockByIndex(index);

    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
         ++linear_index) {
      //ROS_INFO("Going through voxel indexes");
      const Point coord = block_current.computeCoordinatesFromLinearIndex(linear_index);
      const TsdfVoxel& voxel_current = block_current.getVoxelByLinearIndex(linear_index);
      TsdfVoxel* voxel_newly_occupied = newly_occupied_map->getTsdfLayerPtr()->getVoxelPtrByCoordinates(coord);
      TsdfVoxel* voxel_newly_occupied_distance = newly_occupied_map_distance->getTsdfLayerPtr()->getVoxelPtrByCoordinates(coord);
      const TsdfVoxel* voxel_old = old_map->getTsdfLayerPtr()->getVoxelPtrByCoordinates(coord);
      if (voxel_old == nullptr) {
        //ROS_INFO("NOM: voxel_old = nullptr");
        voxel_newly_occupied->weight = 0;
        voxel_newly_occupied_distance->weight = 0;
        continue;
      }

      voxel_newly_occupied_distance->distance = voxel_current.distance - voxel_old->distance;

      if (std::abs(voxel_current.distance) < distance_threshold && voxel_current.weight != 0) {
        if (std::abs(voxel_old->distance) < distance_threshold || voxel_old->weight == 0) {
          //delete voxel in newly_occupied_map
          voxel_newly_occupied->weight = 0;
        }
      } 
      else {
          voxel_newly_occupied->weight = 0;
      }
    }
  }  
}

void TsdfServer::processPointCloudMessageAndInsert(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg,
    const Transformation& T_G_C, const bool is_freespace_pointcloud) {
  // Convert the PCL pointcloud into our awesome format.

  //Allows only for SimpleTsdfIntegrator and does not allow to output a mesh
  TsdfMap::Config config;
  config.tsdf_voxel_size = tsdf_map_->getTsdfLayerPtr()->voxel_size();
  config.tsdf_voxels_per_side = tsdf_map_->getTsdfLayerPtr()->voxels_per_side();
  tsdf_map_.reset(new TsdfMap(config));

  const TsdfIntegratorBase::Config integrator_config = tsdf_integrator_->getConfig();
  tsdf_integrator_.reset(new SimpleTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));

  // Horrible hack fix to fix color parsing colors in PCL.
  bool color_pointcloud = false;
  bool has_intensity = false;
  for (size_t d = 0; d < pointcloud_msg->fields.size(); ++d) {
    if (pointcloud_msg->fields[d].name == std::string("rgb")) {
      pointcloud_msg->fields[d].datatype = sensor_msgs::PointField::FLOAT32;
      color_pointcloud = true;
    } else if (pointcloud_msg->fields[d].name == std::string("intensity")) {
      has_intensity = true;
    }
  }

  Pointcloud points_C;
  Colors colors;
  timing::Timer ptcloud_timer("ptcloud_preprocess");

  // Convert differently depending on RGB or I type.
  if (color_pointcloud) {
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  } else if (has_intensity) {
    pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  } else {
    pcl::PointCloud<pcl::PointXYZ> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  }
  ptcloud_timer.Stop();

  Transformation T_G_C_refined = T_G_C;
  if (enable_icp_) {
    timing::Timer icp_timer("icp");
    if (!accumulate_icp_corrections_) {
      icp_corrected_transform_.setIdentity();
    }
    static Transformation T_offset;
    const size_t num_icp_updates =
        icp_->runICP(tsdf_map_->getTsdfLayer(), points_C,
                     icp_corrected_transform_ * T_G_C, &T_G_C_refined);
    if (verbose_) {
      ROS_INFO("ICP refinement performed %zu successful update steps",
               num_icp_updates);
    }
    icp_corrected_transform_ = T_G_C_refined * T_G_C.inverse();

    if (!icp_->refiningRollPitch()) {
      // its already removed internally but small floating point errors can
      // build up if accumulating transforms
      Transformation::Vector6 T_vec = icp_corrected_transform_.log();
      T_vec[3] = 0.0;
      T_vec[4] = 0.0;
      icp_corrected_transform_ = Transformation::exp(T_vec);
    }

    // Publish transforms as both TF and message.
    tf::Transform icp_tf_msg, pose_tf_msg;
    geometry_msgs::TransformStamped transform_msg;

    tf::transformKindrToTF(icp_corrected_transform_.cast<double>(),
                           &icp_tf_msg);
    tf::transformKindrToTF(T_G_C.cast<double>(), &pose_tf_msg);
    tf::transformKindrToMsg(icp_corrected_transform_.cast<double>(),
                            &transform_msg.transform);
    tf_broadcaster_.sendTransform(
        tf::StampedTransform(icp_tf_msg, pointcloud_msg->header.stamp,
                             world_frame_, icp_corrected_frame_));
    tf_broadcaster_.sendTransform(
        tf::StampedTransform(pose_tf_msg, pointcloud_msg->header.stamp,
                             icp_corrected_frame_, pose_corrected_frame_));

    transform_msg.header.frame_id = world_frame_;
    transform_msg.child_frame_id = icp_corrected_frame_;
    icp_transform_pub_.publish(transform_msg);

    icp_timer.Stop();
  }

  if (verbose_) {
    ROS_INFO("Integrating a pointcloud with %lu points.", points_C.size());
  }

  ros::WallTime start = ros::WallTime::now();
  integratePointcloud(T_G_C_refined, points_C, colors, is_freespace_pointcloud);
  ros::WallTime end = ros::WallTime::now();
  if (verbose_) {
    ROS_INFO("Finished integrating in %f seconds, have %lu blocks.",
             (end - start).toSec(),
             tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks());
  }

  timing::Timer block_remove_timer("remove_distant_blocks");
  tsdf_map_->getTsdfLayerPtr()->removeDistantBlocks(
      T_G_C.getPosition(), max_block_distance_from_body_);
  mesh_layer_->clearDistantMesh(T_G_C.getPosition(),
                                max_block_distance_from_body_);
  block_remove_timer.Stop();

  //Vinz: starting my part now 
  //TsdfMap::Ptr new_map (new TsdfMap(tsdf_map_->getTsdfLayer()));
  //new_map.reset(new ); 

  //queue_.push(new_map);
  clustering_->addCurrentMap(tsdf_map_);
  cluster_matching_active_ = false;
  ROS_INFO("cluster queue size %u", clustering_->getClusterQueueSize());
  while (clustering_->getClusterQueueSize() > clustering_queue_size_) {
    clustering_->popfromQueue();
  }

  if (clustering_->getClusterQueueSize() == clustering_queue_size_) {
    cluster_matching_active_ = true;
    ROS_INFO("newly occupied active true");
    clustering_->matchCommunClusters();
    clustered_pcl_ = clustering_->matchedClusterVisualiser();
   // tsdf_map_newly_occupied_.reset(new TsdfMap(tsdf_map_->getTsdfLayer()));
    //tsdf_map_newly_occupied_distance_.reset(new TsdfMap(tsdf_map_->getTsdfLayer()));
    //createNewlyOccupiedMap(tsdf_map_, queue_.front(), tsdf_map_newly_occupied_, tsdf_map_newly_occupied_distance_);
    //queue_.pop();
    clustering_->popfromQueue();
  }
  ROS_INFO("-----------------------");

  // Callback for inheriting classes.
  newPoseCallback(T_G_C);
}

// Checks if we can get the next message from queue.
bool TsdfServer::getNextPointcloudFromQueue(
    std::queue<sensor_msgs::PointCloud2::Ptr>* queue,
    sensor_msgs::PointCloud2::Ptr* pointcloud_msg, Transformation* T_G_C) {
  const size_t kMaxQueueSize = 10;
  if (queue->empty()) {
    return false;
  }
  *pointcloud_msg = queue->front();
  if (transformer_.lookupTransform((*pointcloud_msg)->header.frame_id,
                                   world_frame_,
                                   (*pointcloud_msg)->header.stamp, T_G_C)) {
    queue->pop();
    return true;
  } else {
    if (queue->size() >= kMaxQueueSize) {
      ROS_ERROR_THROTTLE(60,
                         "Input pointcloud queue getting too long! Dropping "
                         "some pointclouds. Either unable to look up transform "
                         "timestamps or the processing is taking too long.");
      while (queue->size() >= kMaxQueueSize) {
        queue->pop();
      }
    }
  }
  return false;
}

void TsdfServer::insertPointcloud(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in) {
  if (pointcloud_msg_in->header.stamp - last_msg_time_ptcloud_ >
      min_time_between_msgs_) {
    last_msg_time_ptcloud_ = pointcloud_msg_in->header.stamp;
    // So we have to process the queue anyway... Push this back.
    pointcloud_queue_.push(pointcloud_msg_in);
  }

  Transformation T_G_C;
  sensor_msgs::PointCloud2::Ptr pointcloud_msg;
  bool processed_any = false;
  while (
      getNextPointcloudFromQueue(&pointcloud_queue_, &pointcloud_msg, &T_G_C)) {
    constexpr bool is_freespace_pointcloud = false;
    processPointCloudMessageAndInsert(pointcloud_msg, T_G_C,
                                      is_freespace_pointcloud);
    processed_any = true;
  }

  if (!processed_any) {
    return;
  }

  if (publish_pointclouds_on_update_) {
    publishPointclouds();
  }

  if (verbose_) {
    ROS_INFO_STREAM("Timings: " << std::endl << timing::Timing::Print());
    ROS_INFO_STREAM(
        "Layer memory: " << tsdf_map_->getTsdfLayer().getMemorySize());
  }
}

void TsdfServer::insertFreespacePointcloud(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in) {
  if (pointcloud_msg_in->header.stamp - last_msg_time_freespace_ptcloud_ >
      min_time_between_msgs_) {
    last_msg_time_freespace_ptcloud_ = pointcloud_msg_in->header.stamp;
    // So we have to process the queue anyway... Push this back.
    freespace_pointcloud_queue_.push(pointcloud_msg_in);
  }

  Transformation T_G_C;
  sensor_msgs::PointCloud2::Ptr pointcloud_msg;
  while (getNextPointcloudFromQueue(&freespace_pointcloud_queue_,
                                    &pointcloud_msg, &T_G_C)) {
    constexpr bool is_freespace_pointcloud = true;
    processPointCloudMessageAndInsert(pointcloud_msg, T_G_C,
                                      is_freespace_pointcloud);
  }
}

void TsdfServer::integratePointcloud(const Transformation& T_G_C,
                                     const Pointcloud& ptcloud_C,
                                     const Colors& colors,
                                     const bool is_freespace_pointcloud) {
  CHECK_EQ(ptcloud_C.size(), colors.size());
  tsdf_integrator_->integratePointCloud(T_G_C, ptcloud_C, colors,
                                        is_freespace_pointcloud);
}

void TsdfServer::publishAllUpdatedTsdfVoxels() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  createDistancePointcloudFromTsdfLayer(tsdf_map_->getTsdfLayer(), &pointcloud);
  pointcloud.header.frame_id = world_frame_;
  tsdf_pointcloud_pub_.publish(pointcloud);
}

void TsdfServer::publishTsdfSurfacePoints() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  const float surface_distance_thresh =
      tsdf_map_->getTsdfLayer().voxel_size() * 0.75;
  createSurfacePointcloudFromTsdfLayer(tsdf_map_->getTsdfLayer(),
                                       surface_distance_thresh, &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  surface_pointcloud_pub_.publish(pointcloud);
}

void TsdfServer::publishTsdfOccupiedNodes() {
  // Create a pointcloud with distance = intensity.
  visualization_msgs::MarkerArray marker_array;
  createOccupancyBlocksFromTsdfLayer(tsdf_map_->getTsdfLayer(), world_frame_,
                                     &marker_array);
  occupancy_marker_pub_.publish(marker_array);
}

void TsdfServer::publishSlices() {
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  
  createDistancePointcloudFromTsdfLayerSlice(tsdf_map_->getTsdfLayer(), 2,
                                             slice_level_, &pointcloud);
  pointcloud.header.frame_id = world_frame_;
  tsdf_slice_pub_.publish(pointcloud);

  pcl::PointCloud<pcl::PointXYZI> pointcloud_newly_occupied;
  pcl::PointCloud<pcl::PointXYZI> pointcloud_newly_occupied_distance;

  if (cluster_matching_active_) {
    /*createDistancePointcloudFromTsdfLayerSlice(tsdf_map_newly_occupied_->getTsdfLayer(), 2, slice_level_, &pointcloud_newly_occupied);
    pointcloud_newly_occupied.header.frame_id = world_frame_;
    tsdf_newly_occupied_pointcloud_pub_.publish(pointcloud_newly_occupied);

    createDistancePointcloudFromTsdfLayerSlice(tsdf_map_newly_occupied_distance_->getTsdfLayer(), 2, slice_level_, &pointcloud_newly_occupied_distance);
    pointcloud_newly_occupied_distance.header.frame_id = world_frame_;
    tsdf_newly_occupied_distance_pointcloud_pub_.publish(pointcloud_newly_occupied_distance);
    */
    clustered_pcl_.header.frame_id = world_frame_;
    clustered_pointcloud_pub_.publish(clustered_pcl_);
  }
}

void TsdfServer::publishMap(bool reset_remote_map) {
  if (!publish_tsdf_map_) {
    return;
  }
  int subscribers = this->tsdf_map_pub_.getNumSubscribers();
  if (subscribers > 0) {
    if (num_subscribers_tsdf_map_ < subscribers) {
      // Always reset the remote map and send all when a new subscriber
      // subscribes. A bit of overhead for other subscribers, but better than
      // inconsistent map states.
      reset_remote_map = true;
    }
    const bool only_updated = !reset_remote_map;
    timing::Timer publish_map_timer("map/publish_tsdf");
    voxblox_msgs::Layer layer_msg;
    serializeLayerAsMsg<TsdfVoxel>(this->tsdf_map_->getTsdfLayer(),
                                   only_updated, &layer_msg);
    if (reset_remote_map) {
      layer_msg.action = static_cast<uint8_t>(MapDerializationAction::kReset);
    }
    this->tsdf_map_pub_.publish(layer_msg);
    publish_map_timer.Stop();
  }
  num_subscribers_tsdf_map_ = subscribers;
}

void TsdfServer::publishPointclouds() {
  // Combined function to publish all possible pointcloud messages -- surface
  // pointclouds, updated points, and occupied points.
  publishAllUpdatedTsdfVoxels();
  publishTsdfSurfacePoints();
  publishTsdfOccupiedNodes();
  if (publish_slices_) {
    publishSlices();
  }
}

void TsdfServer::updateMesh() {
  if (verbose_) {
    ROS_INFO("Updating mesh.");
  }

  timing::Timer generate_mesh_timer("mesh/update");
  constexpr bool only_mesh_updated_blocks = true;
  constexpr bool clear_updated_flag = true;
  mesh_integrator_->generateMesh(only_mesh_updated_blocks, clear_updated_flag);
  generate_mesh_timer.Stop();

  timing::Timer publish_mesh_timer("mesh/publish");

  voxblox_msgs::Mesh mesh_msg;
  generateVoxbloxMeshMsg(mesh_layer_, color_mode_, &mesh_msg);
  mesh_msg.header.frame_id = world_frame_;
  mesh_pub_.publish(mesh_msg);

  if (cache_mesh_) {
    cached_mesh_msg_ = mesh_msg;
  }

  publish_mesh_timer.Stop();

  if (publish_pointclouds_ && !publish_pointclouds_on_update_) {
    publishPointclouds();
  }
}

bool TsdfServer::generateMesh() {
  timing::Timer generate_mesh_timer("mesh/generate");
  const bool clear_mesh = true;
  if (clear_mesh) {
    constexpr bool only_mesh_updated_blocks = false;
    constexpr bool clear_updated_flag = true;
    mesh_integrator_->generateMesh(only_mesh_updated_blocks,
                                   clear_updated_flag);
  } else {
    constexpr bool only_mesh_updated_blocks = true;
    constexpr bool clear_updated_flag = true;
    mesh_integrator_->generateMesh(only_mesh_updated_blocks,
                                   clear_updated_flag);
  }
  generate_mesh_timer.Stop();

  timing::Timer publish_mesh_timer("mesh/publish");
  voxblox_msgs::Mesh mesh_msg;
  generateVoxbloxMeshMsg(mesh_layer_, color_mode_, &mesh_msg);
  mesh_msg.header.frame_id = world_frame_;
  mesh_pub_.publish(mesh_msg);

  publish_mesh_timer.Stop();

  if (!mesh_filename_.empty()) {
    timing::Timer output_mesh_timer("mesh/output");
    const bool success = outputMeshLayerAsPly(mesh_filename_, *mesh_layer_);
    output_mesh_timer.Stop();
    if (success) {
      ROS_INFO("Output file as PLY: %s", mesh_filename_.c_str());
    } else {
      ROS_INFO("Failed to output mesh as PLY: %s", mesh_filename_.c_str());
    }
  }

  ROS_INFO_STREAM("Mesh Timings: " << std::endl << timing::Timing::Print());
  return true;
}

bool TsdfServer::saveMap(const std::string& file_path) {
  // Inheriting classes should add saving other layers to this function.
  return io::SaveLayer(tsdf_map_->getTsdfLayer(), file_path);
}

bool TsdfServer::loadMap(const std::string& file_path) {
  // Inheriting classes should add other layers to load, as this will only
  // load
  // the TSDF layer.
  constexpr bool kMulitpleLayerSupport = true;
  bool success = io::LoadBlocksFromFile(
      file_path, Layer<TsdfVoxel>::BlockMergingStrategy::kReplace,
      kMulitpleLayerSupport, tsdf_map_->getTsdfLayerPtr());
  if (success) {
    LOG(INFO) << "Successfully loaded TSDF layer.";
  }
  return success;
}

bool TsdfServer::clearMapCallback(std_srvs::Empty::Request& /*request*/,
                                  std_srvs::Empty::Response&
                                  /*response*/) {  // NOLINT
  clear();
  return true;
}

bool TsdfServer::generateMeshCallback(std_srvs::Empty::Request& /*request*/,
                                      std_srvs::Empty::Response&
                                      /*response*/) {  // NOLINT
  return generateMesh();
}

bool TsdfServer::saveMapCallback(voxblox_msgs::FilePath::Request& request,
                                 voxblox_msgs::FilePath::Response&
                                 /*response*/) {  // NOLINT
  return saveMap(request.file_path);
}

bool TsdfServer::loadMapCallback(voxblox_msgs::FilePath::Request& request,
                                 voxblox_msgs::FilePath::Response&
                                 /*response*/) {  // NOLINT
  bool success = loadMap(request.file_path);
  return success;
}

bool TsdfServer::publishPointcloudsCallback(
    std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response&
    /*response*/) {  // NOLINT
  publishPointclouds();
  return true;
}

bool TsdfServer::publishTsdfMapCallback(std_srvs::Empty::Request& /*request*/,
                                        std_srvs::Empty::Response&
                                        /*response*/) {  // NOLINT
  publishMap();
  return true;
}

void TsdfServer::updateMeshEvent(const ros::TimerEvent& /*event*/) {
  updateMesh();
}

void TsdfServer::publishMapEvent(const ros::TimerEvent& /*event*/) {
  publishMap();
}

void TsdfServer::clear() {
  tsdf_map_->getTsdfLayerPtr()->removeAllBlocks();
  mesh_layer_->clear();

  // Publish a message to reset the map to all subscribers.
  if (publish_tsdf_map_) {
    constexpr bool kResetRemoteMap = true;
    publishMap(kResetRemoteMap);
  }
}

void TsdfServer::tsdfMapCallback(const voxblox_msgs::Layer& layer_msg) {
  timing::Timer receive_map_timer("map/receive_tsdf");

  bool success =
      deserializeMsgToLayer<TsdfVoxel>(layer_msg, tsdf_map_->getTsdfLayerPtr());

  if (!success) {
    ROS_ERROR_THROTTLE(10, "Got an invalid TSDF map message!");
  } else {
    ROS_INFO_ONCE("Got an TSDF map from ROS topic!");
    if (publish_pointclouds_on_update_) {
      publishPointclouds();
    }
  }
}

}  // namespace voxblox

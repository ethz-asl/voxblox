#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/esdf_map.h>

#include "voxblox_ros/esdf_server.h"
#include "voxblox_ros/mesh_pcl.h"
#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"
#include "voxblox_ros/conversions.h"

namespace voxblox {

class SkeletonizerNode {
 public:
  SkeletonizerNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : nh_(nh), nh_private_(nh_private), esdf_server_(nh_, nh_private_) {
    skeleton_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ> >(
        "skeleton", 1, true);
  }

  // Initialize the node.
  void init();

  // Make a skeletor!!!
  void skeletonize(Layer<EsdfVoxel>* esdf_layer, voxblox::Pointcloud* skeleton);

 private:
  void getNeighborsAndDistances(
      const BlockIndex& block_index, const VoxelIndex& voxel_index,
      AlignedVector<VoxelKey>* neighbors, AlignedVector<float>* distances,
      AlignedVector<Eigen::Vector3i>* directions) const;

  void getNeighbor(const BlockIndex& block_index, const VoxelIndex& voxel_index,
                   const Eigen::Vector3i& direction,
                   BlockIndex* neighbor_block_index,
                   VoxelIndex* neighbor_voxel_index) const;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher skeleton_pub_;

  EsdfServer esdf_server_;

  int esdf_voxels_per_side_;
};

void SkeletonizerNode::init() {
  // Load a file from the params.
  std::string input_filepath;
  nh_private_.param("input_filepath", input_filepath, input_filepath);

  if (input_filepath.empty()) {
    return;
  }

  esdf_server_.loadMap(input_filepath);

  // Visualize all parts.
  esdf_server_.updateMesh();
  esdf_server_.publishPointclouds();
  esdf_server_.publishMap();

  // Skeletonize????
  esdf_voxels_per_side_ =
      esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxels_per_side();
  voxblox::Pointcloud skeleton;
  skeletonize(esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr(), &skeleton);

  // Publish the skeleton.
  pcl::PointCloud<pcl::PointXYZ> ptcloud_pcl;
  pointcloudToPclXYZ(skeleton, &ptcloud_pcl);
  ptcloud_pcl.header.frame_id = "world";
  skeleton_pub_.publish(ptcloud_pcl);

  // Optionally save back to file.
}

void SkeletonizerNode::skeletonize(Layer<EsdfVoxel>* esdf_layer,
                                   voxblox::Pointcloud* skeleton) {
  // Iterate over all blocks in the ESDF...
  // So should be wavefront ends, but can just check parent direction.
  // Maybe a minimum angle between parent directions of neighbors?
  BlockIndexList blocks;
  esdf_layer->getAllAllocatedBlocks(&blocks);

  for (const BlockIndex& block_index : blocks) {
    const Block<EsdfVoxel>::Ptr& esdf_block =
        esdf_layer->getBlockPtrByIndex(block_index);

    const size_t num_voxels_per_block = esdf_block->num_voxels();

    for (size_t lin_index = 0u; lin_index < num_voxels_per_block; ++lin_index) {
      const EsdfVoxel& esdf_voxel =
          esdf_block->getVoxelByLinearIndex(lin_index);
      VoxelIndex voxel_index =
          esdf_block->computeVoxelIndexFromLinearIndex(lin_index);

      if (!esdf_voxel.observed || esdf_voxel.distance < 0.0f ||
          esdf_voxel.fixed) {
        continue;
      }

      // Get the floating-point distance of this voxel, normalized.
      const Eigen::Vector3f parent_dir =
          esdf_voxel.parent.cast<float>().normalized();

      // See what the neighbors are pointing to. This is 26-connectivity
      // which is probably completely unnecessary.
      AlignedVector<VoxelKey> neighbors;
      AlignedVector<float> distances;
      AlignedVector<Eigen::Vector3i> directions;
      getNeighborsAndDistances(block_index, voxel_index, &neighbors, &distances,
                               &directions);

      // Just go though the 6-connectivity set of this to start.
      const float min_connectivity_distance = 1.0;
      for (size_t i = 0; i < neighbors.size(); ++i) {
        if (distances[i] > min_connectivity_distance) {
          continue;
        }

        // Get this voxel with way too many checks.
        // Get the block for this voxel.
        BlockIndex neighbor_block_index = neighbors[i].first;
        VoxelIndex neighbor_voxel_index = neighbors[i].second;
        Block<EsdfVoxel>::Ptr neighbor_block;
        if (neighbor_block_index == block_index) {
          neighbor_block = esdf_block;
        } else {
          neighbor_block = esdf_layer->getBlockPtrByIndex(neighbor_block_index);
        }
        if (!neighbor_block) {
          continue;
        }
        CHECK(neighbor_block->isValidVoxelIndex(neighbor_voxel_index))
            << "Neigbor voxel index: " << neighbor_voxel_index.transpose();

        EsdfVoxel& neighbor_voxel =
            neighbor_block->getVoxelByVoxelIndex(neighbor_voxel_index);

        if (!neighbor_voxel.observed || neighbor_voxel.distance < 0.0f ||
            neighbor_voxel.fixed) {
          continue;
        }

        // Get the relative distance: that is, if this voxel was on top of
        // ours, what direction would it point in, roughly?
        // Make sure this is a double so we can normalize it.
        Eigen::Vector3f relative_direction =
            neighbor_voxel.parent.cast<float>() - directions[i].cast<float>();

        // Compute the dot product between the two...
        float dot_prod = relative_direction.dot(parent_dir);
        if (dot_prod <= -0.7) {
          // Then this is a ridge or something! Probably. Who knows.
          Point coords =
              esdf_block->computeCoordinatesFromVoxelIndex(voxel_index);
          skeleton->push_back(coords);
          break;
        }
      }
    }
  }
}

// Copied from ESDF integrator... Probably best to factor this out into
// something.
// Uses 26-connectivity and quasi-Euclidean distances.
// Directions is the direction that the neighbor voxel lives in. If you
// need the direction FROM the neighbor voxel TO the current voxel, take
// negative of the given direction.
void SkeletonizerNode::getNeighborsAndDistances(
    const BlockIndex& block_index, const VoxelIndex& voxel_index,
    AlignedVector<VoxelKey>* neighbors, AlignedVector<float>* distances,
    AlignedVector<Eigen::Vector3i>* directions) const {
  CHECK_NOTNULL(neighbors);
  CHECK_NOTNULL(distances);
  CHECK_NOTNULL(directions);

  static const double kSqrt2 = std::sqrt(2);
  static const double kSqrt3 = std::sqrt(3);
  static const size_t kNumNeighbors = 26;

  neighbors->reserve(kNumNeighbors);
  distances->reserve(kNumNeighbors);
  directions->reserve(kNumNeighbors);

  VoxelKey neighbor;
  Eigen::Vector3i direction;
  direction.setZero();
  // Distance 1 set.
  for (unsigned int i = 0; i < 3; ++i) {
    for (int j = -1; j <= 1; j += 2) {
      direction(i) = j;
      getNeighbor(block_index, voxel_index, direction, &neighbor.first,
                  &neighbor.second);
      neighbors->emplace_back(neighbor);
      distances->emplace_back(1.0);
      directions->emplace_back(direction);
    }
    direction(i) = 0;
  }

  // Distance sqrt(2) set.
  for (unsigned int i = 0; i < 3; ++i) {
    unsigned int next_i = (i + 1) % 3;
    for (int j = -1; j <= 1; j += 2) {
      direction(i) = j;
      for (int k = -1; k <= 1; k += 2) {
        direction(next_i) = k;
        getNeighbor(block_index, voxel_index, direction, &neighbor.first,
                    &neighbor.second);
        neighbors->emplace_back(neighbor);
        distances->emplace_back(kSqrt2);
        directions->emplace_back(direction);
      }
      direction(i) = 0;
      direction(next_i) = 0;
    }
  }

  // Distance sqrt(3) set.
  for (int i = -1; i <= 1; i += 2) {
    direction(0) = i;
    for (int j = -1; j <= 1; j += 2) {
      direction(1) = j;
      for (int k = -1; k <= 1; k += 2) {
        direction(2) = k;
        getNeighbor(block_index, voxel_index, direction, &neighbor.first,
                    &neighbor.second);
        neighbors->emplace_back(neighbor);
        distances->emplace_back(kSqrt3);
        directions->emplace_back(direction);
      }
    }
  }

  CHECK_EQ(neighbors->size(), kNumNeighbors);
}

void SkeletonizerNode::getNeighbor(const BlockIndex& block_index,
                                   const VoxelIndex& voxel_index,
                                   const Eigen::Vector3i& direction,
                                   BlockIndex* neighbor_block_index,
                                   VoxelIndex* neighbor_voxel_index) const {
  DCHECK(neighbor_block_index != NULL);
  DCHECK(neighbor_voxel_index != NULL);

  *neighbor_block_index = block_index;
  *neighbor_voxel_index = voxel_index + direction;

  for (unsigned int i = 0; i < 3; ++i) {
    if ((*neighbor_voxel_index)(i) < 0) {
      (*neighbor_block_index)(i)--;
      (*neighbor_voxel_index)(i) += esdf_voxels_per_side_;
    } else if ((*neighbor_voxel_index)(i) >=
               static_cast<IndexElement>(esdf_voxels_per_side_)) {
      (*neighbor_block_index)(i)++;
      (*neighbor_voxel_index)(i) -= esdf_voxels_per_side_;
    }
  }
}

}  // namespace voxblox

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox_skeletonizer");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  FLAGS_alsologtostderr = true;

  voxblox::SkeletonizerNode node(nh, nh_private);

  node.init();

  ros::spin();
  return 0;
}

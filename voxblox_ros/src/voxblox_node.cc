#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>

namespace voxblox {

// TODO(helenol): Split into a ROS wrapper/server and a node that actually
//                sets settings, etc. Follow open_chisel model.
class VoxbloxNode {
 public:
  VoxbloxNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : nh_(nh), nh_private_(nh_private), world_frame_("world") {
    // Advertise topics.
    sdf_marker_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
        "sdf_markers", 1, true);
    surface_pointcloud_pub_ =
        nh_private_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
            "surface_pointcloud", 1, true);
    sdf_pointcloud_pub_ =
        nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
            "sdf_pointcloud", 1, true);

    pointcloud_sub_ = nh_.subscribe("pointcloud", 40,
                                    &VoxbloxNode::insertPointcloudWithTf, this);

    // Determine map parameters.
    TsdfMap::Config config;

    // Workaround for OS X on mac mini not having specializations for float
    // for some reason.
    double voxel_size = config.tsdf_voxel_size;
    int voxels_per_side = config.tsdf_voxels_per_side;
    nh_private_.param("tsdf_voxel_size", voxel_size, voxel_size);
    nh_private_.param("tsdf_voxels_per_side", voxels_per_side, voxels_per_side);
    config.tsdf_voxel_size = static_cast<float>(voxel_size);
    config.tsdf_voxels_per_side = voxels_per_side;
    tsdf_map_.reset(new TsdfMap(config));

    // Determine integrator parameters.
    TsdfIntegrator::Config integrator_config;
    integrator_config.voxel_carving_enabled = true;
    integrator_config.default_truncation_distance = config.tsdf_voxel_size * 4;

    double truncation_distance = integrator_config.default_truncation_distance;
    double max_weight = integrator_config.max_weight;
    nh_private_.param("voxel_carving_enabled",
                      integrator_config.voxel_carving_enabled,
                      integrator_config.voxel_carving_enabled);
    nh_private_.param("truncation_distance", truncation_distance,
                      truncation_distance);
    nh_private_.param("max_weight", max_weight, max_weight);
    integrator_config.default_truncation_distance =
        static_cast<float>(truncation_distance);
    integrator_config.max_weight = static_cast<float>(max_weight);

    tsdf_integrator_.reset(
        new TsdfIntegrator(tsdf_map_->getTsdfLayerPtr(), integrator_config));

    // Advertise services.
    generate_mesh_srv_ = nh_private_.advertiseService(
        "generate_mesh", &VoxbloxNode::generateMeshCallback, this);

    ros::spinOnce();
    publishTsdfSurfacePoints();
  }

  void insertPointcloudWithTf(const sensor_msgs::PointCloud2::Ptr& pointcloud);

  void publishAllUpdatedTsdfVoxels();

  bool lookupTransform(const std::string& from_frame,
                       const std::string& to_frame, const ros::Time& timestamp,
                       Transformation* transform);

  void publishTsdfSurfacePoints();

  bool generateMeshCallback(std_srvs::Empty::Request& request,
                            std_srvs::Empty::Response& response);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Global/map coordinate frame. Will always look up TF transforms to this
  // frame.
  std::string world_frame_;

  // To be replaced (at least optionally) with odometry + static transform from
  // IMU to visual frame.
  tf::TransformListener tf_listener_;

  // Data subscribers.
  ros::Subscriber pointcloud_sub_;

  // Publish markers for visualization.
  ros::Publisher sdf_marker_pub_;
  ros::Publisher sdf_pointcloud_pub_;
  ros::Publisher surface_pointcloud_pub_;

  // Services.
  ros::ServiceServer generate_mesh_srv_;

  std::shared_ptr<TsdfMap> tsdf_map_;
  std::shared_ptr<TsdfIntegrator> tsdf_integrator_;
};

void VoxbloxNode::insertPointcloudWithTf(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg) {
  // Look up transform from sensor frame to world frame.
  Transformation T_G_C;
  if (lookupTransform(pointcloud_msg->header.frame_id, world_frame_,
                      pointcloud_msg->header.stamp, &T_G_C)) {
    // Convert the PCL pointcloud into our awesome format.
    // TODO(helenol): improve...
    // Horrible hack fix to fix color parsing colors in PCL.
    for (size_t d = 0; d < pointcloud_msg->fields.size(); ++d) {
      if (pointcloud_msg->fields[d].name == std::string("rgb")) {
        pointcloud_msg->fields[d].datatype = sensor_msgs::PointField::FLOAT32;
      }
    }

    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);

    timing::Timer ptcloud_timer("ptcloud_preprocess");

    // Filter out NaNs. :|
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(pointcloud_pcl, pointcloud_pcl, indices);

    Pointcloud points_C;
    Colors colors;
    points_C.reserve(pointcloud_pcl.size());
    colors.reserve(pointcloud_pcl.size());
    for (size_t i = 0; i < pointcloud_pcl.points.size(); ++i) {
      points_C.push_back(Point(pointcloud_pcl.points[i].x,
                               pointcloud_pcl.points[i].y,
                               pointcloud_pcl.points[i].z));
      colors.push_back(
          Color(pointcloud_pcl.points[i].r, pointcloud_pcl.points[i].g,
                pointcloud_pcl.points[i].b, pointcloud_pcl.points[i].a));
    }

    ptcloud_timer.Stop();

    ROS_INFO("Integrating a pointcloud with %lu points.", points_C.size());
    ros::WallTime start = ros::WallTime::now();
    tsdf_integrator_->integratePointCloudMerged(T_G_C, points_C, colors);
    ros::WallTime end = ros::WallTime::now();
    ROS_INFO("Finished integrating in %f seconds, have %lu blocks.",
             (end - start).toSec(),
             tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks());
    publishAllUpdatedTsdfVoxels();
    publishTsdfSurfacePoints();

    ROS_INFO_STREAM("Timings: " << std::endl << timing::Timing::Print());
  }
}

void VoxbloxNode::publishAllUpdatedTsdfVoxels() {
  DCHECK(tsdf_map_) << "TSDF map not allocated.";

  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  // Iterate over all voxels to create a pointcloud.
  // TODO(helenol): move this to general IO, replace ply writer with writing
  // this out.
  size_t num_blocks = tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks();
  // This function is block-specific:
  size_t vps = tsdf_map_->getTsdfLayer().voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  pointcloud.reserve(num_blocks * num_voxels_per_block);

  BlockIndexList blocks;
  tsdf_map_->getTsdfLayer().getAllAllocatedBlocks(&blocks);

  // Iterate over all blocks.
  for (const BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const Block<TsdfVoxel>& block =
        tsdf_map_->getTsdfLayer().getBlockByIndex(index);

    VoxelIndex voxel_index = VoxelIndex::Zero();
    for (voxel_index.x() = 0; voxel_index.x() < vps; ++voxel_index.x()) {
      for (voxel_index.y() = 0; voxel_index.y() < vps; ++voxel_index.y()) {
        for (voxel_index.z() = 0; voxel_index.z() < vps; ++voxel_index.z()) {
          const TsdfVoxel& voxel = block.getVoxelByVoxelIndex(voxel_index);

          float distance = voxel.distance;
          float weight = voxel.weight;

          // Get back the original coordinate of this voxel.
          Point coord = block.computeCoordinatesFromVoxelIndex(voxel_index);

          if (weight > 0.0) {
            pcl::PointXYZI point;
            point.x = coord.x();
            point.y = coord.y();
            point.z = coord.z();
            point.intensity = distance;
            pointcloud.push_back(point);
          }
        }
      }
    }
  }

  pointcloud.header.frame_id = world_frame_;
  sdf_pointcloud_pub_.publish(pointcloud);
}

void VoxbloxNode::publishTsdfSurfacePoints() {
  DCHECK(tsdf_map_) << "TSDF map not allocated.";

  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  // Iterate over all voxels to create a pointcloud.
  // TODO(helenol): move this to general IO, replace ply writer with writing
  // this out.
  size_t num_blocks = tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks();
  // This function is block-specific:
  size_t vps = tsdf_map_->getTsdfLayer().voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  pointcloud.reserve(num_blocks * num_voxels_per_block);

  BlockIndexList blocks;
  tsdf_map_->getTsdfLayer().getAllAllocatedBlocks(&blocks);

  // Iterate over all blocks.
  const float surface_distance_thresh =
      tsdf_map_->getTsdfLayer().voxel_size() * 0.5;
  for (const BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const Block<TsdfVoxel>& block =
        tsdf_map_->getTsdfLayer().getBlockByIndex(index);

    VoxelIndex voxel_index = VoxelIndex::Zero();
    for (voxel_index.x() = 0; voxel_index.x() < vps; ++voxel_index.x()) {
      for (voxel_index.y() = 0; voxel_index.y() < vps; ++voxel_index.y()) {
        for (voxel_index.z() = 0; voxel_index.z() < vps; ++voxel_index.z()) {
          const TsdfVoxel& voxel = block.getVoxelByVoxelIndex(voxel_index);

          float distance = voxel.distance;
          float weight = voxel.weight;
          Color color = voxel.color;

          // Get back the original coordinate of this voxel.
          Point coord = block.computeCoordinatesFromVoxelIndex(voxel_index);

          if (std::abs(distance) < surface_distance_thresh && weight > 0) {
            pcl::PointXYZRGB point;
            point.x = coord.x();
            point.y = coord.y();
            point.z = coord.z();
            point.r = color.r;
            point.g = color.g;
            point.b = color.b;
            pointcloud.push_back(point);
          }
        }
      }
    }
  }

  pointcloud.header.frame_id = world_frame_;
  surface_pointcloud_pub_.publish(pointcloud);
}

// Stolen from octomap_manager
bool VoxbloxNode::lookupTransform(const std::string& from_frame,
                                  const std::string& to_frame,
                                  const ros::Time& timestamp,
                                  Transformation* transform) {
  tf::StampedTransform tf_transform;

  ros::Time time_to_lookup = timestamp;

  // If this transform isn't possible at the time, then try to just look up
  // the latest (this is to work with bag files and static transform publisher,
  // etc).
  if (!tf_listener_.canTransform(to_frame, from_frame, time_to_lookup)) {
    time_to_lookup = ros::Time(0);
    ROS_WARN("Using latest TF transform instead of timestamp match.");
  }

  try {
    tf_listener_.lookupTransform(to_frame, from_frame, time_to_lookup,
                                 tf_transform);
  } catch (tf::TransformException& ex) {  // NOLINT
    ROS_ERROR_STREAM(
        "Error getting TF transform from sensor data: " << ex.what());
    return false;
  }

  tf::transformTFToKindr(tf_transform, transform);
  return true;
}

bool VoxbloxNode::generateMeshCallback(std_srvs::Empty::Request& request,
                                       std_srvs::Empty::Response& response) {
  MeshLayer::Ptr mesh_layer(new MeshLayer(tsdf_map_->block_size()));

  MeshIntegrator mesh_integrator(tsdf_map_->getTsdfLayerPtr(),
                                 mesh_layer.get());

  timing::Timer generate_mesh_timer("mesh/generate");
  mesh_integrator.generateWholeMesh();
  generate_mesh_timer.Stop();

  bool success = outputMeshLayerAsPly("/Users/helen/mesh.ply", mesh_layer);
  ROS_INFO("Output file as PLY.");

  ROS_INFO_STREAM("Timings: " << std::endl << timing::Timing::Print());

  return success;
}

}  // namespace voxblox

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox_node");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  voxblox::VoxbloxNode node(nh, nh_private);

  ros::spin();
  return 0;
}

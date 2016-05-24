#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <minkindr_conversions/kindr_xml.h>
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
#include <deque>

#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>

#include "voxblox_ros/visualization.h"

namespace voxblox {

class VoxbloxNode {
 public:
  VoxbloxNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  void insertPointcloudWithTf(const sensor_msgs::PointCloud2::Ptr& pointcloud);

  void publishAllUpdatedTsdfVoxels();

  bool lookupTransform(const std::string& from_frame,
                       const std::string& to_frame, const ros::Time& timestamp,
                       Transformation* transform);
  bool lookupTransformTf(const std::string& from_frame,
                         const std::string& to_frame,
                         const ros::Time& timestamp, Transformation* transform);
  bool lookupTransformQueue(const std::string& from_frame,
                            const std::string& to_frame,
                            const ros::Time& timestamp,
                            Transformation* transform);

  void publishTsdfSurfacePoints();

  void transformCallback(const geometry_msgs::TransformStamped& transform_msg);
  bool generateMeshCallback(std_srvs::Empty::Request& request,     // NOLINT
                            std_srvs::Empty::Response& response);  // NOLINT
  void updateMeshEvent(const ros::TimerEvent& e);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  bool verbose_;

  // Global/map coordinate frame. Will always look up TF transforms to this
  // frame.
  std::string world_frame_;
  // Whether to use TF transform resolution (true) or fixed transforms from
  // parameters and transform topics (false).
  bool use_tf_transforms_;
  int64_t timestamp_tolerance_ns_;
  Transformation T_B_C_;
  Transformation T_B_D_;

  // Mesh output settings. Mesh is only written to file if mesh_filename_ is not
  // empty.
  std::string mesh_filename_;
  // How to color the mesh.
  ColorMode color_mode_;

  // To be replaced (at least optionally) with odometry + static transform from
  // IMU to visual frame.
  tf::TransformListener tf_listener_;

  // Data subscribers.
  ros::Subscriber pointcloud_sub_;
  // Only used if use_tf_transforms_ set to false.
  ros::Subscriber transform_sub_;

  // Publish markers for visualization.
  ros::Publisher mesh_pub_;
  ros::Publisher sdf_pointcloud_pub_;
  ros::Publisher surface_pointcloud_pub_;

  // Services.
  ros::ServiceServer generate_mesh_srv_;

  // Timers.
  ros::Timer update_mesh_timer_;

  // Maps and integrators.
  std::shared_ptr<TsdfMap> tsdf_map_;
  std::shared_ptr<TsdfIntegrator> tsdf_integrator_;
  // Mesh accessories.
  std::shared_ptr<MeshLayer> mesh_layer_;
  std::shared_ptr<MeshIntegrator> mesh_integrator_;

  // Transform queue, used only when use_tf_transforms is false.
  std::deque<geometry_msgs::TransformStamped> transform_queue_;
};

VoxbloxNode::VoxbloxNode(const ros::NodeHandle& nh,
                         const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(true),
      world_frame_("world"),
      use_tf_transforms_(true),
      // 10 ms here:
      timestamp_tolerance_ns_(10000000) {
  // Advertise topics.
  mesh_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("mesh", 1, true);
  surface_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
          "surface_pointcloud", 1, true);
  sdf_pointcloud_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "sdf_pointcloud", 1, true);

  pointcloud_sub_ = nh_.subscribe("pointcloud", 40,
                                  &VoxbloxNode::insertPointcloudWithTf, this);

  nh_private_.param("verbose", verbose_, verbose_);

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

  // Mesh settings.
  nh_private_.param("mesh_filename", mesh_filename_, mesh_filename_);
  std::string color_mode("color");
  nh_private_.param("color_mode", color_mode, color_mode);
  if (color_mode == "color") {
    color_mode_ = ColorMode::kColor;
  } else if (color_mode == "height") {
    color_mode_ = ColorMode::kHeight;
  } else if (color_mode == "normals") {
    color_mode_ = ColorMode::kNormals;
  } else if (color_mode == "lambert") {
    color_mode_ = ColorMode::kLambert;
  } else {  // Default case is gray.
    color_mode_ = ColorMode::kGray;
  }

  mesh_layer_.reset(new MeshLayer(tsdf_map_->block_size()));

  MeshIntegrator::Config mesh_config;
  mesh_integrator_.reset(new MeshIntegrator(tsdf_map_->getTsdfLayerPtr(),
                                            mesh_layer_.get(), mesh_config));

  // Advertise services.
  generate_mesh_srv_ = nh_private_.advertiseService(
      "generate_mesh", &VoxbloxNode::generateMeshCallback, this);

  // If set, use a timer to progressively integrate the mesh.
  double update_mesh_every_n_sec = 0.0;
  nh_private_.param("update_mesh_every_n_sec", update_mesh_every_n_sec,
                    update_mesh_every_n_sec);

  if (update_mesh_every_n_sec > 0.0) {
    update_mesh_timer_ =
        nh_private_.createTimer(ros::Duration(update_mesh_every_n_sec),
                                &VoxbloxNode::updateMeshEvent, this);
  }

  // Transform settings.
  nh_private_.param("use_tf_transforms", use_tf_transforms_,
                    use_tf_transforms_);
  // If we use topic transforms, we have 2 parts: a dynamic transform from a
  // topic and a static transform from parameters.
  // Static transform should be T_G_D (where D is whatever sensor the
  // dynamic coordinate frame is in) and the static should be T_D_C (where
  // C is the sensor frame that produces the depth data). It is possible to
  // specific T_C_D and set invert_static_tranform to true.
  if (!use_tf_transforms_) {
    transform_sub_ =
        nh_.subscribe("transform", 40, &VoxbloxNode::transformCallback, this);
    // Retrieve T_D_C from params.
    Eigen::Matrix4d transform_mat;
    transform_mat.setIdentity();
    XmlRpc::XmlRpcValue T_B_D_xml;
    // TODO(helenol): split out into a function to avoid duplication.
    if (nh_private_.getParam("T_B_D", T_B_D_xml)) {
      kindr::minimal::xmlRpcToKindr(T_B_D_xml, &T_B_D_);

      // See if we need to invert it.
      bool invert_static_tranform = false;
      nh_private_.param("invert_T_B_D", invert_static_tranform,
                        invert_static_tranform);
      if (invert_static_tranform) {
        T_B_D_ = T_B_D_.inverse();
      }
    }
    transform_mat.setIdentity();
    XmlRpc::XmlRpcValue T_B_C_xml;
    if (nh_private_.getParam("T_B_C", T_B_C_xml)) {
      kindr::minimal::xmlRpcToKindr(T_B_C_xml, &T_B_C_);

      // See if we need to invert it.
      bool invert_static_tranform = false;
      nh_private_.param("invert_T_B_C", invert_static_tranform,
                        invert_static_tranform);
      if (invert_static_tranform) {
        T_B_C_ = T_B_C_.inverse();
      }
    }

    ROS_INFO_STREAM("Static transforms loaded from file.\nT_B_D:\n"
                    << T_B_D_ << "\nT_B_C:" << T_B_C_);
  }

  ros::spinOnce();
  publishTsdfSurfacePoints();
}

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

    if (verbose_) {
      ROS_INFO("Integrating a pointcloud with %lu points.", points_C.size());
    }
    ros::WallTime start = ros::WallTime::now();
    tsdf_integrator_->integratePointCloudMerged(T_G_C, points_C, colors);
    ros::WallTime end = ros::WallTime::now();
    if (verbose_) {
      ROS_INFO("Finished integrating in %f seconds, have %lu blocks.",
               (end - start).toSec(),
               tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks());
    }
    publishAllUpdatedTsdfVoxels();
    publishTsdfSurfacePoints();

    if (verbose_) {
      ROS_INFO_STREAM("Timings: " << std::endl << timing::Timing::Print());
    }
  }
}

void VoxbloxNode::transformCallback(
    const geometry_msgs::TransformStamped& transform_msg) {
  transform_queue_.push_back(transform_msg);
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
            point.intensity = weight;
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

bool VoxbloxNode::lookupTransform(const std::string& from_frame,
                                  const std::string& to_frame,
                                  const ros::Time& timestamp,
                                  Transformation* transform) {
  if (use_tf_transforms_) {
    return lookupTransformTf(from_frame, to_frame, timestamp, transform);
  } else {
    return lookupTransformQueue(from_frame, to_frame, timestamp, transform);
  }
}

// Stolen from octomap_manager
bool VoxbloxNode::lookupTransformTf(const std::string& from_frame,
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

bool VoxbloxNode::lookupTransformQueue(const std::string& from_frame,
                                       const std::string& to_frame,
                                       const ros::Time& timestamp,
                                       Transformation* transform) {
  // Try to match the transforms in the queue.
  bool match_found = false;
  std::deque<geometry_msgs::TransformStamped>::iterator it =
      transform_queue_.begin();
  for (; it != transform_queue_.end(); ++it) {
    // If the current transform is newer than the requested timestamp, we need
    // to break.
    if (it->header.stamp > timestamp) {
      if ((it->header.stamp - timestamp).toNSec() < timestamp_tolerance_ns_) {
        match_found = true;
      }
      break;
    }

    if ((timestamp - it->header.stamp).toNSec() < timestamp_tolerance_ns_) {
      match_found = true;
      break;
    }
  }

  if (match_found) {
    Transformation T_G_D;
    tf::transformMsgToKindr(it->transform, &T_G_D);

    // If we have a static transform, apply it too.
    // Transform should actually be T_G_C. So need to take it through the full
    // chain.
    *transform = T_G_D * T_B_D_.inverse() * T_B_C_;

    // And also clear the queue up to this point. This leaves the current
    // message in place.
    transform_queue_.erase(transform_queue_.begin(), it);
  } else {
    ROS_WARN_STREAM_THROTTLE(
        30, "No match found for transform timestamp: " << timestamp);
    if (!transform_queue_.empty()) {
      ROS_WARN_STREAM_THROTTLE(
          30,
          "Queue front: " << transform_queue_.front().header.stamp
                          << " back: " << transform_queue_.back().header.stamp);
    }
  }
  return match_found;
}

bool VoxbloxNode::generateMeshCallback(
    std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& response) {  // NOLINT
  timing::Timer generate_mesh_timer("mesh/generate");
  const bool clear_mesh = true;
  if (clear_mesh) {
    mesh_integrator_->generateWholeMesh();
  } else {
    const bool clear_updated_flag = true;
    mesh_integrator_->generateMeshForUpdatedBlocks(clear_updated_flag);
  }
  generate_mesh_timer.Stop();

  timing::Timer publish_mesh_timer("mesh/publish");
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(1);
  fillMarkerWithMesh(mesh_layer_, color_mode_, &marker_array.markers[0]);
  mesh_pub_.publish(marker_array);
  publish_mesh_timer.Stop();

  if (!mesh_filename_.empty()) {
    timing::Timer output_mesh_timer("mesh/output");
    bool success = outputMeshLayerAsPly(mesh_filename_, mesh_layer_);
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

void VoxbloxNode::updateMeshEvent(const ros::TimerEvent& e) {
  if (verbose_) {
    ROS_INFO("Updating mesh.");
  }
  timing::Timer generate_mesh_timer("mesh/update");
  const bool clear_updated_flag = true;
  mesh_integrator_->generateMeshForUpdatedBlocks(clear_updated_flag);
  generate_mesh_timer.Stop();

  // TODO(helenol): also think about how to update markers incrementally?
  timing::Timer publish_mesh_timer("mesh/publish");
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(1);
  fillMarkerWithMesh(mesh_layer_, color_mode_, &marker_array.markers[0]);
  mesh_pub_.publish(marker_array);
  publish_mesh_timer.Stop();
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

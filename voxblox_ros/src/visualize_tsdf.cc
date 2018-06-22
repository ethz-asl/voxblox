#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>

#include "voxblox_ros/mesh_pcl.h"
#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"

namespace voxblox {
class SimpleTsdfVisualizer {
 public:
  explicit SimpleTsdfVisualizer(const ros::NodeHandle& nh_private)
      : nh_private_(nh_private),
        tsdf_surface_distance_threshold_factor_(2.0),
        tsdf_world_frame_("world"),
        tsdf_mesh_color_mode_(ColorMode::kColor),
        tsdf_voxel_ply_output_path_("") {
    ROS_DEBUG_STREAM("\tSetting up ROS publishers...");

    surface_pointcloud_pub_ =
        nh_private_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
            "tsdf_voxels_near_surface", 1, true);

    tsdf_pointcloud_pub_ =
        nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI>>(
            "all_tsdf_voxels", 1, true);

    mesh_pub_ = nh_private_.advertise<voxblox_msgs::Mesh>("mesh", 1, true);

    mesh_pointcloud_pub_ =
        nh_private_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
            "mesh_as_pointcloud", 1, true);

    mesh_pcl_mesh_pub_ =
        nh_private_.advertise<pcl_msgs::PolygonMesh>("mesh_pcl", 1, true);

    ROS_DEBUG_STREAM("\tRetreiving ROS parameters...");

    nh_private_.param("tsdf_surface_distance_threshold_factor",
                      tsdf_surface_distance_threshold_factor_,
                      tsdf_surface_distance_threshold_factor_);
    nh_private_.param("tsdf_world_frame", tsdf_world_frame_, tsdf_world_frame_);
    nh_private_.param("tsdf_voxel_ply_output_path", tsdf_voxel_ply_output_path_,
                      tsdf_voxel_ply_output_path_);
    nh_private_.param("tsdf_mesh_output_path", tsdf_mesh_output_path_,
                      tsdf_mesh_output_path_);

    std::string color_mode = "color";
    nh_private_.param("tsdf_mesh_color_mode", color_mode, color_mode);
    if (color_mode == "color") {
      tsdf_mesh_color_mode_ = ColorMode::kColor;
    } else if (color_mode == "height") {
      tsdf_mesh_color_mode_ = ColorMode::kHeight;
    } else if (color_mode == "normals") {
      tsdf_mesh_color_mode_ = ColorMode::kNormals;
    } else if (color_mode == "lambert") {
      tsdf_mesh_color_mode_ = ColorMode::kLambert;
    } else if (color_mode == "gray") {
      tsdf_mesh_color_mode_ = ColorMode::kGray;
    } else {
      ROS_FATAL_STREAM("Undefined mesh coloring mode: " << color_mode);
      ros::shutdown();
    }

    ros::spinOnce();
  }

  void run(const Layer<TsdfVoxel>& tsdf_layer);

 private:
  ros::NodeHandle nh_private_;

  ros::Publisher surface_pointcloud_pub_;
  ros::Publisher tsdf_pointcloud_pub_;
  ros::Publisher mesh_pub_;
  ros::Publisher mesh_pointcloud_pub_;
  ros::Publisher mesh_pcl_mesh_pub_;

  // Settings
  double tsdf_surface_distance_threshold_factor_;
  std::string tsdf_world_frame_;
  ColorMode tsdf_mesh_color_mode_;
  std::string tsdf_voxel_ply_output_path_;
  std::string tsdf_mesh_output_path_;
};

void SimpleTsdfVisualizer::run(const Layer<TsdfVoxel>& tsdf_layer) {
  ROS_INFO_STREAM("\nTSDF Layer info:\n"
                  << "\tVoxel size:\t\t " << tsdf_layer.voxel_size() << "\n"
                  << "\t# Voxels per side:\t " << tsdf_layer.voxels_per_side()
                  << "\n"
                  << "\tMemory size:\t\t "
                  << tsdf_layer.getMemorySize() / 1024 / 1024 << "MB\n"
                  << "\t# Allocated blocks:\t "
                  << tsdf_layer.getNumberOfAllocatedBlocks() << "\n");

  ROS_DEBUG_STREAM("\tVisualize voxels near surface...");
  {
    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    const FloatingPoint surface_distance_thresh_m =
        tsdf_layer.voxel_size() * tsdf_surface_distance_threshold_factor_;
    voxblox::createSurfaceDistancePointcloudFromTsdfLayer(
        tsdf_layer, surface_distance_thresh_m, &pointcloud);

    pointcloud.header.frame_id = tsdf_world_frame_;
    surface_pointcloud_pub_.publish(pointcloud);
  }

  ROS_DEBUG_STREAM("\tVisualize all voxels...");
  {
    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    voxblox::createDistancePointcloudFromTsdfLayer(tsdf_layer, &pointcloud);

    pointcloud.header.frame_id = tsdf_world_frame_;
    tsdf_pointcloud_pub_.publish(pointcloud);

    if (!tsdf_voxel_ply_output_path_.empty()) {
      pcl::PLYWriter writer;
      constexpr bool kUseBinary = true;
      writer.write(tsdf_voxel_ply_output_path_, pointcloud, kUseBinary);
    }
  }

  ROS_DEBUG_STREAM("\tVisualize mesh...");
  {
    std::shared_ptr<MeshLayer> mesh_layer;
    mesh_layer.reset(new MeshLayer(tsdf_layer.block_size()));
    MeshIntegratorConfig mesh_config;
    std::shared_ptr<MeshIntegrator<TsdfVoxel>> mesh_integrator;
    mesh_integrator.reset(new MeshIntegrator<TsdfVoxel>(mesh_config, tsdf_layer,
                                                        mesh_layer.get()));

    constexpr bool kOnlyMeshUpdatedBlocks = false;
    constexpr bool kClearUpdatedFlag = false;
    mesh_integrator->generateMesh(kOnlyMeshUpdatedBlocks, kClearUpdatedFlag);

    // Output as native voxblox mesh.
    voxblox_msgs::Mesh mesh_msg;
    generateVoxbloxMeshMsg(mesh_layer, tsdf_mesh_color_mode_, &mesh_msg);
    mesh_msg.header.frame_id = tsdf_world_frame_;
    mesh_pub_.publish(mesh_msg);

    // Output as point cloud.
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
    fillPointcloudWithMesh(mesh_layer, tsdf_mesh_color_mode_, &pointcloud);
    pointcloud.header.frame_id = tsdf_world_frame_;
    mesh_pointcloud_pub_.publish(pointcloud);

    // Output as pcl mesh.
    pcl::PolygonMesh polygon_mesh;
    toConnectedPCLPolygonMesh(*mesh_layer, tsdf_world_frame_, &polygon_mesh);
    pcl_msgs::PolygonMesh pcl_mesh_msg;
    pcl_conversions::fromPCL(polygon_mesh, pcl_mesh_msg);
    mesh_msg.header.stamp = ros::Time::now();
    mesh_pcl_mesh_pub_.publish(mesh_msg);

    if (!tsdf_mesh_output_path_.empty()) {
      if (voxblox::outputMeshLayerAsPly(tsdf_mesh_output_path_, *mesh_layer)) {
        ROS_INFO_STREAM("Output mesh PLY file to " << tsdf_mesh_output_path_);
      }
    }
  }
}

}  // namespace voxblox

int main(int argc, char** argv) {
  ros::init(argc, argv, "visualize_tsdf_node");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh_private("~");

  std::string tsdf_proto_path = "";
  nh_private.param("tsdf_proto_path", tsdf_proto_path, tsdf_proto_path);
  if (tsdf_proto_path.empty()) {
    ROS_FATAL_STREAM(
        "Please provide a TSDF proto file to visualize using the ros "
        << "parameter: tsdf_proto_path");
    ros::shutdown();
    return 1;
  }
  ROS_INFO_STREAM("Visualize TSDF grid from " << tsdf_proto_path);

  ROS_INFO_STREAM("Loading...");
  voxblox::Layer<voxblox::TsdfVoxel>::Ptr tsdf_layer;
  if (!voxblox::io::LoadLayer<voxblox::TsdfVoxel>(tsdf_proto_path,
                                                  &tsdf_layer)) {
    ROS_FATAL_STREAM("Unable to load a TSDF grid from: " << tsdf_proto_path);
    ros::shutdown();
    return 1;
  }
  CHECK(tsdf_layer);
  ROS_INFO_STREAM("Done.");

  ROS_INFO_STREAM("Visualizing...");
  voxblox::SimpleTsdfVisualizer visualizer(nh_private);
  visualizer.run(*tsdf_layer);
  ROS_INFO_STREAM("Done.");

  ros::spin();

  return 0;
}

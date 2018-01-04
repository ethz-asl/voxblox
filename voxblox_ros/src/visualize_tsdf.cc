#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/conversions.h>
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

DEFINE_string(tsdf_proto_path, "", "Path to the serialized TSDF grid.");

DEFINE_double(tsdf_surface_distance_threshold_factor, 0.75,
              "This factor multiplied with the voxel size determines the "
              "maximum distance to the surface for voxels to be considered "
              "near-surface voxels.");

DEFINE_string(tsdf_world_frame, "world",
              "Name of the world frame. This frame will be used to publish all "
              "the pointclouds and meshes.");

DEFINE_string(tsdf_mesh_color_mode, "color",
              "Color mode for the TSDF mesh extraction, options: "
              "[color, height, normals, lambert, gray]");

namespace voxblox {

class SimpleTsdfVisualizer {
 public:
  SimpleTsdfVisualizer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private)
      : nh_(nh), nh_private_(nh_private) {
    VLOG(1) << "\tSetting up ROS publishers...";

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

    ros::spinOnce();
  }

  void run(const Layer<TsdfVoxel>& tsdf_layer);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher surface_pointcloud_pub_;
  ros::Publisher tsdf_pointcloud_pub_;
  ros::Publisher mesh_pub_;
  ros::Publisher mesh_pointcloud_pub_;
  ros::Publisher mesh_pcl_mesh_pub_;
};

void SimpleTsdfVisualizer::run(const Layer<TsdfVoxel>& tsdf_layer) {
  LOG(INFO) << "\nTSDF Layer info:\n"
            << "\tVoxel size:\t\t " << tsdf_layer.voxel_size() << "\n"
            << "\t# Voxels per side:\t " << tsdf_layer.voxels_per_side() << "\n"
            << "\tMemory size:\t\t " << tsdf_layer.getMemorySize() * 1e-6
            << "MB\n"
            << "\t# Allocated blocks:\t "
            << tsdf_layer.getNumberOfAllocatedBlocks() << "\n";

  VLOG(1) << "\tVisualize voxels near surface...";
  {
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
    const float surface_distance_thresh_m =
        tsdf_layer.voxel_size() * FLAGS_tsdf_surface_distance_threshold_factor;
    voxblox::createSurfacePointcloudFromTsdfLayer(
        tsdf_layer, surface_distance_thresh_m, &pointcloud);

    pointcloud.header.frame_id = FLAGS_tsdf_world_frame;
    surface_pointcloud_pub_.publish(pointcloud);
  }

  VLOG(1) << "\tVisualize all voxels...";
  {
    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    voxblox::createDistancePointcloudFromTsdfLayer(tsdf_layer, &pointcloud);

    pointcloud.header.frame_id = FLAGS_tsdf_world_frame;
    tsdf_pointcloud_pub_.publish(pointcloud);
  }

  VLOG(1) << "\tVisualize mesh...";
  {
    std::shared_ptr<MeshLayer> mesh_layer;
    mesh_layer.reset(new MeshLayer(tsdf_layer.block_size()));
    MeshIntegrator<TsdfVoxel>::Config mesh_config;
    std::shared_ptr<MeshIntegrator<TsdfVoxel>> mesh_integrator;
    mesh_integrator.reset(new MeshIntegrator<TsdfVoxel>(mesh_config, tsdf_layer,
                                                        mesh_layer.get()));

    constexpr bool kOnlyMeshUpdatedBlocks = false;
    constexpr bool kClearUpdatedFlag = false;
    mesh_integrator->generateMesh(kOnlyMeshUpdatedBlocks, kClearUpdatedFlag);

    const std::string color_mode_string = FLAGS_tsdf_mesh_color_mode;
    ColorMode color_mode;
    if (color_mode_string == "color") {
      color_mode = ColorMode::kColor;
    } else if (color_mode_string == "height") {
      color_mode = ColorMode::kHeight;
    } else if (color_mode_string == "normals") {
      color_mode = ColorMode::kNormals;
    } else if (color_mode_string == "lambert") {
      color_mode = ColorMode::kLambert;
    } else if (color_mode_string == "gray") {
      color_mode = ColorMode::kGray;
    } else {
      LOG(FATAL) << "Undefined mesh coloring mode: "
                 << FLAGS_tsdf_mesh_color_mode;
    }

    // Output as native voxblox mesh.
    voxblox_msgs::Mesh mesh_msg;
    generateVoxbloxMeshMsg(mesh_layer, color_mode, &mesh_msg);
    mesh_msg.header.frame_id = FLAGS_tsdf_world_frame;
    mesh_pub_.publish(mesh_msg);

    // Output as point cloud.
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
    fillPointcloudWithMesh(mesh_layer, color_mode, &pointcloud);
    pointcloud.header.frame_id = FLAGS_tsdf_world_frame;
    mesh_pointcloud_pub_.publish(pointcloud);

    // Output as pcl mesh.
    pcl::PolygonMesh polygon_mesh;
    toPCLPolygonMesh(*mesh_layer, FLAGS_tsdf_world_frame, &polygon_mesh);
    pcl_msgs::PolygonMesh pcl_mesh_msg;
    pcl_conversions::fromPCL(polygon_mesh, pcl_mesh_msg);
    mesh_msg.header.stamp = ros::Time::now();
    mesh_pcl_mesh_pub_.publish(mesh_msg);
  }
}

}  // namespace voxblox

int main(int argc, char** argv) {
  ros::init(argc, argv, "visualize_tsdf_node");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  if (FLAGS_tsdf_proto_path.empty()) {
    LOG(FATAL) << "Please provide a TSDF proto file to visualize using "
               << "--tsdf_proto_path";
  }

  LOG(INFO) << "Visualize TSDF grid from " << FLAGS_tsdf_proto_path;

  VLOG(1) << "Loading...";
  voxblox::Layer<voxblox::TsdfVoxel>::Ptr tsdf_layer;
  if (!voxblox::io::LoadLayer<voxblox::TsdfVoxel>(FLAGS_tsdf_proto_path,
                                                  &tsdf_layer)) {
    LOG(FATAL) << "Unable to load a TSDF grid from: " << FLAGS_tsdf_proto_path;
  }
  CHECK(tsdf_layer);
  VLOG(1) << "Done.";

  VLOG(1) << "Visualizing...";
  voxblox::SimpleTsdfVisualizer visualizer(nh, nh_private);
  visualizer.run(*tsdf_layer);
  VLOG(1) << "Done.";

  ros::spin();

  return 0;
}

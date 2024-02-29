#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/msg/polygon_mesh.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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
  explicit SimpleTsdfVisualizer()
      : Node("voxblox"),
        tsdf_surface_distance_threshold_factor_(2.0),
        tsdf_world_frame_("world"),
        tsdf_mesh_color_mode_(ColorMode::kColor),
        tsdf_voxel_ply_output_path_("") {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "\tSetting up ROS publishers...");

    surface_pointcloud_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "tsdf_voxels_near_surface", 1);

    tsdf_pointcloud_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("all_tsdf_voxels",
                                                              1);

    mesh_pub_ = this->create_publisher<voxblox_msgs::msg::Mesh>("mesh", 1);

    mesh_pointcloud_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "mesh_as_pointcloud", 1);

    mesh_pcl_mesh_pub_ =
        this->create_publisher<pcl_msgs::msg::PolygonMesh>("mesh_pcl", 1);

    RCLCPP_DEBUG_STREAM(this->get_logger(), "\tRetreiving ROS parameters...");

    tsdf_surface_distance_threshold_factor_ =
        this->declare_parameter("tsdf_surface_distance_threshold_factor",
                                tsdf_surface_distance_threshold_factor_);
    tsdf_world_frame_ =
        this->declare_parameter("tsdf_world_frame", tsdf_world_frame_);
    tsdf_voxel_ply_output_path_ = this->declare_parameter(
        "tsdf_voxel_ply_output_path", tsdf_voxel_ply_output_path_);
    tsdf_mesh_output_path_ = this->declare_parameter("tsdf_mesh_output_path",
                                                     tsdf_mesh_output_path_);

    std::string color_mode = "color";
    color_mode = this->declare_parameter("tsdf_mesh_color_mode", color_mode);
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
      RCLCPP_FATAL_STREAM(this->get_logger(),
                          "Undefined mesh coloring mode: " << color_mode);
      rclcpp::shutdown();
    }

    rclcpp::spin_some(this);
  }

  void run(const Layer<TsdfVoxel>& tsdf_layer);

 private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      surface_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      tsdf_pointcloud_pub_;
  rclcpp::Publisher<voxblox_msgs::msg::Mesh>::SharedPtr mesh_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      mesh_pointcloud_pub_;
  rclcpp::Publisher<pcl_msgs::msg::PolygonMesh>::SharedPtr mesh_pcl_mesh_pub_;

  // Settings
  double tsdf_surface_distance_threshold_factor_;
  std::string tsdf_world_frame_;
  ColorMode tsdf_mesh_color_mode_;
  std::string tsdf_voxel_ply_output_path_;
  std::string tsdf_mesh_output_path_;
};

void SimpleTsdfVisualizer::run(const Layer<TsdfVoxel>& tsdf_layer) {
  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "\nTSDF Layer info:\n"
          << "\tVoxel size:\t\t " << tsdf_layer.voxel_size() << "\n"
          << "\t# Voxels per side:\t " << tsdf_layer.voxels_per_side() << "\n"
          << "\tMemory size:\t\t " << tsdf_layer.getMemorySize() / 1024 / 1024
          << "MB\n"
          << "\t# Allocated blocks:\t "
          << tsdf_layer.getNumberOfAllocatedBlocks() << "\n");

  RCLCPP_DEBUG_STREAM(this->get_logger(), "\tVisualize voxels near surface...");
  {
    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    const FloatingPoint surface_distance_thresh_m =
        tsdf_layer.voxel_size() * tsdf_surface_distance_threshold_factor_;
    voxblox::createSurfaceDistancePointcloudFromTsdfLayer(
        tsdf_layer, surface_distance_thresh_m, &pointcloud);

    pointcloud.header.frame_id = tsdf_world_frame_;

    sensor_msgs::msg::PointCloud2 pointcloud_message;
    pcl::toROSMsg(pointcloud, pointcloud_message);

    surface_pointcloud_pub_->publish(pointcloud_message);
  }

  RCLCPP_DEBUG_STREAM(this->get_logger(), "\tVisualize all voxels...");
  {
    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    voxblox::createDistancePointcloudFromTsdfLayer(tsdf_layer, &pointcloud);

    pointcloud.header.frame_id = tsdf_world_frame_;

    sensor_msgs::msg::PointCloud2 pointcloud_message;
    pcl::toROSMsg(pointcloud, pointcloud_message);

    tsdf_pointcloud_pub_->publish(pointcloud_message);

    if (!tsdf_voxel_ply_output_path_.empty()) {
      pcl::PLYWriter writer;
      constexpr bool kUseBinary = true;
      writer.write(tsdf_voxel_ply_output_path_, pointcloud, kUseBinary);
    }
  }

  RCLCPP_DEBUG_STREAM(this->get_logger(), "\tVisualize mesh...");
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
    voxblox_msgs::msg::Mesh mesh_msg;
    generateVoxbloxMeshMsg(mesh_layer, tsdf_mesh_color_mode_, &mesh_msg);
    mesh_msg.header.frame_id = tsdf_world_frame_;
    mesh_pub_->publish(mesh_msg);

    // Output as point cloud.
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud;

    fillPointcloudWithMesh(mesh_layer, tsdf_mesh_color_mode_, &pointcloud);
    pointcloud.header.frame_id = tsdf_world_frame_;

    sensor_msgs::msg::PointCloud2 pointcloud_message;
    pcl::toROSMsg(pointcloud, pointcloud_message);

    mesh_pointcloud_pub_->publish(pointcloud_message);

    // Output as pcl mesh.
    pcl::PolygonMesh polygon_mesh;
    toConnectedPCLPolygonMesh(*mesh_layer, tsdf_world_frame_, &polygon_mesh);
    pcl_msgs::msg::PolygonMesh pcl_mesh_msg;
    pcl_conversions::fromPCL(polygon_mesh, pcl_mesh_msg);
    mesh_msg.header.stamp = rclcpp::Clock().now();
    mesh_pcl_mesh_pub_->publish(pcl_mesh_msg);

    if (!tsdf_mesh_output_path_.empty()) {
      if (voxblox::outputMeshLayerAsPly(tsdf_mesh_output_path_, *mesh_layer)) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Output mesh PLY file to "
                                                   << tsdf_mesh_output_path_);
      }
    }
  }
}

}  // namespace voxblox

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  // google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  auto node = std::make_shared<voxblox::SimpleTsdfVisualizer>();

  std::string tsdf_proto_path = "";
  tsdf_proto_path = node->declare_parameter("tsdf_proto_path", tsdf_proto_path);
  if (tsdf_proto_path.empty()) {
    RCLCPP_FATAL_STREAM(
        this->get_logger(),
        "Please provide a TSDF proto file to visualize using the ros "
            << "parameter: tsdf_proto_path");
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Visualize TSDF grid from " << tsdf_proto_path);

  RCLCPP_INFO_STREAM(this->get_logger(), "Loading...");
  voxblox::Layer<voxblox::TsdfVoxel>::Ptr tsdf_layer;
  if (!voxblox::io::LoadLayer<voxblox::TsdfVoxel>(tsdf_proto_path,
                                                  &tsdf_layer)) {
    RCLCPP_FATAL_STREAM(this->get_logger(),
                        "Unable to load a TSDF grid from: " << tsdf_proto_path);
    rclcpp::shutdown();
    return 1;
  }
  CHECK(tsdf_layer);
  RCLCPP_INFO_STREAM(this->get_logger(), "Done.");

  RCLCPP_INFO_STREAM(this->get_logger(), "Visualizing...");

  visualizer.run(*tsdf_layer);
  RCLCPP_INFO_STREAM(this->get_logger(), "Done.");

  rclcpp::spin(node);

  return 0;
}

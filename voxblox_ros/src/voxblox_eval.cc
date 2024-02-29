#include <deque>

#include <gflags/gflags.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <minkindr_conversions/kindr_xml.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <voxblox/core/esdf_map.h>
#include <voxblox/core/occupancy_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/occupancy_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>

#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"

// This binary evaluates a pre-built voxblox map against a provided ground
// truth dataset, provided as pointcloud, and outputs a variety of statistics.
namespace voxblox {

class VoxbloxEvaluator {
 public:
  VoxbloxEvaluator();
  void evaluate();
  void visualize();
  bool shouldExit() const { return !visualize_; }

 private:
  // Whether to do the visualizations (involves generating mesh of the TSDF
  // layer) and keep alive (for visualization) after finishing the eval.
  // Otherwise only outputs the evaluations statistics.
  bool visualize_;
  // Whether to recolor the voxels by error to the GT before generating a mesh.
  bool recolor_by_error_;
  // How to color the mesh.
  ColorMode color_mode_;
  // If visualizing, what TF frame to visualize in.
  std::string frame_id_;

  // Transformation between the ground truth dataset and the voxblox map.
  // The GT is transformed INTO the voxblox coordinate frame.
  Transformation T_V_G_;

  // Visualization publishers.
  rclcpp::Publisher mesh_pub_;
  rclcpp::Publisher gt_ptcloud_pub_;

  // Core data to compare.
  std::shared_ptr<Layer<TsdfVoxel>> tsdf_layer_;
  pcl::PointCloud<pcl::PointXYZRGB> gt_ptcloud_;

  // Interpolator to get the distance at the exact point in the GT.
  Interpolator<TsdfVoxel>::Ptr interpolator_;

  // Mesh visualization.
  std::shared_ptr<MeshLayer> mesh_layer_;
  std::shared_ptr<MeshIntegrator<TsdfVoxel>> mesh_integrator_;
};

VoxbloxEvaluator::VoxbloxEvaluator()
    : Node("voxblox"),
      visualize_(true),
      recolor_by_error_(false),
      frame_id_("world") {
  // Load parameters.
  visualize_ = this->declare_parameter("visualize", visualize_);
  recolor_by_error_ =
      this->declare_parameter("recolor_by_error", recolor_by_error_);
  frame_id_ = this->declare_parameter("frame_id", frame_id_);

  // Load transformations.
  // XmlRpc::XmlRpcValue T_V_G_xml;
  // if (this->getParam("T_V_G", T_V_G_xml)) {
  //   kindr::minimal::vectorOfVectorsToKindr(T_V_G_xml, &T_V_G_);
  //   bool invert_static_tranform = false;
  //   invert_static_tranform = this->declare_parameter("invert_T_V_G",
  //   invert_static_tranform); if (invert_static_tranform) {
  //     T_V_G_ = T_V_G_.inverse();
  //   }
  // }

  // Load the actual map and GT.
  // Just exit if there's any issues here (this is just an evaluation node,
  // after all).
  std::string voxblox_file_path, gt_file_path;
  voxblox_file_path_ = get_parameter("voxblox_file_path").as_string();
  CHECK(voxblox_file_path_)
      << "No file path provided for voxblox map! Set the \"voxblox_file_path\" "
         "param.";

  gt_file_path = get_parameter("gt_file_path").as_string();
  CHECK(gt_file_path)
      << "No file path provided for ground truth pointcloud! Set the "
         "\"gt_file_path\" param.";

  CHECK(io::LoadLayer<TsdfVoxel>(voxblox_file_path, &tsdf_layer_))
      << "Could not load voxblox map.";
  pcl::PLYReader ply_reader;

  CHECK_EQ(ply_reader.read(gt_file_path, gt_ptcloud_), 0)
      << "Could not load pointcloud ground truth.";

  // Initialize the interpolator.
  interpolator_.reset(new Interpolator<TsdfVoxel>(tsdf_layer_.get()));

  // If doing visualizations, initialize the publishers.
  if (visualize_) {
    mesh_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("mesh", 1);
    gt_ptcloud_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("gt_ptcloud", 1);

    std::string color_mode("color");
    color_mode = this->declare_parameter("color_mode", color_mode);
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
  }

  std::cout << "Evaluating " << voxblox_file_path << std::endl;
}

void VoxbloxEvaluator::evaluate() {
  // First, transform the pointcloud into the correct coordinate frame.
  // First, rotate the pointcloud into the world frame.
  pcl::transformPointCloud(gt_ptcloud_, gt_ptcloud_,
                           T_V_G_.getTransformationMatrix());

  // Go through each point, use trilateral interpolation to figure out the
  // distance at that point.
  // This double-counts -- multiple queries to the same voxel will be counted
  // separately.
  uint64_t total_evaluated_voxels = 0;
  uint64_t unknown_voxels = 0;
  uint64_t outside_truncation_voxels = 0;

  // TODO(helenol): make this dynamic.
  double truncation_distance = 2 * tsdf_layer_->voxel_size();

  double mse = 0.0;

  for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it =
           gt_ptcloud_.begin();
       it != gt_ptcloud_.end(); ++it) {
    Point point(it->x, it->y, it->z);

    FloatingPoint distance = 0.0;
    float weight = 0.0;
    bool valid = false;

    const float min_weight = 0.01;
    const bool interpolate = true;
    // We will do multiple lookups -- the first is to determine whether the
    // voxel exists.
    if (!interpolator_->getNearestDistanceAndWeight(point, &distance,
                                                    &weight)) {
      unknown_voxels++;
    } else if (weight <= min_weight) {
      unknown_voxels++;
    } else if (distance >= truncation_distance) {
      outside_truncation_voxels++;
      mse += truncation_distance * truncation_distance;
      valid = true;
    } else {
      // In case this fails, distance is still the nearest neighbor distance.
      interpolator_->getDistance(point, &distance, interpolate);
      mse += distance * distance;
      valid = true;
    }

    if (valid && visualize_ && recolor_by_error_) {
      Layer<TsdfVoxel>::BlockType::Ptr block_ptr =
          tsdf_layer_->getBlockPtrByCoordinates(point);
      if (block_ptr != nullptr) {
        TsdfVoxel& voxel = block_ptr->getVoxelByCoordinates(point);
        voxel.color = grayColorMap(std::fabs(distance) / truncation_distance);
      }
    }

    total_evaluated_voxels++;
  }

  double rms = sqrt(mse / (total_evaluated_voxels - unknown_voxels));

  std::cout << "Finished evaluating.\n"
            << "\nRMS Error:           " << rms
            << "\nTotal evaluated:     " << total_evaluated_voxels
            << "\nUnknown voxels:       " << unknown_voxels << " ("
            << static_cast<double>(unknown_voxels) / total_evaluated_voxels
            << ")\nOutside truncation: " << outside_truncation_voxels << " ("
            << outside_truncation_voxels /
                   static_cast<double>(total_evaluated_voxels)
            << ")\n";

  if (visualize_) {
    visualize();
  }
}

void VoxbloxEvaluator::visualize() {
  // Generate the mesh.
  MeshIntegratorConfig mesh_config;
  mesh_layer_.reset(new MeshLayer(tsdf_layer_->block_size()));
  mesh_integrator_.reset(new MeshIntegrator<TsdfVoxel>(
      mesh_config, tsdf_layer_.get(), mesh_layer_.get()));

  constexpr bool only_mesh_updated_blocks = false;
  constexpr bool clear_updated_flag = true;
  mesh_integrator_->generateMesh(only_mesh_updated_blocks, clear_updated_flag);

  // Publish mesh.
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.resize(1);
  marker_array.markers[0].header.frame_id = frame_id_;
  fillMarkerWithMesh(mesh_layer_, color_mode_, &marker_array.markers[0]);
  mesh_pub_->publish(marker_array);

  gt_ptcloud_.header.frame_id = frame_id_;

  sensor_msgs::msg::PointCloud2 pointcloud_message;
  pcl::toROSMsg(gt_ptcloud_, pointcloud_message);

  gt_ptcloud_pub_->publish(pointcloud_message);
  std::cout << "Finished visualizing.\n";
}

}  // namespace voxblox

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  // google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  auto node = std::make_shared<voxblox::VoxbloxEvaluator>();
  node.evaluate();

  if (!node.shouldExit()) {
    rclcpp::spin(node);
  }
  return 0;
}

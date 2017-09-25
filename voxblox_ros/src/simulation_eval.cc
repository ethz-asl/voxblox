#include <ros/ros.h>

#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/esdf_occ_integrator.h>
#include <voxblox/integrator/occupancy_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/simulation/simulation_world.h>

#include "voxblox_ros/conversions.h"
#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"

namespace voxblox {

class SimulationServer {
 public:
  SimulationServer(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);

  // Runs all of the below functions in the correct order:
  void run();

  // Creates a new world, and prepares ground truth SDF(s).
  void prepareWorld();

  // Generates a SDF by generating random poses and putting them into an SDF.
  void generateSDF();

  // Evaluate errors...
  void evaluate();

  // Visualize results. :)
  void visualize();

 private:
  // Convenience function to generate valid viewpoints.
  bool generatePlausibleViewpoint(FloatingPoint min_distance, Point* ray_origin,
                                  Point* ray_direction) const;

  void transformPointcloud(const Transformation& T_N_O,
                           const Pointcloud& ptcloud,
                           Pointcloud* ptcloud_out) const;

  template <typename VoxelType>
  FloatingPoint evaluateLayerAgainstGt(const Layer<VoxelType>& layer_test,
                                       const Layer<VoxelType>& layer_gt) const;

  template <typename VoxelType>
  bool evaluateVoxel(const VoxelType& voxel_test, const VoxelType& voxel_gt,
                     FloatingPoint* error) const;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // A bunch of publishers :)
  ros::Publisher sim_pub_;
  ros::Publisher tsdf_gt_pub_;
  ros::Publisher esdf_gt_pub_;
  ros::Publisher tsdf_gt_mesh_pub_;
  ros::Publisher tsdf_test_pub_;
  ros::Publisher esdf_test_pub_;
  ros::Publisher tsdf_test_mesh_pub_;
  ros::Publisher view_ptcloud_pub_;

  // Settings
  std::string world_frame_;
  bool generate_occupancy_;
  bool visualize_;
  bool generate_mesh_;
  bool incremental_;
  FloatingPoint truncation_distance_;
  FloatingPoint esdf_max_distance_;

  // Actual simulation server.
  SimulationWorld world_;

  // Maps (GT and generates from sensors) generated here.
  std::unique_ptr<Layer<TsdfVoxel> > tsdf_gt_;
  std::unique_ptr<Layer<EsdfVoxel> > esdf_gt_;

  // Generated maps:
  std::unique_ptr<Layer<TsdfVoxel> > tsdf_test_;
  std::unique_ptr<Layer<EsdfVoxel> > esdf_test_;
  std::unique_ptr<Layer<OccupancyVoxel> > occ_test_;

  // Integrators:
  std::unique_ptr<TsdfIntegratorBase> tsdf_integrator_;
  std::unique_ptr<EsdfIntegrator> esdf_integrator_;
  std::unique_ptr<OccupancyIntegrator> occ_integrator_;
  std::unique_ptr<EsdfOccIntegrator> esdf_occ_integrator_;
};

SimulationServer::SimulationServer(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      world_frame_("world"),
      generate_occupancy_(false),
      visualize_(true),
      generate_mesh_(true),
      incremental_(true) {
  // Settings for simulation.
  nh_private_.param("incremental", incremental_, incremental_);
  nh_private_.param("generate_mesh", generate_mesh_, generate_mesh_);
  nh_private_.param("visualize", visualize_, visualize_);
  nh_private_.param("generate_occupancy", generate_occupancy_,
                    generate_occupancy_);

  FloatingPoint voxel_size = 0.1;
  int voxels_per_side = 16;

  nh_private_.param("voxel_size", voxel_size, voxel_size);

  tsdf_gt_.reset(new Layer<TsdfVoxel>(voxel_size, voxels_per_side));
  esdf_gt_.reset(new Layer<EsdfVoxel>(voxel_size, voxels_per_side));

  tsdf_test_.reset(new Layer<TsdfVoxel>(voxel_size, voxels_per_side));
  esdf_test_.reset(new Layer<EsdfVoxel>(voxel_size, voxels_per_side));

  if (generate_occupancy_) {
    occ_test_.reset(new Layer<OccupancyVoxel>(voxel_size, voxels_per_side));
  }

  // Make some integrators.
  TsdfIntegratorBase::Config integrator_config;
  integrator_config.voxel_carving_enabled = true;
  // Used to be * 4 according to Marius's experience, now * 2.
  // This should be made bigger again if behind-surface weighting is improved.
  integrator_config.default_truncation_distance = voxel_size * 4;
  integrator_config.allow_clear = false;
  integrator_config.use_const_weight = false;
  integrator_config.use_weight_dropoff = true;
  integrator_config.max_ray_length_m = 5.0;
  truncation_distance_ = integrator_config.default_truncation_distance;

  nh_private_.param("voxel_carving_enabled",
                    integrator_config.voxel_carving_enabled,
                    integrator_config.voxel_carving_enabled);
  nh_private_.param("truncation_distance", truncation_distance_,
                    truncation_distance_);
  nh_private_.param("max_ray_length_m", integrator_config.max_ray_length_m,
                    integrator_config.max_ray_length_m);
  nh_private_.param("min_ray_length_m", integrator_config.min_ray_length_m,
                    integrator_config.min_ray_length_m);
  nh_private_.param("max_weight", integrator_config.max_weight,
                    integrator_config.max_weight);
  nh_private_.param("use_const_weight", integrator_config.use_const_weight,
                    integrator_config.use_const_weight);
  nh_private_.param("allow_clear", integrator_config.allow_clear,
                    integrator_config.allow_clear);
  integrator_config.default_truncation_distance =
      static_cast<float>(truncation_distance_);

  tsdf_integrator_.reset(
      new MergedTsdfIntegrator(integrator_config, tsdf_test_.get()));

  EsdfIntegrator::Config esdf_integrator_config;
  // Make sure that this is the same as the truncation distance OR SMALLER!
  esdf_integrator_config.min_distance_m = truncation_distance_ / 2.0;
  nh_private_.param("esdf_max_distance_m",
                    esdf_integrator_config.max_distance_m,
                    esdf_integrator_config.max_distance_m);
  esdf_max_distance_ = esdf_integrator_config.max_distance_m;
  esdf_integrator_config.default_distance_m = esdf_max_distance_;

  esdf_integrator_.reset(new EsdfIntegrator(
      esdf_integrator_config, tsdf_test_.get(), esdf_test_.get()));

  if (generate_occupancy_) {
    OccupancyIntegrator::Config occ_integrator_config;
    occ_integrator_config.max_ray_length_m = integrator_config.max_ray_length_m;
    occ_integrator_.reset(
        new OccupancyIntegrator(occ_integrator_config, occ_test_.get()));

    EsdfOccIntegrator::Config esdf_occ_config;
    esdf_occ_config.max_distance_m = esdf_max_distance_;
    esdf_occ_config.default_distance_m = esdf_max_distance_;
    esdf_occ_integrator_.reset(new EsdfOccIntegrator(
        esdf_occ_config, occ_test_.get(), esdf_test_.get()));
  }

  // ROS stuff.
  // GT
  tsdf_gt_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "tsdf_gt", 1, true);
  esdf_gt_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "esdf_gt", 1, true);
  tsdf_gt_mesh_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "tsdf_gt_mesh", 1, true);

  // Test
  tsdf_test_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "tsdf_test", 1, true);
  esdf_test_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "esdf_test", 1, true);
  tsdf_test_mesh_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "tsdf_test_mesh", 1, true);

  view_ptcloud_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
      "view_ptcloud_pub", 1, true);

  // Set random seed to a fixed value.
  srand(0);
}

void SimulationServer::prepareWorld() {
  world_.addObject(std::unique_ptr<Object>(
      new Sphere(Point(0.0, 0.0, 2.0), 2.0, Color::Red())));

  world_.addObject(std::unique_ptr<Object>(
      new Plane(Point(-2.0, -4.0, 2.0), Point(0, 1, 0), Color::White())));

  world_.addObject(std::unique_ptr<Object>(
      new Plane(Point(4.0, 0.0, 0.0), Point(-1, 0, 0), Color::Pink())));

  world_.addObject(std::unique_ptr<Object>(
      new Cube(Point(-4.0, 4.0, 2.0), Point(4, 4, 4), Color::Green())));

  world_.addGroundLevel(0.03);

  world_.generateSdfFromWorld(truncation_distance_, tsdf_gt_.get());
  world_.generateSdfFromWorld(esdf_max_distance_, esdf_gt_.get());
}

bool SimulationServer::generatePlausibleViewpoint(FloatingPoint min_distance,
                                                  Point* ray_origin,
                                                  Point* ray_direction) const {
  // Generate a viewpoint at least min_distance from any objects (if you want
  // just outside an object, just call this with min_distance = 0).
  constexpr int max_tries = 50;

  FloatingPoint max_distance = min_distance * 2.0;

  // Figure out the dimensions of the space.
  Point space_min = world_.getMinBound() / 2.0;
  Point space_size = world_.getMaxBound() - space_min / 2.0;

  Point position = Point::Zero();
  bool success = false;
  // Generate a position, and check if it's ok.
  for (int i = 0; i < max_tries; ++i) {
    position.setRandom();
    // Make this span the whole space.
    position =
        space_size.cwiseProduct(position + Point::Ones()) / 2.0 + space_min;

    if (world_.getDistanceToPoint(position, max_distance) >= min_distance) {
      success = true;
      break;
    }
  }
  if (!success) {
    return false;
  }

  // Now that we have a position, generate the ray direction.
  *ray_origin = position;
  ray_direction->setRandom();
  ray_direction->normalize();
  return true;
}

void SimulationServer::transformPointcloud(const Transformation& T_N_O,
                                           const Pointcloud& ptcloud,
                                           Pointcloud* ptcloud_out) const {
  ptcloud_out->clear();
  ptcloud_out->resize(ptcloud.size());

  for (size_t i = 0; i < ptcloud.size(); ++i) {
    (*ptcloud_out)[i] = T_N_O * ptcloud[i];
  }
}

void SimulationServer::generateSDF() {
  Eigen::Vector2i res(320, 240);
  FloatingPoint fov_h_rad = 1.5708;  // 90 degrees.
  FloatingPoint max_dist = 10.0;
  FloatingPoint min_dist = 0.5;
  constexpr int num_viewpoints = 50;

  Pointcloud ptcloud;
  Colors colors;

  Point view_origin(0.0, 0.0, 2.0);
  Point view_direction(0.0, 1.0, 0.0);
  view_direction.normalize();

  pcl::PointCloud<pcl::PointXYZRGB> ptcloud_pcl;

  for (int i = 0; i < num_viewpoints; ++i) {
    if (!generatePlausibleViewpoint(min_dist, &view_origin, &view_direction)) {
      ROS_WARN(
          "Could not generate enough viewpoints. Generated: %d, Needed: %d", i,
          num_viewpoints);
      break;
    }

    ptcloud.clear();
    colors.clear();

    world_.getPointcloudFromViewpoint(view_origin, view_direction, res,
                                      fov_h_rad, max_dist, &ptcloud, &colors);

    // Get T_G_C from ray origin and ray direction.
    Transformation T_G_C(view_origin,
                         Eigen::Quaternion<FloatingPoint>::FromTwoVectors(
                             Point(0.0, 0.0, 1.0), view_direction));

    // Transform back into camera frame.
    Pointcloud ptcloud_C;
    transformPointcloud(T_G_C.inverse(), ptcloud, &ptcloud_C);

    // Put into the real map.
    tsdf_integrator_->integratePointCloud(T_G_C, ptcloud_C, colors);

    if (generate_occupancy_) {
      occ_integrator_->integratePointCloud(T_G_C, ptcloud_C);
    }

    const bool clear_updated_flag = true;
    if (incremental_) {
      esdf_integrator_->updateFromTsdfLayer(clear_updated_flag);
    }

    // Convert to a XYZRGB pointcloud.
    if (visualize_) {
      ptcloud_pcl.header.frame_id = world_frame_;
      pcl::PointXYZRGB point;
      point.x = view_origin.x();
      point.y = view_origin.y();
      point.z = view_origin.z();
      ptcloud_pcl.push_back(point);

      view_ptcloud_pub_.publish(ptcloud_pcl);
      ros::spinOnce();
    }
  }

  // Generate ESDF in batch.
  if (!incremental_) {
    if (generate_occupancy_) {
      esdf_occ_integrator_->updateFromOccLayerBatch();
    }

    esdf_integrator_->updateFromTsdfLayerBatch();

    // Other batch options for reference:
    // esdf_integrator_->updateFromTsdfLayerBatchFullEuclidean();
    // esdf_integrator_->updateFromTsdfLayerBatchOccupancy();
  }
}

template <typename VoxelType>
FloatingPoint SimulationServer::evaluateLayerAgainstGt(
    const Layer<VoxelType>& layer_test,
    const Layer<VoxelType>& layer_gt) const {
  // Iterate over all voxels in GT set and look them up in the test set.
  // Then compute RMSE.
  BlockIndexList block_list;
  layer_gt.getAllAllocatedBlocks(&block_list);
  size_t vps = layer_gt.voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  double total_squared_error = 0.0;
  size_t num_evaluated_voxels = 0;

  for (const BlockIndex block_index : block_list) {
    if (!layer_test.hasBlock(block_index)) {
      continue;
    }
    const Block<VoxelType>& gt_block = layer_gt.getBlockByIndex(block_index);
    const Block<VoxelType>& test_block =
        layer_test.getBlockByIndex(block_index);

    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
         ++linear_index) {
      FloatingPoint error = 0.0;
      if (evaluateVoxel<VoxelType>(
              test_block.getVoxelByLinearIndex(linear_index),
              gt_block.getVoxelByLinearIndex(linear_index), &error)) {
        total_squared_error += error * error;
        num_evaluated_voxels++;
      }
    }
  }
  // Return the RMSE.
  if (num_evaluated_voxels == 0) {
    return 0.0;
  }
  ROS_INFO_STREAM("Number of voxels evaluated: " << num_evaluated_voxels);
  return sqrt(total_squared_error / num_evaluated_voxels);
}

template <>
bool SimulationServer::evaluateVoxel(const TsdfVoxel& voxel_test,
                                     const TsdfVoxel& voxel_gt,
                                     FloatingPoint* error) const {
  if (voxel_test.weight < 1e-6 || voxel_gt.distance < 0.0) {
    return false;
  }

  *error = voxel_gt.distance - voxel_test.distance;

  /* if (voxel_gt.distance < -truncation_distance_) {
    *error = -truncation_distance_ - voxel_test.distance;
  } */
  return true;
}

template <>
bool SimulationServer::evaluateVoxel(const EsdfVoxel& voxel_test,
                                     const EsdfVoxel& voxel_gt,
                                     FloatingPoint* error) const {
  if (!voxel_test.observed || voxel_gt.distance < 0.0) {
    return false;
  }

  *error = voxel_gt.distance - voxel_test.distance;

  /* if (voxel_gt.distance < -truncation_distance_) {
    *error = -truncation_distance_ - voxel_test.distance;
  } */
  return true;
}

void SimulationServer::evaluate() {
  // First evaluate the TSDF vs ground truth...
  // Use only observed points.
  double tsdf_rmse = evaluateLayerAgainstGt(*tsdf_test_, *tsdf_gt_);
  double esdf_rmse = evaluateLayerAgainstGt(*esdf_test_, *esdf_gt_);

  ROS_INFO_STREAM("TSDF RMSE: " << tsdf_rmse << " ESDF RMSE: " << esdf_rmse);

  ROS_INFO_STREAM("Mesh Timings: " << std::endl << timing::Timing::Print());
}

void SimulationServer::visualize() {
  if (!visualize_) {
    return;
  }
  FloatingPoint slice_level = 2.0;

  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  pointcloud.header.frame_id = world_frame_;
  createDistancePointcloudFromTsdfLayerSlice(*tsdf_gt_, 2, slice_level,
                                             &pointcloud);
  // createDistancePointcloudFromTsdfLayer(*tsdf_gt_, &pointcloud);
  tsdf_gt_pub_.publish(pointcloud);

  pointcloud.clear();
  createDistancePointcloudFromEsdfLayerSlice(*esdf_gt_, 2, slice_level,
                                             &pointcloud);
  // createDistancePointcloudFromEsdfLayer(*esdf_gt_, &pointcloud);
  esdf_gt_pub_.publish(pointcloud);

  pointcloud.clear();
  createDistancePointcloudFromTsdfLayerSlice(*tsdf_test_, 2, slice_level,
                                             &pointcloud);

  // createDistancePointcloudFromTsdfLayer(*tsdf_test_, &pointcloud);
  tsdf_test_pub_.publish(pointcloud);

  pointcloud.clear();
  createDistancePointcloudFromEsdfLayerSlice(*esdf_test_, 2, slice_level,
                                             &pointcloud);
  // createDistancePointcloudFromEsdfLayer(*esdf_test_, &pointcloud);
  esdf_test_pub_.publish(pointcloud);

  if (generate_mesh_) {
    // Generate TSDF GT mesh.
    MeshIntegrator<TsdfVoxel>::Config mesh_config;
    MeshLayer::Ptr mesh(new MeshLayer(tsdf_gt_->block_size()));
    MeshIntegrator<TsdfVoxel> mesh_integrator(mesh_config, tsdf_gt_.get(),
                                              mesh.get());

    constexpr bool only_mesh_updated_blocks = false;
    constexpr bool clear_updated_flag = true;
    mesh_integrator.generateMesh(only_mesh_updated_blocks, clear_updated_flag);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(1);
    ColorMode color_mode = ColorMode::kNormals;
    fillMarkerWithMesh(mesh, color_mode, &marker_array.markers[0]);
    marker_array.markers[0].header.frame_id = world_frame_;
    tsdf_gt_mesh_pub_.publish(marker_array);

    // Also generate test mesh
    MeshLayer::Ptr mesh_test(new MeshLayer(tsdf_test_->block_size()));
    MeshIntegrator<TsdfVoxel> mesh_integrator_test(
        mesh_config, tsdf_test_.get(), mesh_test.get());
    mesh_integrator_test.generateMesh(only_mesh_updated_blocks,
                                      clear_updated_flag);
    marker_array.markers.clear();
    marker_array.markers.resize(1);
    fillMarkerWithMesh(mesh_test, color_mode, &marker_array.markers[0]);
    marker_array.markers[0].header.frame_id = world_frame_;
    tsdf_test_mesh_pub_.publish(marker_array);
  }
}

void SimulationServer::run() {
  prepareWorld();
  generateSDF();
  evaluate();
  visualize();
}

}  // namespace voxblox

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox_sim");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  voxblox::SimulationServer sim_eval(nh, nh_private);

  sim_eval.run();

  ROS_INFO("Done.");
  ros::spin();
  return 0;
}

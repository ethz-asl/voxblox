#include <gtest/gtest.h>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/esdf_integrator.h"
#include "voxblox/integrator/tsdf_integrator.h"
#include "voxblox/simulation/simulation_world.h"
#include "voxblox/utils/evaluation_utils.h"
#include "voxblox/utils/layer_utils.h"
#include "voxblox/io/layer_io.h"

using namespace voxblox;  // NOLINT

DECLARE_bool(alsologtostderr);
DECLARE_bool(logtostderr);
DECLARE_int32(v);

class SdfIntegratorsTest : public ::testing::Test {
 public:
  SdfIntegratorsTest()
      : depth_camera_resolution_(Eigen::Vector2i(320, 240)),
        fov_h_rad_(2.61799),  // 150 degrees
        max_dist_(10.0),
        min_dist_(0.5),
        voxel_size_(0.10),
        voxels_per_side_(16) {}

  virtual void SetUp() {
    // Create a test environment.
    // It consists of a 10x10x7 m environment with an object in the middle.
    Point min_bound(-5.0, -5.0, -1.0);
    Point max_bound(5.0, 5.0, 6.0);
    world_.setBounds(min_bound, max_bound);
    Point cylinder_center(0.0, 0.0, 2.0);
    FloatingPoint cylinder_radius = 2;
    FloatingPoint cylinder_height = 4;
    world_.addObject(std::unique_ptr<Object>(new Cylinder(
        cylinder_center, cylinder_radius, cylinder_height, Color::Red())));
    world_.addGroundLevel(0.0);

    // Next, generate poses evenly spaced in a circle around the object.
    FloatingPoint radius = 4.0;
    FloatingPoint height = 2.0;
    int num_poses = 10;
    poses_.reserve(num_poses);

    FloatingPoint max_angle = 2 * M_PI;

    FloatingPoint angle_increment = max_angle / num_poses;
    for (FloatingPoint angle = 0.0; angle < max_angle;
         angle += angle_increment) {
      // Generate a transformation to look at the center pose.
      Point position(radius * sin(angle), radius * cos(angle), height);
      Point facing_direction = cylinder_center - position;

      FloatingPoint desired_yaw = 0.0;
      if (std::abs(facing_direction.x()) > 1e-4 ||
          std::abs(facing_direction.y()) > 1e-4) {
        desired_yaw = atan2(facing_direction.y(), facing_direction.x());
      }

      // Face the desired yaw and pitch forward a bit to get some of the floor.
      Quaternion rotation =
          Quaternion(Eigen::AngleAxis<FloatingPoint>(-0.0, Point::UnitY())) *
          Eigen::AngleAxis<FloatingPoint>(desired_yaw, Point::UnitZ());

      poses_.emplace_back(Transformation(rotation, position));
    }

    // Finally, get the GT.
    tsdf_gt_.reset(new Layer<TsdfVoxel>(voxel_size_, voxels_per_side_));
    esdf_gt_.reset(new Layer<EsdfVoxel>(voxel_size_, voxels_per_side_));

    truncation_distance_ = 2 * voxel_size_;
    esdf_max_distance_ = 4.0;

    world_.generateSdfFromWorld(truncation_distance_, tsdf_gt_.get());
    world_.generateSdfFromWorld(esdf_max_distance_, esdf_gt_.get());
  }

 protected:
  SimulationWorld world_;

  // Camera settings
  Eigen::Vector2i depth_camera_resolution_;
  FloatingPoint fov_h_rad_;
  FloatingPoint max_dist_;
  FloatingPoint min_dist_;
  int num_viewpoints_;

  // Map settings
  FloatingPoint voxel_size_;
  int voxels_per_side_;
  FloatingPoint truncation_distance_;
  FloatingPoint esdf_max_distance_;

  // Generated viewpoints
  AlignedVector<Transformation> poses_;

  // Maps (GT and generates from sensors) generated here.
  std::unique_ptr<Layer<TsdfVoxel> > tsdf_gt_;
  std::unique_ptr<Layer<EsdfVoxel> > esdf_gt_;
};

TEST_F(SdfIntegratorsTest, TsdfIntegrators) {
  TsdfIntegratorBase::Config config;
  config.default_truncation_distance = truncation_distance_;
  config.integrator_threads = 1;

  // Simple integrator
  Layer<TsdfVoxel> simple_layer(voxel_size_, voxels_per_side_);
  SimpleTsdfIntegrator simple_integrator(config, &simple_layer);

  // Merged integrator
  Layer<TsdfVoxel> merged_layer(voxel_size_, voxels_per_side_);
  MergedTsdfIntegrator merged_integrator(config, &merged_layer);

  // Fast integrator
  Layer<TsdfVoxel> fast_layer(voxel_size_, voxels_per_side_);
  FastTsdfIntegrator fast_integrator(config, &fast_layer);

  for (size_t i = 0; i < poses_.size(); i++) {
    Pointcloud ptcloud, ptcloud_C;
    Colors colors;

    world_.getPointcloudFromTransform(poses_[i], depth_camera_resolution_,
                                      fov_h_rad_, max_dist_, &ptcloud, &colors);
    transformPointcloud(poses_[i].inverse(), ptcloud, &ptcloud_C);
    // std::cout << "Pose: " << poses_[i] << std::endl;
    simple_integrator.integratePointCloud(poses_[i], ptcloud_C, colors);
    merged_integrator.integratePointCloud(poses_[i], ptcloud_C, colors);
    fast_integrator.integratePointCloud(poses_[i], ptcloud_C, colors);
  }

  utils::VoxelEvaluationDetails simple_result, merged_result, fast_result;
  utils::evaluateLayersRmse(*tsdf_gt_, simple_layer,
                            utils::VoxelEvaluationMode::kEvaluateAllVoxels,
                            &simple_result);
  utils::evaluateLayersRmse(*tsdf_gt_, merged_layer,
                            utils::VoxelEvaluationMode::kEvaluateAllVoxels,
                            &merged_result);
  utils::evaluateLayersRmse(*tsdf_gt_, fast_layer,
                            utils::VoxelEvaluationMode::kEvaluateAllVoxels,
                            &fast_result);
  std::cout << "Simple Integrator: " << simple_result.toString();
  std::cout << "Merged Integrator: " << merged_result.toString();
  std::cout << "Fast Integrator: " << fast_result.toString();
  std::cout << "Truncation distance: " << truncation_distance_ << std::endl;

  // Figure out some metrics to compare against, based on voxel size.
  constexpr FloatingPoint kFloatingPointToleranceLow = 1e-6;
  constexpr FloatingPoint kFloatingPointToleranceHigh = 1e-4;
  constexpr FloatingPoint kKindaSimilar = 1e-2;

  size_t total_voxels = simple_result.num_overlapping_voxels +
                        simple_result.num_non_overlapping_voxels;
  size_t one_percent_of_voxels = static_cast<size_t>(total_voxels * 0.01);

  // Make sure they're all similar.
  EXPECT_NEAR(simple_result.num_overlapping_voxels,
              merged_result.num_overlapping_voxels, one_percent_of_voxels);
  EXPECT_NEAR(simple_result.num_overlapping_voxels,
              fast_result.num_overlapping_voxels, one_percent_of_voxels);

  // Make sure they're all reasonable.
  EXPECT_NEAR(simple_result.min_error, 0.0, kFloatingPointToleranceHigh);
  EXPECT_NEAR(merged_result.min_error, 0.0, kFloatingPointToleranceHigh);
  EXPECT_NEAR(fast_result.min_error, 0.0, kFloatingPointToleranceHigh);

  EXPECT_LT(simple_result.max_error, truncation_distance_ * 2);
  EXPECT_LT(merged_result.max_error, truncation_distance_ * 2);
  EXPECT_LT(fast_result.max_error, truncation_distance_ * 2);

  EXPECT_LT(simple_result.rmse, voxel_size_ / 2.0);
  EXPECT_LT(merged_result.rmse, voxel_size_ / 2.0);
  EXPECT_LT(fast_result.rmse, voxel_size_ / 2.0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  FLAGS_logtostderr = true;
  FLAGS_alsologtostderr = true;
  FLAGS_v = 1;

  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  voxblox::timing::Timing::Print(std::cout);

  return result;
}

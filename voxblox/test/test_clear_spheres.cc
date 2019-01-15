#include <gtest/gtest.h>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/esdf_integrator.h"
#include "voxblox/integrator/tsdf_integrator.h"
#include "voxblox/io/layer_io.h"
#include "voxblox/simulation/simulation_world.h"
#include "voxblox/utils/evaluation_utils.h"
#include "voxblox/utils/layer_utils.h"

using namespace voxblox;  // NOLINT

DECLARE_bool(alsologtostderr);
DECLARE_bool(logtostderr);
DECLARE_int32(v);

class ClearSphereTest : public ::testing::TestWithParam<FloatingPoint> {
 public:
  ClearSphereTest()
      : depth_camera_resolution_(Eigen::Vector2i(320, 240)),
        fov_h_rad_(2.61799),  // 150 degrees
        max_dist_(15.0),
        min_dist_(0.5),
        voxel_size_(0.10),
        voxels_per_side_(16) {}

  virtual void SetUp() {
    voxel_size_ = GetParam();

    // Create a test environment.
    // It consists of a 10x10x7 m environment with an object in the middle and
    // walls on the side.
    Point min_bound(-5.0, -5.0, -1.0);
    Point max_bound(5.0, 5.0, 6.0);
    world_.setBounds(min_bound, max_bound);
    world_.addPlaneBoundaries(-4.0, 4.0, -4.0, 4.0);
    world_.addGroundLevel(0.0);

    // Next, generate some poses.
    FloatingPoint radius = 1.0;
    FloatingPoint height = 2.0;
    int num_poses = 10;
    poses_.reserve(num_poses);

    FloatingPoint max_angle = 2 * M_PI;

    FloatingPoint angle_increment = max_angle / num_poses;
    for (FloatingPoint angle = 0.0; angle < max_angle;
         angle += angle_increment) {
      // Generate a transformation to look at the center pose.
      Point position(radius * sin(angle), radius * cos(angle), height);
      Point facing_direction(sin(angle), cos(angle), 0.0);

      FloatingPoint desired_yaw = 0.0;
      if (std::abs(facing_direction.x()) > 1e-4 ||
          std::abs(facing_direction.y()) > 1e-4) {
        desired_yaw = atan2(facing_direction.y(), facing_direction.x());
      }

      // Face the desired yaw and pitch forward a bit to get some of the floor.
      Quaternion rotation =
          Quaternion(Eigen::AngleAxis<FloatingPoint>(-0.1, Point::UnitY())) *
          Eigen::AngleAxis<FloatingPoint>(desired_yaw, Point::UnitZ());

      poses_.emplace_back(Transformation(rotation, position));
    }

    // Finally, get the GT.
    tsdf_gt_.reset(new Layer<TsdfVoxel>(voxel_size_, voxels_per_side_));
    esdf_gt_.reset(new Layer<EsdfVoxel>(voxel_size_, voxels_per_side_));

    truncation_distance_ = 4 * voxel_size_;
    esdf_max_distance_ = 4.0f;

    LOG(INFO) << "Truncation distance: " << truncation_distance_
              << " ESDF max distance: " << esdf_max_distance_;

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

TEST_P(ClearSphereTest, EsdfIntegrators) {
  // TSDF layer + integrator
  TsdfIntegratorBase::Config config;
  config.default_truncation_distance = truncation_distance_;
  config.integrator_threads = 1;
  Layer<TsdfVoxel> tsdf_layer(voxel_size_, voxels_per_side_);
  MergedTsdfIntegrator tsdf_integrator(config, &tsdf_layer);

  // ESDF layers
  Layer<EsdfVoxel> esdf_layer(voxel_size_, voxels_per_side_);

  EsdfIntegrator::Config esdf_config;
  esdf_config.max_distance_m = esdf_max_distance_;
  esdf_config.default_distance_m = esdf_max_distance_;
  esdf_config.min_distance_m = truncation_distance_ / 2.0;
  esdf_config.min_diff_m = 0.0;
  esdf_config.full_euclidean_distance = false;
  esdf_config.add_occupied_crust = false;
  esdf_config.multi_queue = false;
  esdf_config.clear_sphere_radius = 1.0;
  esdf_config.occupied_sphere_radius = 4.0;
  EsdfIntegrator esdf_integrator(esdf_config, &tsdf_layer, &esdf_layer);

  size_t i = 0;
  Pointcloud ptcloud, ptcloud_C;
  Colors colors;
  constexpr bool clear_updated_flag = true;

  // Ok for the first pose... Let's do this one at a time.
  esdf_integrator.addNewRobotPosition(poses_[i].getPosition());

  // Output for debugging.
  io::SaveLayer(tsdf_layer, "esdf_clear1a.voxblox", true);
  io::SaveLayer(esdf_layer, "esdf_clear1a.voxblox", false);

  world_.getPointcloudFromTransform(poses_[i], depth_camera_resolution_,
                                    fov_h_rad_, max_dist_, &ptcloud, &colors);
  transformPointcloud(poses_[i].inverse(), ptcloud, &ptcloud_C);
  tsdf_integrator.integratePointCloud(poses_[i], ptcloud_C, colors);

  // Update the ESDF integrator.
  esdf_integrator.updateFromTsdfLayer(clear_updated_flag);

  io::SaveLayer(tsdf_layer, "esdf_clear1b.voxblox", true);
  io::SaveLayer(esdf_layer, "esdf_clear1b.voxblox", false);
  i = 2;

  esdf_integrator.addNewRobotPosition(poses_[i].getPosition());

  io::SaveLayer(tsdf_layer, "esdf_clear2a.voxblox", true);
  io::SaveLayer(esdf_layer, "esdf_clear2a.voxblox", false);

  ptcloud.clear();
  colors.clear();
  ptcloud_C.clear();
  world_.getPointcloudFromTransform(poses_[i], depth_camera_resolution_,
                                    fov_h_rad_, max_dist_, &ptcloud, &colors);
  transformPointcloud(poses_[i].inverse(), ptcloud, &ptcloud_C);
  tsdf_integrator.integratePointCloud(poses_[i], ptcloud_C, colors);
  esdf_integrator.updateFromTsdfLayer(clear_updated_flag);

  io::SaveLayer(tsdf_layer, "esdf_clear2b.voxblox", true);
  io::SaveLayer(esdf_layer, "esdf_clear2b.voxblox", false);

  // Main test: compare every voxel in ESDF and TSDF. Those within truncation
  // distance should match TSDF and not be hallucinated.
  BlockIndexList block_list;
  tsdf_layer.getAllAllocatedBlocks(&block_list);
  size_t vps = tsdf_layer.voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  for (const BlockIndex& block_index : block_list) {
    const Block<TsdfVoxel>& tsdf_block =
        tsdf_layer.getBlockByIndex(block_index);
    ASSERT_TRUE(esdf_layer.hasBlock(block_index));
    const Block<EsdfVoxel>& esdf_block =
        esdf_layer.getBlockByIndex(block_index);

    for (size_t linear_index = 0u; linear_index < num_voxels_per_block;
         ++linear_index) {
      const TsdfVoxel& tsdf_voxel =
          tsdf_block.getVoxelByLinearIndex(linear_index);
      const EsdfVoxel& esdf_voxel =
          esdf_block.getVoxelByLinearIndex(linear_index);

      if (tsdf_voxel.weight < 1e-6 && esdf_voxel.observed) {
        EXPECT_TRUE(esdf_voxel.hallucinated);
      }
      if (tsdf_voxel.weight > 1e-6 &&
          std::abs(tsdf_voxel.distance) <= esdf_config.min_distance_m) {
        EXPECT_TRUE(esdf_voxel.observed);
        EXPECT_FALSE(esdf_voxel.hallucinated);
        EXPECT_EQ(signum(tsdf_voxel.distance), signum(esdf_voxel.distance));
        EXPECT_NEAR(tsdf_voxel.distance, esdf_voxel.distance, 1e-3);
      }
    }
  }

  io::SaveLayer(*tsdf_gt_, "esdf_clear_gt.voxblox", true);
  io::SaveLayer(*esdf_gt_, "esdf_clear_gt.voxblox", false);
}

INSTANTIATE_TEST_CASE_P(VoxelSizes, ClearSphereTest,
                        ::testing::Values(0.1f, 0.2f));

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

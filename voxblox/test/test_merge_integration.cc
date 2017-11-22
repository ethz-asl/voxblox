#include <gtest/gtest.h>

#include "./Block.pb.h"
#include "./Layer.pb.h"
#include "voxblox/core/block.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/merge_integration.h"
#include "voxblox/io/layer_io.h"
#include "voxblox/io/mesh_ply.h"
#include "voxblox/io/sdf_ply.h"
#include "voxblox/simulation/simulation_world.h"
#include "voxblox/test/layer_test_utils.h"
#include "voxblox/utils/evaluation_utils.h"
#include "voxblox/utils/layer_utils.h"

using namespace voxblox;  // NOLINT

DECLARE_bool(alsologtostderr);
DECLARE_bool(logtostderr);
DECLARE_int32(v);

static const std::string kFolderPrefix = "test_results/";

template <typename VoxelType>
class LayerMergeToolTest : public ::testing::Test,
                           public voxblox::test::LayerTest<VoxelType> {
 protected:
  virtual void SetUp() {}

  void initializeSimulatedWorlds() {
    world_A_.reset(new Layer<VoxelType>(voxel_size_, voxels_per_side_));
    world_B_.reset(new Layer<VoxelType>(voxel_size_, voxels_per_side_));
    world_B_compare_.reset(new Layer<VoxelType>(voxel_size_, voxels_per_side_));

    SimulationWorld simulation;
    Point lower_bound, upper_bound;
    lower_bound << -0.5, -0.5, -0.5;
    upper_bound = -lower_bound;
    simulation.setBounds(lower_bound, upper_bound);

    constexpr FloatingPoint kTranslation = 0.15;
    // 20 degrees
    constexpr FloatingPoint kRotationAngle = 0.5;

    T_A_B_.setRandom(kTranslation, kRotationAngle);

    // Define sphere 1.
    Point c1_A;
    c1_A << 0.1, 0., 0.;
    constexpr FloatingPoint sphere_1_radius_m = 0.05;

    // Define sphere 2.
    Point c2_A;
    c2_A << -0.2, 0, 0.2;
    constexpr FloatingPoint sphere_2_radius_m = 0.03;

    // Define sphere 3.
    Point c3_A;
    c3_A << 0., 0.1, -0.2;
    constexpr FloatingPoint sphere_3_radius_m = 0.075;

    // Prepare world A
    simulation.addObject(std::unique_ptr<Object>(
        new Sphere(c1_A, sphere_1_radius_m, Color(255u, 0u, 0u))));
    simulation.addObject(std::unique_ptr<Object>(
        new Sphere(c2_A, sphere_2_radius_m, Color(255u, 0u, 0u))));
    simulation.addObject(std::unique_ptr<Object>(
        new Sphere(c3_A, sphere_3_radius_m, Color(255u, 0u, 0u))));
    simulation.generateSdfFromWorld(max_distance_world, world_A_.get());
    simulation.clear();

    // Prepare world B
    simulation.addObject(std::unique_ptr<Object>(new Sphere(
        T_A_B_.transform(c1_A), sphere_1_radius_m, Color(0u, 255u, 0u))));
    simulation.addObject(std::unique_ptr<Object>(new Sphere(
        T_A_B_.transform(c2_A), sphere_2_radius_m, Color(0u, 255u, 0u))));
    simulation.addObject(std::unique_ptr<Object>(new Sphere(
        T_A_B_.transform(c3_A), sphere_3_radius_m, Color(0u, 255u, 0u))));
    simulation.generateSdfFromWorld(max_distance_world, world_B_.get());
    simulation.generateSdfFromWorld(max_distance_world, world_B_compare_.get());
  }

  Transformation T_A_B_;
  typename Layer<VoxelType>::Ptr world_A_;
  typename Layer<VoxelType>::Ptr world_B_;
  typename Layer<VoxelType>::Ptr world_B_compare_;

  const double voxel_size_ = 0.005;
  const size_t voxels_per_side_ = 16u;
  const double max_distance_world = 2.0;
};

typedef LayerMergeToolTest<TsdfVoxel> TsdfLayerMergeToolTest;
typedef LayerMergeToolTest<EsdfVoxel> EsdfLayerMergeToolTest;

TEST_F(TsdfLayerMergeToolTest, MergeTwoTsdfLayers) {
  initializeSimulatedWorlds();

  const std::string kWorldAPlyFile = kFolderPrefix + "world_A.tsdf.ply";
  LOG(INFO) << "Writing world A to : " << kWorldAPlyFile;
  io::outputLayerAsPly(*world_A_, kWorldAPlyFile,
                       io::PlyOutputTypes::kSdfIsosurface);

  const std::string kWorldBPlyFile = kFolderPrefix + "world_B.tsdf.ply";
  LOG(INFO) << "Writing world B to : " << kWorldBPlyFile;
  io::outputLayerAsPly(*world_B_, kWorldBPlyFile,
                       io::PlyOutputTypes::kSdfIsosurface);

  LOG(INFO) << "Merging world A into world B...";
  constexpr bool kUseNaiveMethod = false;
  mergeLayerAintoLayerB(*world_A_, T_A_B_, world_B_.get(), kUseNaiveMethod);

  LOG(INFO) << "Evaluating merged world...";
  utils::VoxelEvaluationDetails result;
  utils::evaluateLayersRmse(*world_B_compare_, *world_B_,
                            utils::VoxelEvaluationMode::kEvaluateAllVoxels,
                            &result);
  constexpr FloatingPoint kFloatingPointTolerance = 5e-5;
  EXPECT_NEAR(result.rmse, 6e-5, kFloatingPointTolerance);
  EXPECT_NEAR(result.min_error, 0.0, kFloatingPointTolerance);
  EXPECT_NEAR(result.max_error, 0.002089, kFloatingPointTolerance);
  constexpr size_t kUintTolerance = 10;
  EXPECT_NEAR(result.num_evaluated_voxels, 7529536, kUintTolerance);
  EXPECT_NEAR(result.num_ignored_voxels, 0, kUintTolerance);
  EXPECT_NEAR(result.num_overlapping_voxels, 7529536, kUintTolerance);
  EXPECT_NEAR(result.num_non_overlapping_voxels, 3074567, kUintTolerance);

  const std::string kMergedLayerPlyFile =
      kFolderPrefix + "world_A_and_B.tsdf.ply";
  LOG(INFO) << "Writing merged world to : " << kMergedLayerPlyFile;
  io::outputLayerAsPly(*world_B_, kMergedLayerPlyFile,
                       io::PlyOutputTypes::kSdfIsosurface);
  LOG(INFO) << "Done.";
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  FLAGS_logtostderr = true;
  FLAGS_alsologtostderr = true;
  FLAGS_v = 1;

  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}

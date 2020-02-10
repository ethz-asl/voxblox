#include <gtest/gtest.h>

#include "voxblox/Block.pb.h"
#include "voxblox/Layer.pb.h"
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

    // Transformation generated at random by minkindr
    // Translation 0.15m, Rotation: 28.6479 degrees
    // clang-format off
    Eigen::Matrix<FloatingPoint, 4, 4> T_B_A;
    T_B_A <<  0.959292,  0.233991, 0.158138, -0.0627464,
             -0.277701,  0.883428, 0.377407,  0.126472,
             -0.051394, -0.405959, 0.912445, -0.0506717,
              0.      ,  0.      , 0.      ,  1.;
    // clang-format on
    T_B_A_ = Transformation(T_B_A);

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
        T_B_A_.transform(c1_A), sphere_1_radius_m, Color(0u, 255u, 0u))));
    simulation.addObject(std::unique_ptr<Object>(new Sphere(
        T_B_A_.transform(c2_A), sphere_2_radius_m, Color(0u, 255u, 0u))));
    simulation.addObject(std::unique_ptr<Object>(new Sphere(
        T_B_A_.transform(c3_A), sphere_3_radius_m, Color(0u, 255u, 0u))));
    simulation.generateSdfFromWorld(max_distance_world, world_B_.get());
    simulation.generateSdfFromWorld(max_distance_world, world_B_compare_.get());
  }

  Transformation T_B_A_;
  typename Layer<VoxelType>::Ptr world_A_;
  typename Layer<VoxelType>::Ptr world_B_;
  typename Layer<VoxelType>::Ptr world_B_compare_;

  const double voxel_size_ = 0.005;
  const size_t voxels_per_side_ = 16u;
  const double max_distance_world = 2.0;
  const double max_sdf_value_to_visualize = 0.015;
};

typedef LayerMergeToolTest<TsdfVoxel> TsdfLayerMergeToolTest;

// TODO(mfehr): Not yet supported by the layer merging function.
// typedef LayerMergeToolTest<EsdfVoxel> EsdfLayerMergeToolTest;

TEST_F(TsdfLayerMergeToolTest, MergeTwoTsdfLayers) {
  initializeSimulatedWorlds();

  const std::string kWorldAPlyFile = kFolderPrefix + "world_A.tsdf.ply";
  LOG(INFO) << "Writing world A to : " << kWorldAPlyFile;
  io::outputLayerAsPly(*world_A_, kWorldAPlyFile,
                       io::PlyOutputTypes::kSdfIsosurfaceConnected);
  const std::string kWorldASdfPlyFile = kFolderPrefix + "world_A.tsdf_sdf.ply";
  io::outputLayerAsPly(*world_A_, kWorldASdfPlyFile,
                       io::PlyOutputTypes::kSdfColoredDistanceField,
                       max_sdf_value_to_visualize, max_sdf_value_to_visualize);

  const std::string kWorldBPlyFile = kFolderPrefix + "world_B.tsdf.ply";
  LOG(INFO) << "Writing world B to : " << kWorldBPlyFile;
  io::outputLayerAsPly(*world_B_, kWorldBPlyFile,
                       io::PlyOutputTypes::kSdfIsosurfaceConnected);
  const std::string kWorldBSdfPlyFile = kFolderPrefix + "world_B.tsdf_sdf.ply";
  io::outputLayerAsPly(*world_B_, kWorldBSdfPlyFile,
                       io::PlyOutputTypes::kSdfColoredDistanceField,
                       max_sdf_value_to_visualize, max_sdf_value_to_visualize);

  LOG(INFO) << "Merging world A into world B...";
  constexpr bool kUseNaiveMethod = false;
  mergeLayerAintoLayerB(*world_A_, T_B_A_, world_B_.get(), kUseNaiveMethod);

  LOG(INFO) << "Evaluating merged world...";
  utils::VoxelEvaluationDetails result;
  utils::evaluateLayersRmse(*world_B_compare_, *world_B_,
                            utils::VoxelEvaluationMode::kEvaluateAllVoxels,
                            &result);
  constexpr FloatingPoint kFloatingPointToleranceLow = 5e-5;
  EXPECT_NEAR(result.rmse, 6e-5, kFloatingPointToleranceLow);

  constexpr FloatingPoint kFloatingPointToleranceHigh = 1e-4;
  EXPECT_NEAR(result.min_error, 0.0, kFloatingPointToleranceHigh);
  EXPECT_NEAR(result.max_error, 0.002, kFloatingPointToleranceHigh);

  constexpr size_t kUintTolerance = 10u;
  // The overlap between B and A+B is obviously exactly as big as B itself.
  EXPECT_NEAR(result.num_evaluated_voxels, 8000000, kUintTolerance);
  EXPECT_NEAR(result.num_overlapping_voxels, 8000000, kUintTolerance);
  EXPECT_NEAR(result.num_ignored_voxels, 0, kUintTolerance);
  // This results from the simulated world of A being transformed into b, i.e.
  // this is A_transformed - B.
  EXPECT_NEAR(result.num_non_overlapping_voxels, 4278466, kUintTolerance);

  const std::string kMergedLayerPlyFile =
      kFolderPrefix + "world_A_and_B.tsdf.ply";
  LOG(INFO) << "Writing merged world to : " << kMergedLayerPlyFile;
  io::outputLayerAsPly(*world_B_, kMergedLayerPlyFile,
                       io::PlyOutputTypes::kSdfIsosurfaceConnected);
  const std::string kMergedLayerSdfPlyFile =
      kFolderPrefix + "world_A_and_B.tsdf_sdf.ply";
  io::outputLayerAsPly(*world_B_, kMergedLayerSdfPlyFile,
                       io::PlyOutputTypes::kSdfColoredDistanceField,
                       max_sdf_value_to_visualize, max_sdf_value_to_visualize);
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

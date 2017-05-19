#include <random>

#include <eigen-checks/entrypoint.h>
#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>

#include "../../htwfnc_benchmarks/include/voxblox/mesh/mesh_layer.h"
#include "../../htwfsc_benchmarks/include/voxblox/core/tsdf_map.h"
#include "../../htwfsc_benchmarks/include/voxblox/integrator/tsdf_integrator.h"
#include "../../htwfsc_benchmarks/include/voxblox/integrator/tsdf_integrator_fast.h"
#include "../../htwfsc_benchmarks/include/voxblox/io/mesh_ply.h"
#include "../../htwfsc_benchmarks/include/voxblox/mesh/mesh_integrator.h"
#include "../../htwfsc_benchmarks/include/voxblox/simulation/sphere_simulator.h"
#include "../../htwfsc_benchmarks/include/voxblox/test/layer_test_utils.h"

using namespace voxblox;  // NOLINT

static constexpr size_t kSeed = 242u;

class FastE2ETest : public ::testing::Test {
 public:
  // Test data params.
  static constexpr double kMean = 0;
  static constexpr double kSigma = 0.01;
  static constexpr int kNumPoints = 1000;
  static constexpr double kRadius = 0.5;
  static constexpr size_t kNumDifferentSpheres = 10u;

  static constexpr double kVoxelSize = 0.01;
  static constexpr size_t kVoxelsPerSide = 16u;

 protected:
  virtual void SetUp() {
    std::default_random_engine gen(kSeed);
    std::normal_distribution<double> translation_norm_dist(0.0, 0.5);
    std::normal_distribution<double> angle_dist(0.0,
                                                2.0 * 3.141592653589793238463);

    baseline_layer_.reset(new Layer<TsdfVoxel>(kVoxelSize, kVoxelsPerSide));
    fast_layer_.reset(new Layer<TsdfVoxel>(kVoxelSize, kVoxelsPerSide));
    baseline_integrator_.reset(
        new TsdfIntegrator(config_, baseline_layer_.get()));
    fast_integrator_.reset(
        new fast::TsdfIntegrator(fast_config_, fast_layer_.get()));

    T_G_C_vector_.clear();
    colors_vector_.clear();
    sphere_points_C_vector_.clear();

    T_G_C_vector_.resize(kNumDifferentSpheres);
    colors_vector_.resize(kNumDifferentSpheres);
    sphere_points_C_vector_.resize(kNumDifferentSpheres);

    for (size_t sphere_idx = 0u; sphere_idx < kNumDifferentSpheres;
         ++sphere_idx) {
      sphere_sim::createSphere(kMean, kSigma, kRadius, kNumPoints,
                               &(sphere_points_C_vector_[sphere_idx]));

      colors_vector_[sphere_idx].resize(
          sphere_points_C_vector_[sphere_idx].size(), Color(128, 255, 0));

      T_G_C_vector_[sphere_idx].setRandom(translation_norm_dist(gen),
                                          angle_dist(gen));
    }

    MeshIntegrator::Config config;
    baseline_mesh_layer_.reset(new MeshLayer(kVoxelSize * kVoxelsPerSide));
    baseline_mesh_integrator_.reset(new MeshIntegrator(
        config, baseline_layer_.get(), baseline_mesh_layer_.get()));

    fast_mesh_layer_.reset(new MeshLayer(kVoxelSize * kVoxelsPerSide));
    fast_mesh_integrator_.reset(
        new MeshIntegrator(config, fast_layer_.get(), fast_mesh_layer_.get()));
  }

  std::vector<Colors> colors_vector_;
  std::vector<Pointcloud> sphere_points_C_vector_;
  std::vector<Transformation> T_G_C_vector_;

  TsdfIntegrator::Config config_;
  fast::TsdfIntegrator::Config fast_config_;

  std::unique_ptr<TsdfIntegrator> baseline_integrator_;
  std::unique_ptr<fast::TsdfIntegrator> fast_integrator_;

  std::unique_ptr<Layer<TsdfVoxel>> baseline_layer_;
  std::unique_ptr<Layer<TsdfVoxel>> fast_layer_;

  test::LayerTest<TsdfVoxel> layer_test_;

  std::unique_ptr<MeshIntegrator> baseline_mesh_integrator_;
  std::unique_ptr<MeshLayer> baseline_mesh_layer_;
  std::unique_ptr<MeshIntegrator> fast_mesh_integrator_;
  std::unique_ptr<MeshLayer> fast_mesh_layer_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

TEST_F(FastE2ETest, CompareToBaseline) {
  for (size_t sphere_idx = 0u; sphere_idx < kNumDifferentSpheres;
       ++sphere_idx) {
    baseline_integrator_->integratePointCloud(
        T_G_C_vector_[sphere_idx], sphere_points_C_vector_[sphere_idx],
        colors_vector_[sphere_idx]);
    fast_integrator_->integratePointCloud(T_G_C_vector_[sphere_idx],
                                          sphere_points_C_vector_[sphere_idx],
                                          colors_vector_[sphere_idx]);
  }
  layer_test_.CompareLayers(*baseline_layer_, *fast_layer_);

  baseline_mesh_integrator_->generateWholeMesh();
  CHECK(outputMeshLayerAsPly("baseline.ply", *baseline_mesh_layer_));
  fast_mesh_integrator_->generateWholeMesh();
  CHECK(outputMeshLayerAsPly("fast.ply", *fast_mesh_layer_));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}

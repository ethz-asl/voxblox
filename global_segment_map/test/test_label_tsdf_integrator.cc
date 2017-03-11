#include <gtest/gtest.h>

#include <voxblox/io/layer_io.h>
#include <voxblox/io/mesh_ply.h>

#include "global_segment_map/label_tsdf_integrator.h"
#include "global_segment_map/label_tsdf_map.h"
#include "global_segment_map/label_tsdf_mesh_integrator.h"
#include "global_segment_map/test/layer_test_utils.h"

namespace voxblox {

class LabelTsdfIntegratorTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    LabelTsdfMap::Config map_config;
    map_config.voxel_size = 0.1f;
    map_config.voxels_per_side = 8u;
    map_.reset(new LabelTsdfMap(map_config));

    LabelTsdfIntegrator::Config integrator_config;
    integrator_.reset(new LabelTsdfIntegrator(
        integrator_config, map_->getTsdfLayerPtr(), map_->getLabelLayerPtr(),
        map_->getHighestLabelPtr()));

    //save_layers_ = true;
  }

  void visualizeTestResult(const std::string output_file) {
    MeshLayer mesh_layer(map_->block_size());
    MeshLabelIntegrator::Config mesh_config;
    MeshLabelIntegrator mesh_integrator(mesh_config,
                                        map_->getTsdfLayerPtr(),
                                        map_->getLabelLayerPtr(),
                                        &mesh_layer);

    mesh_integrator.generateWholeMesh();

    voxblox::outputMeshLayerAsPly(output_file, mesh_layer);
  }

  std::shared_ptr<LabelTsdfMap> map_;
  std::shared_ptr<LabelTsdfIntegrator> integrator_;

  voxblox::test::LayerTest<LabelVoxel> label_layer_test_;
  voxblox::test::LayerTest<TsdfVoxel> tsdf_layer_test_;

  bool save_layers_;
};

TEST_F(LabelTsdfIntegratorTest, IntegratePointCloud) {
  Point sensor_origin(0.0, 0.0, 0.0);
  Transformation transform(sensor_origin, Eigen::Quaterniond::Identity());

  Pointcloud frame_to_integrate;

  Colors colors_to_integrate;
  Labels labels_to_integrate;

  constexpr Label kFirstLabel = 1u;
  // Build a 2x2m regular grid pointcloud.
  for (double x = 0.0; x < 2.0; x = x + 0.05) {
    for (double z = 0.0; z < 2.0; z = z + 0.05) {
      frame_to_integrate.push_back(transform.inverse() * Point(x, 1.0, z));
      // Assign a dummy color to the pointcloud to integrate.
      colors_to_integrate.push_back(Color());
      // Assign the same label to the whole pointcloud to integrate.
      labels_to_integrate.push_back(kFirstLabel);
    }
  }

  integrator_->integratePointCloud(transform, frame_to_integrate,
                                   colors_to_integrate, labels_to_integrate);

  const std::string tsdf_file =
        "test_data/label_tsdf_integrator_test_1.tsdf.voxblox";
  const std::string label_file =
        "test_data/label_tsdf_integrator_test_1.label.voxblox";

  if (save_layers_) {
    // Store tsdf and label layers ground truth to file
    io::SaveLayer(map_->getTsdfLayer(), tsdf_file);
    io::SaveLayer(map_->getLabelLayer(), label_file);
  } else {
    // Read tsdf and label layers ground truth from file
    Layer<TsdfVoxel>::Ptr tsdf_layer_from_file;
    io::LoadLayer<TsdfVoxel>(tsdf_file, &tsdf_layer_from_file);

    tsdf_layer_test_.CompareLayers(map_->getTsdfLayer(),
                                   *tsdf_layer_from_file);

    Layer<LabelVoxel>::Ptr label_layer_from_file;
    io::LoadLayer<LabelVoxel>(label_file, &label_layer_from_file);

    label_layer_test_.CompareLayers(map_->getLabelLayer(),
                                    *label_layer_from_file);

    #ifdef VISUALIZE_UNIT_TEST_RESULTS
      visualizeTestResult("labeltsdf_integrator_test_mesh_1.ply");
    #endif
  }
}

TEST_F(LabelTsdfIntegratorTest, ReadLabelPointCloud) {
  Point sensor_origin(0.0, 0.0, 0.0);
  Transformation transform(sensor_origin, Eigen::Quaterniond::Identity());

  Pointcloud frame_to_integrate;
  Pointcloud frame_to_compute_labels;

  Colors colors_to_integrate;
  Labels labels_to_integrate;

  constexpr Label kFirstLabel = 1u;
  // Build two 2x2m regular grid pointclouds.
  for (double x = 0.0; x < 2.0; x = x + 0.05) {
    for (double z = 0.0; z < 2.0; z = z + 0.05) {
      frame_to_integrate.push_back(transform.inverse() * Point(x, 1.0, z));
      frame_to_compute_labels.push_back(transform.inverse() * Point(x, 1.0, z));

      // Assign a dummy color to the pointcloud to integrate.
      colors_to_integrate.push_back(Color());
      // Assign the same label to the whole pointcloud to integrate.
      labels_to_integrate.push_back(kFirstLabel);
    }
  }

  integrator_->integratePointCloud(transform, frame_to_integrate,
                                   colors_to_integrate, labels_to_integrate);

  Labels computed_labels;
  integrator_->computePointCloudLabel(transform, frame_to_compute_labels,
                                      &computed_labels);

  // The computed labels match exactly the ones integrated
  EXPECT_TRUE(std::equal(computed_labels.begin(), computed_labels.end(),
                         labels_to_integrate.begin()));

  #ifdef VISUALIZE_UNIT_TEST_RESULTS
    visualizeTestResult("labeltsdf_integrator_test_mesh_2.ply");
  #endif
}

TEST_F(LabelTsdfIntegratorTest, ComputeDominantLabelPointCloud) {
  Point sensor_origin(0.0, 0.0, 0.0);
  Transformation transform(sensor_origin, Eigen::Quaterniond::Identity());

  Pointcloud frame_to_integrate;
  Pointcloud frame_to_compute_labels;

  Colors colors_to_integrate;
  Labels labels_to_integrate;

  constexpr Label kFirstLabel = 1u;
  constexpr Label kSecondLabel = 2u;

  // Build two 2x2m regular grid pointclouds.
  for (double x = 0.0; x < 2.0; x = x + 0.05) {
    for (double z = 0.0; z < 2.0; z = z + 0.05) {
      frame_to_integrate.push_back(transform.inverse() * Point(x, 1.0, z));
      frame_to_compute_labels.push_back(transform.inverse() * Point(x, 1.0, z));

      // Assign a dummy color to the pointcloud to integrate.
      colors_to_integrate.push_back(Color());
      // Assign two different labels to different.
      // parts of the pointcloud to integrate.
      if (x <= 1.5) {
        labels_to_integrate.push_back(kFirstLabel);
      } else {
        labels_to_integrate.push_back(kSecondLabel);
      }

    }
  }

  integrator_->integratePointCloud(transform, frame_to_integrate,
                                   colors_to_integrate, labels_to_integrate);

  Labels computed_labels;
  integrator_->computePointCloudLabel(transform, frame_to_compute_labels,
                                      &computed_labels);

  // The computed labels are all 1 since it's the dominant integrated label
  Labels expected_labels(computed_labels.size(), kFirstLabel);

  EXPECT_TRUE(std::equal(computed_labels.begin(), computed_labels.end(),
                         expected_labels.begin()));


  #ifdef VISUALIZE_UNIT_TEST_RESULTS
    visualizeTestResult("labeltsdf_integrator_test_mesh_3.ply");
  #endif
}

TEST_F(LabelTsdfIntegratorTest, ComputeUnseenLabelPointCloud) {
  Point sensor_origin(0.0, 0.0, 0.0);
  Transformation transform(sensor_origin, Eigen::Quaterniond::Identity());

  Pointcloud frame_to_integrate;
  Pointcloud frame_to_compute_labels;

  Colors colors_to_integrate;
  Labels labels_to_integrate;

  constexpr Label kFirstLabel = 1u;
  constexpr Label kUnseenLabel = 2u;

  // Build two 2x2m regular grid pointclouds.
  for (double x = 0.0; x < 2.0; x = x + 0.05) {
    for (double z = 0.0; z < 2.0; z = z + 0.05) {
      frame_to_integrate.push_back(transform.inverse() * Point(x, 1.0, z));
      // Read labels for a pointcloud for an unobserved area
      frame_to_compute_labels.push_back(transform.inverse() *
                                        Point(x + 2.5, 1.0, z));

      // Assign a dummy color to the pointcloud to integrate.
      colors_to_integrate.push_back(Color());
      // Assign the same label to the whole pointcloud to integrate.
      labels_to_integrate.push_back(kFirstLabel);
    }
  }

  integrator_->integratePointCloud(transform, frame_to_integrate,
                                   colors_to_integrate, labels_to_integrate);

  Labels computed_labels;
  integrator_->computePointCloudLabel(transform, frame_to_compute_labels,
                                      &computed_labels);

  // The computed labels are all the unseen label 2.
  Labels expected_labels(computed_labels.size(), kUnseenLabel);

  EXPECT_TRUE(std::equal(computed_labels.begin(), computed_labels.end(),
                         expected_labels.begin()));

  #ifdef VISUALIZE_UNIT_TEST_RESULTS
    visualizeTestResult("label_tsdf_integrator_test_mesh_4.ply");
  #endif
}
}  // namespace voxblox

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}

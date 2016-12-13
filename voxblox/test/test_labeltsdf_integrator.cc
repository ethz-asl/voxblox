#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "voxblox/core/labeltsdf_map.h"
#include "voxblox/integrator/labeltsdf_integrator.h"
#include "voxblox/io/mesh_ply.h"
#include "voxblox/mesh/mesh_label_integrator.h"

using namespace voxblox;

class LabelTsdfIntegratorTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    LabelTsdfMap::Config map_config;
    map_config.voxel_size = 0.1;
    map_config.voxels_per_side = 8;
    map_.reset(new LabelTsdfMap(map_config));

    LabelTsdfIntegrator::Config integrator_config;
    integrator_.reset(new LabelTsdfIntegrator(integrator_config,
                                              map_->getTsdfLayerPtr(),
                                              map_->getLabelLayerPtr(),
                                              map_->getHighestLabelPtr()));
  }
  std::shared_ptr<LabelTsdfMap> map_;
  std::shared_ptr<LabelTsdfIntegrator> integrator_;
};

TEST_F(LabelTsdfIntegratorTest, ReadLabelPointCloud) {
  Point sensor_origin(0, 0, 0);
  Transformation transform(sensor_origin, Eigen::Quaterniond::Identity());

  Pointcloud frame_to_integrate;
  Pointcloud frame_to_compute_labels;

  Colors colors_to_integrate;
  Labels labels_to_integrate;

  size_t label = 1u;
  // Build two 2x2m regular grid pointclouds.
  for (float x = 0; x < 2; x = x + 0.05) {
    for (float z = 0; z < 2; z = z + 0.05) {
      frame_to_integrate.push_back(transform.inverse() * Point(x, 1, z));
      frame_to_compute_labels.push_back(transform.inverse() * Point(x, 1, z));

      // Assign a dummy color to the pointcloud to integrate.
      colors_to_integrate.push_back(Color());
      // Assign the same label to the whole pointcloud to integrate.
      labels_to_integrate.push_back(label);
    }
  }

  integrator_->integratePointCloud(transform,
                                   frame_to_integrate,
                                   colors_to_integrate,
                                   labels_to_integrate);


  Labels computed_labels;
  integrator_->computePointCloudLabel(transform,
                                      frame_to_compute_labels,
                                      &computed_labels);

  // The computed labels match exactly the ones integrated
  EXPECT_THAT(labels_to_integrate, ::testing::ContainerEq(computed_labels));
}

TEST_F(LabelTsdfIntegratorTest, ComputeLabelPointCloud) {
  Point sensor_origin(0, 0, 0);
  Transformation transform(sensor_origin, Eigen::Quaterniond::Identity());

  Pointcloud frame_to_integrate;
  Pointcloud frame_to_compute_labels;

  Colors colors_to_integrate;
  Labels labels_to_integrate;

  size_t label = 1u;
  // Build two 2x2m regular grid pointclouds.
  for (float x = 0; x < 2; x = x + 0.05) {
    for (float z = 0; z < 2; z = z + 0.05) {
      frame_to_integrate.push_back(transform.inverse() * Point(x, 1, z));
      frame_to_compute_labels.push_back(transform.inverse() * Point(x, 1, z));

      // Assign a dummy color to the pointcloud to integrate.
      colors_to_integrate.push_back(Color());
      // Assign two different labels to different.
      // parts of the pointcloud to integrate.
      if (x > 1.5f) {
        label = 2u;
      }
      labels_to_integrate.push_back(label);
    }
  }

  integrator_->integratePointCloud(transform,
                                   frame_to_integrate,
                                   colors_to_integrate,
                                   labels_to_integrate);

  Labels computed_labels;
  integrator_->computePointCloudLabel(transform,
                                      frame_to_compute_labels,
                                      &computed_labels);

  // The computed labels are all 1 since it's the dominant integrated label
  EXPECT_THAT(computed_labels, ::testing::Each(1));

  std::shared_ptr<MeshLayer> mesh_layer_;
  std::shared_ptr<MeshLabelIntegrator> mesh_integrator_;

  // Generate the mesh.
  MeshLabelIntegrator::Config mesh_config;
  mesh_layer_.reset(new MeshLayer(map_->block_size()));
  mesh_integrator_.reset(new MeshLabelIntegrator(mesh_config,
                                                 map_->getTsdfLayerPtr(),
                                                 map_->getLabelLayerPtr(),
                                                 mesh_layer_.get()));
  mesh_integrator_->generateWholeMesh();

  voxblox::outputMeshLayerAsPly("test_tsdf.ply", mesh_layer_);
}

TEST_F(LabelTsdfIntegratorTest, ComputeNewLabelPointCloud) {
  Point sensor_origin(0, 0, 0);
  Transformation transform(sensor_origin, Eigen::Quaterniond::Identity());

  Pointcloud frame_to_integrate;
  Pointcloud frame_to_compute_labels;

  Colors colors_to_integrate;
  Labels labels_to_integrate;

  size_t label = 1u;
  // Build two 2x2m regular grid pointclouds.
  for (float x = 0; x < 2; x = x + 0.05) {
    for (float z = 0; z < 2; z = z + 0.05) {
      frame_to_integrate.push_back(
          transform.inverse() * Point(x, 1, z));
      // Read labels for a pointcloud for an unobserved area
      frame_to_compute_labels.push_back(
          transform.inverse() * Point(x + 2.5, 1, z));

      // Assign a dummy color to the pointcloud to integrate.
      colors_to_integrate.push_back(Color());
      // Assign the same label to the whole pointcloud to integrate.
      labels_to_integrate.push_back(label);
    }
  }

  integrator_->integratePointCloud(transform,
                                   frame_to_integrate,
                                   colors_to_integrate,
                                   labels_to_integrate);

  Labels computed_labels;
  integrator_->computePointCloudLabel(transform,
                                      frame_to_compute_labels,
                                      &computed_labels);

  // The computed labels are all the unseen label 2.
  EXPECT_THAT(computed_labels, ::testing::Each(2));
}


int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}

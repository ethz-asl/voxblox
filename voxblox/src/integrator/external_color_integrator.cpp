#include "voxblox/integrator/external_color_integrator.h"
#include "voxblox/utils/distance_utils.h"

namespace voxblox {
ExternalColorIntegrator::ExternalColorIntegrator(
    const Layer<TsdfVoxel> &tsdf_layer, Layer<ColorVoxel> *color_layer)
    : max_distance_(15.0f),
      max_weight_(100.0f),
      intensity_prop_voxel_radius_(2),
      tsdf_layer_(tsdf_layer),
      color_layer_(color_layer) {}

bool ExternalColorIntegrator::integrateColorBearingVectors(
    const kindr::minimal::QuatTransformation &T_G_S,
    const Pointcloud &bearing_vectors_camera_frame, const Colors &colors) {
  timing::Timer external_color_timer("external_color/integrate");

  Pointcloud bearing_vectors_world_frame(bearing_vectors_camera_frame.size());
  Point origin = T_G_S.getPosition().cast<FloatingPoint>();
  for (const Point &bearing_vector_camera_frame :
       bearing_vectors_camera_frame) {
    Point bearing_vector_world_frame =
        T_G_S.transform(bearing_vector_camera_frame.cast<double>())
            .cast<FloatingPoint>();
    bearing_vectors_world_frame.emplace_back(bearing_vector_world_frame);
  }
  CHECK_EQ(bearing_vectors_world_frame.size(), colors.size())
      << "Color and bearing vector sizes do not match!";
  const FloatingPoint voxel_size = tsdf_layer_.voxel_size();
  bool global_success = false;
  for (size_t i = 0; i < bearing_vectors_world_frame.size(); ++i) {
    Point surface_intersection = Point::Zero();
    // Cast ray from the origin in the direction of the bearing vector until
    // finding an intersection with a surface.
    bool success = getSurfaceDistanceAlongRay<TsdfVoxel>(
        tsdf_layer_, origin, bearing_vectors_world_frame[i], max_distance_,
        &surface_intersection);

    if (!success) {
      continue;
    }
    global_success = true;
    // Now look up the matching voxels in the intensity layer and mark them.
    // Let's just start with 1.
    Block<ColorVoxel>::Ptr block_ptr =
        color_layer_->allocateBlockPtrByCoordinates(surface_intersection);
    ColorVoxel &voxel = block_ptr->getVoxelByCoordinates(surface_intersection);
    voxel.color = Color::blendTwoColors(voxel.color, voxel.weight, colors[i],
                                        measurement_weight_);

    voxel.weight = std::min(max_weight_, voxel.weight + measurement_weight_);

    // Now check the surrounding voxels along the bearing vector. If they have
    // never been observed, then fill in their value. Otherwise don't.
    Point close_voxel = surface_intersection;
    for (int voxel_offset = -intensity_prop_voxel_radius_;
         voxel_offset <= intensity_prop_voxel_radius_; voxel_offset++) {
      close_voxel = surface_intersection +
                    bearing_vectors_world_frame[i] * voxel_offset * voxel_size;
      Block<ColorVoxel>::Ptr new_block_ptr =
          color_layer_->allocateBlockPtrByCoordinates(close_voxel);
      ColorVoxel &new_voxel = block_ptr->getVoxelByCoordinates(close_voxel);
      if (new_voxel.weight < 1e-6) {
        new_voxel.color = colors[i];
        new_voxel.weight += 1.0;
      }
    }
  }
  return global_success;
}

void ExternalColorIntegrator::recolorMeshLayer(voxblox::MeshLayer *mesh_layer) {
  CHECK_NOTNULL(mesh_layer);

  // Go over and recolor all the mesh blocks
  BlockIndexList mesh_indices;
  mesh_layer->getAllAllocatedMeshes(&mesh_indices);

  for (BlockIndex &mesh_index : mesh_indices) {
    Mesh::Ptr mesh = mesh_layer->getMeshPtrByIndex(mesh_index);

    // Look up vertices in the color layer
    for (size_t i = 0; i < mesh->vertices.size(); i++) {
      CHECK(mesh->hasColors())
          << "Make sure that colors are enabled for your mesh";

      // TODO(victorr): Use interpolation instead of nearest neighbor lookup
      const ColorVoxel *color_voxel =
          color_layer_->getVoxelPtrByCoordinates(mesh->vertices[i]);

      if (color_voxel != nullptr && color_voxel->weight > 0.0) {
        mesh->colors[i] = color_voxel->color;
      }
    }
  }
}
}  // namespace voxblox

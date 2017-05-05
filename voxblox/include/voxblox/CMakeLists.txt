add_custom_target(headers_voxblox SOURCES
  core/block.h
  core/block_hash.h
  core/block_inl.h
  core/color.h
  core/common.h
  core/esdf_map.h
  core/layer.h
  core/layer_inl.h
  core/occupancy_map.h
  core/tsdf_map.h
  core/voxel.h

  integrator/esdf_integrator.h
  integrator/esdf_occ_integrator.h
  integrator/integrator_utils.h
  integrator/merge_integrator.h
  integrator/occupancy_integrator.h
  integrator/tsdf_integrator.h

  interpolator/interpolator.h
  interpolator/interpolator_inl.h

  io/layer_io.h
  io/mesh_ply.h
  io/ply_writer.h
  io/sdf_ply.h

  mesh/marching_cubes.h
  mesh/mesh.h
  mesh/mesh_integrator.h
  mesh/mesh_layer.h

  simulation/objects.h
  simulation/simulation_world.h
  simulation/simulation_world_inl.h
  simulation/SphereSimulator.h

  test/layer_test_utils.h

  utils/bucket_queue.h
  utils/protobuf_utils.h
  utils/timing.h
)


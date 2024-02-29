cmake_minimum_required(VERSION 3.5)
project(voxblox)

set(CMAKE_CXX_STANDARD 14)
add_definitions(-Wall -Wextra)

set(AMENT_ENABLE_TESTING 0)
set(CMAKE_MACOSX_RPATH 0)

# Load necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(voxblox_proto REQUIRED)

ament_auto_find_build_dependencies()


####################
# HEADER FILES #
####################

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR} # Generate proto headers are here
  ${EIGEN3_INCLUDE_DIRS}
  ${VOXBLOX_PROTO_INCLUDE_DIRS}
)

####################
# SET SOURCE FILES #
####################

set(${PROJECT_NAME}_SRCS
  src/alignment/icp.cc
  src/core/block.cc
  src/core/esdf_map.cc
  src/core/tsdf_map.cc
  src/integrator/esdf_integrator.cc
  src/integrator/esdf_occ_integrator.cc
  src/integrator/integrator_utils.cc
  src/integrator/intensity_integrator.cc
  src/integrator/tsdf_integrator.cc
  src/io/mesh_ply.cc
  src/io/sdf_ply.cc
  src/mesh/marching_cubes.cc
  src/simulation/objects.cc
  src/simulation/simulation_world.cc
  src/utils/camera_model.cc
  src/utils/evaluation_utils.cc
  src/utils/layer_utils.cc
  src/utils/neighbor_tools.cc
  src/utils/protobuf_utils.cc
  src/utils/timing.cc
  src/utils/voxel_utils.cc
)

#############
# LIBRARIES #
#############

ament_auto_add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS}
)

target_link_libraries(${PROJECT_NAME} 
  ${voxblox_proto_LIBRARIES}
  glog::glog
)

############
# BINARIES #
############

ament_auto_add_executable(tsdf_to_esdf 
  test/tsdf_to_esdf.cc
)
target_link_libraries(tsdf_to_esdf ${PROJECT_NAME})

ament_auto_add_executable(test_load_esdf 
  test/test_load_esdf.cc
)
target_link_libraries(test_load_esdf ${PROJECT_NAME})

# TODO: Added tests

##########
# EXPORT #
##########

ament_auto_package()

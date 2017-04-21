cmake_minimum_required(VERSION 2.8.3)
project(voxblox)

find_package(catkin_simple REQUIRED)
catkin_simple()

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11 -Wno-sign-compare -Wno-unused-value)

############
# PROTOBUF #
############
# General idea: first check if we have protobuf catkin, then use that.
# Otherwise use system protobuf.
set(PROTO_DEFNS proto/voxblox/Block.proto
                proto/voxblox/Layer.proto)
set(ADDITIONAL_LIBRARIES "")

find_package(protobuf_catkin QUIET)
if (protobuf_catkin_FOUND)
    message(STATUS "Using protobuf_catkin")
    list(APPEND catkin_INCLUDE_DIRS ${protobuf_catkin_INCLUDE_DIRS})
    list(APPEND catkin_LIBRARIES ${protobuf_catkin_LIBRARIES})
    include_directories(${CMAKE_CURRENT_BINARY_DIR})

    PROTOBUF_CATKIN_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
    set(ADDITIONAL_LIBRARIES ${protobuf_catkin_LIBRARIES})
else()
  message(STATUS "Using system protobuf")
  find_package(Protobuf REQUIRED)
  include_directories(${PROTOBUF_INCLUDE_DIRS})
  include_directories(${CMAKE_CURRENT_BINARY_DIR})

  PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
  set(ADDITIONAL_LIBRARIES ${PROTOBUF_LIBRARY})
endif()


#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/core/block.cc
  src/core/esdf_map.cc
  src/integrator/esdf_integrator.cc
  src/integrator/esdf_occ_integrator.cc
  src/io/mesh_ply.cc
  src/mesh/marching_cubes.cc
  src/simulation/objects.cc
  src/simulation/simulation_world.cc
  src/utils/protobuf_utils.cc
  src/utils/timing.cc
  ${PROTO_SRCS}
)
target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES})

#########
# TESTS #
#########
add_custom_target(test_data)
add_custom_command(TARGET test_data
                  COMMAND rm -rf test_data
                  COMMAND mkdir -p test_data
                  COMMAND cp -r ${CMAKE_SOURCE_DIR}/test/test_data/*
                                test_data/ || :)

#add_definitions(-DVISUALIZE_UNIT_TEST_RESULTS)

catkin_add_gtest(test_tsdf_map
  test/test_tsdf_map.cc
)
target_link_libraries(test_tsdf_map ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(test_protobuf
  test/test_protobuf.cc
)
target_link_libraries(test_protobuf ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(test_tsdf_interpolator
  test/test_tsdf_interpolator.cc
)
target_link_libraries(test_tsdf_interpolator ${PROJECT_NAME} ${catkin_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export(INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR}
          CFG_EXTRAS voxblox-extras.cmake
          LIBRARIES ${ADDITIONAL_LIBRARIES})

cmake_minimum_required(VERSION 2.8.3)
project(voxblox)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11 -Wno-sign-compare -Wno-unused-value)

############
# PROTOBUF #
############
set(PROTO_DEFNS proto/voxblox/Block.proto
                proto/voxblox/Layer.proto)
PROTOBUF_CATKIN_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/core/block.cc
  src/core/esdf_map.cc
  src/io/mesh_ply.cc
  src/mesh/marching_cubes.cc
  src/utils/protobuf_utils.cc
  src/utils/timing.cc
  ${PROTO_SRCS}
)

############
# BINARIES #
############
cs_add_executable(voxblox_rox
  src/voxblox.cc
)
target_link_libraries(voxblox_rox ${PROJECT_NAME})


#########
# TESTS #
#########
catkin_add_gtest(test_tsdf_map
  test/test_tsdf_map.cc
)
target_link_libraries(test_tsdf_map ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(test_protobuf
  test/test_protobuf.cc
)
target_link_libraries(test_protobuf ${PROJECT_NAME} ${catkin_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export(INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR}
          CFG_EXTRAS voxblox-extras.cmake)

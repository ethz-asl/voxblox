cmake_minimum_required(VERSION 2.8.3)
project(voxblox)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

############
# PROTOBUF #
############

set(PROTO_DEFNS proto/Block.proto
                proto/Layer.proto)
PROTOBUF_CATKIN_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

#############
# LIBRARIES #
#############

# TODO(mfehr, helenol): Add a library as soon as we have source files.

############
# BINARIES #
############

cs_add_executable(voxblox_rox
  src/voxblox.cc
)



#########
# TESTS #
#########

catkin_add_gtest(test_tsdf_map
  test/test_tsdf_map.cpp
)
target_link_libraries(test_tsdf_map ${catkin_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()

cmake_minimum_required(VERSION 2.8.3)
project(voxblox_tango_interface)

find_package(catkin_simple REQUIRED)
catkin_simple()

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11 -Wno-sign-compare -Wno-unused-value)

############
# PROTOBUF #
############
# General idea: first check if we have protobuf catkin, then use that.
# Otherwise use system protobuf.
set(PROTO_DEFNS proto/tsdf2/MapHeader.proto
                proto/tsdf2/Volume.proto)
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

####################
# SET SOURCE FILES #
####################

set("${PROJECT_NAME}_SRCS"
  src/core/tango_block_interface.cc
)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS}
  ${PROTO_SRCS}
)
target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES} pthread)

############
# BINARIES #
############

add_executable(mesh_tango_tsdf
  test/mesh_tango_tsdf.cc
)
target_link_libraries(mesh_tango_tsdf ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(tango_tsdf_to_esdf
  test/tango_tsdf_to_esdf.cc
)
target_link_libraries(tango_tsdf_to_esdf ${PROJECT_NAME} ${catkin_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export(INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR}
          LIBRARIES ${ADDITIONAL_LIBRARIES})

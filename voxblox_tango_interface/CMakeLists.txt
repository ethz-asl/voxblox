cmake_minimum_required(VERSION 2.8.3)
project(voxblox_tango_interface)

find_package(catkin_simple REQUIRED)
catkin_simple()

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11 -Wno-sign-compare -Wno-unused-value)

if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/../tools/pybind11/CMakeLists.txt")
  set(HAVE_PYBIND11 TRUE)
  message(STATUS "Found pybind11; generating Python bindings")
endif()

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
# NOTE(mereweth@jpl.nasa.gov) - Be careful when compiling Proto cc files. It is
# best to do so only once (make a shared library for each set of Proto files.
# Otherwise, at some point, you will get errors from double-adding protobuf
# formats, of the following form:
# [libprotobuf ERROR google/protobuf/descriptor_database.cc:57]
# File already exists in database: Block.proto

# Avoid having multiple compiled copies of the same .pb.cc
# file sharing a single copy of libprotobuf.so
cs_add_library(${PROJECT_NAME}_proto
  ${PROTO_SRCS}
)
target_link_libraries(${PROJECT_NAME}_proto ${PROTOBUF_LIBRARIES})

cs_add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS}
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_proto ${PROTOBUF_LIBRARIES} pthread)

###################
# PYTHON BINDINGS #
###################

if(HAVE_PYBIND11)
  catkin_python_setup()

  add_subdirectory(../tools/pybind11 pybind11)
  message("Building Python bindings for voxblox")
  pybind11_add_module(voxblox_tango_interfacepy
    ${${PROJECT_NAME}_SRCS}
    src/pybind11/bind.cc
    ../voxblox/src/pybind11/layer_bind.cc
    src/pybind11/tango_layer_interface_bind.cc
    src/pybind11/tango_layer_io_bind.cc)
  set_target_properties(voxblox_tango_interfacepy PROPERTIES LINKER_LANGUAGE CXX)
  target_link_libraries(voxblox_tango_interfacepy PUBLIC ${PROJECT_NAME}_proto
                                                         ${PROTOBUF_LIBRARIES})

  set_target_properties(voxblox_tango_interfacepy
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/../
  )
endif()

############
# BINARIES #
############

add_executable(mesh_tango_tsdf
  test/mesh_tango_tsdf.cc
)
target_link_libraries(mesh_tango_tsdf ${PROJECT_NAME})

add_executable(tango_tsdf_resize_to_tsdf
  test/tango_tsdf_resize_to_tsdf.cc
)
target_link_libraries(tango_tsdf_resize_to_tsdf ${PROJECT_NAME})

add_executable(tango_tsdf_to_esdf
  test/tango_tsdf_to_esdf.cc
)
target_link_libraries(tango_tsdf_to_esdf ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export(INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR}
          LIBRARIES ${ADDITIONAL_LIBRARIES})

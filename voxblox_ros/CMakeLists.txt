cmake_minimum_required(VERSION 2.8.3)
project(voxblox_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -Wno-sign-compare -Wno-unused-value)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/transformer.cc
  src/tsdf_server.cc
  src/esdf_server.cc
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

############
# BINARIES #
############

cs_add_executable(voxblox_eval
  src/voxblox_eval.cc
)

cs_add_executable(voxblox_node
  src/voxblox_node.cc
)
add_dependencies(voxblox_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

##########
# EXPORT #
##########
cs_install()
cs_export()

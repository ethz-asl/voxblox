cmake_minimum_required(VERSION 2.8.3)
project(voxblox_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
#cs_add_library(${PROJECT_NAME}
#  src/voxblox.cc
#)

############
# BINARIES #
############
cs_add_executable(voxblox_node
  src/voxblox_node.cc
)
#target_link_libraries(voxblox_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()

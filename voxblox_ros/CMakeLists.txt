cmake_minimum_required(VERSION 2.8.3)
project(voxblox_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -Wno-sign-compare -Wno-unused-value)

#############
# LIBRARIES #
#############

# TODO(mfehr, helenol): Add a library as soon as we have source files.

############
# BINARIES #
############

cs_add_executable(voxblox_node
  src/voxblox_node.cc
)

##########
# EXPORT #
##########

cs_install()
cs_export()

cmake_minimum_required(VERSION 3.0.2)
project(voxblox_msgs)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

##########
# EXPORT #
##########

cs_install()
cs_export()

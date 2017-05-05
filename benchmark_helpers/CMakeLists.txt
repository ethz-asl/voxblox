cmake_minimum_required(VERSION 2.8.3)
project(benchmark_helpers)

find_package(catkin_simple REQUIRED)
catkin_simple()

cs_install()
cs_export(CFG_EXTRAS benchmark-helpers.cmake.em)

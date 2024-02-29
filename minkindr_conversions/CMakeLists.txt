cmake_minimum_required(VERSION 3.5)
project(minkindr_conversions)

add_definitions(-std=c++11)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_package()



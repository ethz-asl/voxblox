cmake_minimum_required(VERSION 3.5)
project(minkindr)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

find_package(Threads)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Add all files to show up in QtCreator.
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})

ament_auto_package()


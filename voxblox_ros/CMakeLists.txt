cmake_minimum_required(VERSION 3.5)
project(voxblox_ros)

set(CMAKE_CXX_STANDARD 14)
add_definitions(-Wall -Wextra)

set(AMENT_ENABLE_TESTING 0)
set(CMAKE_MACOSX_RPATH 0)

# Load necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_eigen_kdl REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(voxblox REQUIRED)
find_package(voxblox_proto REQUIRED)
find_package(voxblox_msgs REQUIRED)

find_package(tf2_geometry_msgs REQUIRED)
find_package(glog REQUIRED)

ament_auto_find_build_dependencies()

set(AMENT_DEPENDENCIES
    rclcpp

    std_srvs
    visualization_msgs
    sensor_msgs
    geometry_msgs
    tf2_geometry_msgs

    voxblox
    voxblox_proto
    voxblox_msgs

    tf2
    tf2_ros
    pcl_ros
    pcl_conversions
    tf2_eigen
    tf2_eigen_kdl

    glog
    Eigen3
)

set(OTHER_DEPENDENCIES
  glog::glog
  Eigen3::Eigen3
)

#############
# LIBRARIES #
#############

ament_auto_add_library(${PROJECT_NAME}
  # src/esdf_server.cc
  # src/interactive_slider.cc
  # src/intensity_server.cc
  src/transformer.cc
  src/tsdf_server.cc
)

ament_target_dependencies(${PROJECT_NAME} 
  ${AMENT_DEPENDENCIES}
)

target_link_libraries(${PROJECT_NAME} 
  ${OTHER_DEPENDENCIES}
)

############
# BINARIES #
############

# TODO: Add other executables and fix their ROS2 migration errors

# ament_auto_add_executable(voxblox_eval
#   src/voxblox_eval.cc
# )
# target_link_libraries(voxblox_eval ${PROJECT_NAME})

# TSDF_SERVER:
ament_auto_add_executable(tsdf_server
  src/tsdf_server_node.cc
)
target_link_libraries(tsdf_server 
  ${PROJECT_NAME}
)

# # ESDF_SERVER:
# ament_auto_add_executable(esdf_server
#   src/esdf_server_node.cc
# )
# target_link_libraries(esdf_server 
#   ${PROJECT_NAME}
# )

# ament_auto_add_executable(intensity_server
#   src/intensity_server_node.cc
# )
# target_link_libraries(intensity_server ${PROJECT_NAME})

# ament_auto_add_executable(visualize_tsdf
#   src/visualize_tsdf.cc
# )
# target_link_libraries(visualize_tsdf ${PROJECT_NAME})


##########
# EXPORT #
##########

ament_auto_package()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

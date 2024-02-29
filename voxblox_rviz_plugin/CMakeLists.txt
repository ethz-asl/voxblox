cmake_minimum_required(VERSION 3.5)
project(voxblox_rviz_plugin)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(voxblox REQUIRED)
find_package(voxblox_msgs REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(pluginlib REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MACOSX_RPATH 0)
set(OGRE_PLUGIN_PATH "rviz_ogre_vendor")

add_definitions(-Wall)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
find_package(Qt5 REQUIRED Core Widgets)
## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
set(QT_LIBRARIES Qt5::Widgets)
QT5_WRAP_CPP(MOC_FILES
  ${INCLUDE_FILES_QT}
)

## Avoid Qt signals and slots defining "emit", "slots", etc.
add_definitions(-DQT_NO_KEYWORDS)

set(HEADER_FILES 
  include/voxblox_rviz_plugin/voxblox_mesh_display.h 
  include/voxblox_rviz_plugin/voxblox_mesh_visual.h
)

set(SRC_FILES 
  src/voxblox_mesh_display.cc 
  src/voxblox_mesh_visual.cc
)

add_library(${PROJECT_NAME}
  SHARED
  ${SRC_FILES}
  ${HEADER_FILES}
  ${MOC_FILES}
)

ament_target_dependencies(${PROJECT_NAME} 
  voxblox
  voxblox_msgs
  rviz_common
  rviz_ogre_vendor
)

target_include_directories(${PROJECT_NAME}  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  rviz_common::rviz_common
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  ${voxblox_TARGETS}
  ${voxblox_msgs_TARGETS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "VOXBLOX_RVIZ_PLUGIN_BUILDING_LIBRARY")

##########
# EXPORT #
##########

ament_export_dependencies(
  pluginlib
  Qt5
  rclcpp
  rviz_ogre_vendor
  voxblox_msgs
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/icons
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/voxblox.rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()


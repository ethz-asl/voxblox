cmake_minimum_required(VERSION 2.8.3)
project(global_segment_map)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/dummy.cc
  src/label_block_serialization.cc
)

############
# BINARIES #
############


#########
# TESTS #
#########
add_custom_target(test_data)
add_custom_command(TARGET test_data
                  COMMAND rm -rf test_data
                  COMMAND mkdir -p test_data
                  COMMAND cp -r ${CMAKE_SOURCE_DIR}/test/test_data/*
                                test_data/ || :)

# set(VISUALIZE_UNIT_TEST_RESULTS)

catkin_add_gtest(test_label_tsdf_integrator
  test/test_label_tsdf_integrator.cc
)
target_link_libraries(test_label_tsdf_integrator
  ${PROJECT_NAME} ${catkin_LIBRARIES}
)
add_dependencies(test_label_tsdf_integrator test_data)

##########
# EXPORT #
##########
cs_install()
cs_export()

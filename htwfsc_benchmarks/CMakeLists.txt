cmake_minimum_required(VERSION 2.8.3)
project(htwfsc_benchmarks)

find_package(catkin_simple REQUIRED)
catkin_simple()

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11 -Wno-sign-compare -Wno-unused-value)

##############
# BENCHMARKS #
##############
add_benchmark(bm_cast_ray test/benchmark_cast_ray.cc)
target_link_libraries(bm_cast_ray ${PROJECT_NAME})

add_benchmark(bm_e2e test/benchmark_e2e.cc)
target_link_libraries(bm_e2e ${PROJECT_NAME})

#add_benchmark(bm_update_tsdf test/benchmark_update_tsdf.cc)
#target_link_libraries(bm_update_tsdf ${PROJECT_NAME})

# #########
# # TESTS #
# #########

#add_definitions(-DVISUALIZE_UNIT_TEST_RESULTS)

# Fast code tests.
catkin_add_gtest(test_fast_vs_baseline_e2e
  test/test_fast_vs_baseline_e2e.cc
)
target_link_libraries(test_fast_vs_baseline_e2e ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(test_fast_vs_baseline_cast_ray
  test/test_fast_vs_baseline_cast_ray.cc
)
target_link_libraries(test_fast_vs_baseline_cast_ray ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(test_fast_vs_baseline_update_tsdf
  test/test_fast_vs_baseline_update_tsdf.cc
)
target_link_libraries(test_fast_vs_baseline_update_tsdf ${PROJECT_NAME} ${catkin_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()

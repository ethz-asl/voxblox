function(add_benchmark target)
  if(NOT DEFINED CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    message(FATAL_ERROR "add_executable_with_benchmark() must be called after catkin_package() so that default output directories for the executables are defined")
  endif()

  # Add include directory if available.
  set(${PROJECT_NAME}_LOCAL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
  if(NOT IS_DIRECTORY ${${PROJECT_NAME}_LOCAL_INCLUDE_DIR})
    set(${PROJECT_NAME}_LOCAL_INCLUDE_DIR)
  endif()
  include_directories(${${PROJECT_NAME}_LOCAL_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
  
  # Build the test (but not in target all).
  cs_add_executable(${target} ${ARGN})
  set_target_properties(${target} PROPERTIES EXCLUDE_FROM_ALL TRUE)
  target_link_libraries(${target} ${catkin_LIBRARIES} ${THREADS_LIBRARY})
  
  # Add to benchmark meta-target.
  if(NOT TARGET benchmarks)
    add_custom_target(benchmarks)
  endif()
  add_dependencies(benchmarks ${target})
    
  # Create run_benchmark custom-target.
  if(TARGET ${target})
    get_target_property(_target_path ${target} RUNTIME_OUTPUT_DIRECTORY)
    
    set(OUTPUT_FILE "${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME}/benchmark-${target}.csv")
    set(cmd bash -c ${_target_path}/${target} --benchmark_out_format=csv --benchmark_out=${OUTPUT_FILE})
    
    add_custom_command(
    	OUTPUT ${OUTPUT_FILE}
    	COMMAND ${cmd} 
    	DEPENDS ${target} 
    	WORKING_DIRECTORY ${ARG_WORKING_DIRECTORY}
    )
    add_custom_target(run_benchmarks_${target} DEPENDS ${OUTPUT_FILE})
    
    # Add to benchmark meta-target.
  	if(NOT TARGET run_benchmarks)
		add_custom_target(run_benchmarks)
  	endif()
  	add_dependencies(run_benchmarks run_benchmarks_${target})
  endif()
endfunction()

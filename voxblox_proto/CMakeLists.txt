cmake_minimum_required(VERSION 3.5)
project(voxblox_proto)

set(CMAKE_CXX_STANDARD 14)
add_definitions(-Wall -Wextra)

# Load necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(Protobuf REQUIRED)

ament_auto_find_build_dependencies()

############
# PROTOBUF #
############

set(PROTOBUF_COMPILE_PYTHON true)

set(PROTO_FILES
    "proto/Block.proto"
    "proto/Layer.proto"
)

# Generate C++ headers from .proto files
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

ament_auto_add_library(${PROJECT_NAME}
  ${PROTO_SRCS} 
  ${PROTO_HDRS}
)

target_link_libraries(${PROJECT_NAME}
  ${Protobuf_LIBRARIES}
)

##########
# EXPORT #
##########

install(
    FILES ${PROTO_HDRS}
    DESTINATION include/${PROJECT_NAME}/
)

# Export the library and headers
ament_export_dependencies(ament_cmake)
ament_export_include_directories(include)
ament_export_libraries(
  ${PROJECT_NAME}
  ${Protobuf_LIBRARIES}
)

ament_auto_package()

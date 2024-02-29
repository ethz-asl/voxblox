cmake_minimum_required(VERSION 3.5)
project(voxblox_msgs)

set(CMAKE_CXX_STANDARD 14)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# message generation
rosidl_generate_interfaces(
    ${PROJECT_NAME}
        "msg/Block.msg"
        "msg/Layer.msg"
        "msg/Mesh.msg"
        "msg/MeshBlock.msg"
        "msg/VoxelEvaluationDetails.msg"
        "srv/FilePath.srv"
    DEPENDENCIES 
        std_msgs 
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()


import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    voxblox_rviz_plugin_share_directory = get_package_share_directory('voxblox_rviz_plugin')
    voxblox_ros_share_directory = get_package_share_directory("voxblox_ros")
    parameters_file_path = Path(voxblox_ros_share_directory) / "config" / "cow_and_lady.yaml"

    bag_file_arg = DeclareLaunchArgument(
        'bag_file', 
        default_value = "./data/cow_and_lady_dataset",
        description='Filepath to bag file to playback.'
    )
    
    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file', 
        default_value = f"{voxblox_rviz_plugin_share_directory}/voxblox.rviz",
        description='File path to rviz config file for voxblox.'
    )
    
    tsdf_server_node = Node(
        package='voxblox_ros',
        executable='tsdf_server',
        name='voxblox_node',
        output='screen',
        arguments=['-alsologtostderr'],
        parameters=[
            str(parameters_file_path),
            {'tsdf_voxel_size': 0.05},
            {'tsdf_voxels_per_side': 16},
            {'voxel_carving_enabled': True},
            {'color_mode': 'color'},
            {'use_tf_transforms': False},
            {'update_mesh_every_n_sec': 1.0},
            {'min_time_between_msgs_sec': 0.0},
            {'method': 'fast'},
            {'use_const_weight': False},
            {'allow_clear': True},
            {'verbose': True},
            {'mesh_filename': launch.substitutions.LaunchConfiguration('bag_file')},
        ],
        remappings=[
            ('pointcloud', '/camera/depth_registered/points'), 
            ('transform', '/kinect/vrpn_client/estimated_transform')
        ],
    )
        
    play_bag_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration("bag_file")],
        output='screen',
    )
    
    start_rviz_process = ExecuteProcess(
        cmd=['rviz2', '-d', LaunchConfiguration("rviz_config_file")],
        output='screen',
    )
    
    launch_list = [
        bag_file_arg,
        rviz_config_file_arg,

        tsdf_server_node,
        play_bag_process,
        start_rviz_process,
    ]
    
    return LaunchDescription(launch_list)


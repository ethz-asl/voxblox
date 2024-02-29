import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    voxblox_rviz_plugin_share_directory = get_package_share_directory('voxblox_rviz_plugin')

    bag_file_arg = DeclareLaunchArgument(
        'bag_file', 
        default_value = "./data/basement_dataset",
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
        # arguments=['-alsologtostderr', '--ros-args', '--log-level', 'debug'],
        remappings=[('pointcloud', '/velodyne_points')],
        parameters=[
            {'min_time_between_msgs_sec': 0.0},
            {'tsdf_voxel_size': 0.2},
            {'truncation_distance': 0.5},
            {'color_mode': 'normals'},
            {'enable_icp': True},
            {'icp_refine_roll_pitch': False},
            {'update_mesh_every_n_sec': 1.0},
            {'mesh_min_weight': 2.0},
            {'method': 'fast'},
            {'max_ray_length_m': 10.0},
            {'use_const_weight': True},
            {'world_frame': 'world'},
            {'verbose': True},
            {'mesh_filename': launch.substitutions.LaunchConfiguration('bag_file')}
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
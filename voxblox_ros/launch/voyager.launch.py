import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    lidarslam_share_directory = get_package_share_directory('lidarslam')
    voxblox_rviz_plugin_share_directory = get_package_share_directory('voxblox_rviz_plugin')

    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value="true",
        description='Whether to launch RViz or not.'
    )
    
    play_bag_arg = DeclareLaunchArgument(
        'play_bag',
        default_value="true",
        description='Whether to play ros bag file or not.'
    )
    
    bag_file_arg = DeclareLaunchArgument(
        'bag_file', 
        default_value = "/home/mlouw2/bags/lab_001_inside",
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
        remappings=[('/pointcloud', '/ouster/points')],
        parameters=[
            {'min_time_between_msgs_sec': 0.2},
            {'tsdf_voxel_size': 0.2},
            {'truncation_distance': 1.0},
            {'color_mode': 'normals'},
            {'enable_icp': False},
            {'icp_refine_roll_pitch': False},
            {'update_mesh_every_n_sec': 1.0},
            {'mesh_min_weight': 2.0},
            {'method': 'fast'},
            {'max_ray_length_m': 25.0},
            {'use_const_weight': True},
            {'world_frame': 'map'},
            {'sensor_frame': ''},
            {'verbose': True},
            {'mesh_filename': launch.substitutions.LaunchConfiguration('bag_file')}
        ],
    )
    
    lidarslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([lidarslam_share_directory, '/launch/lidarslam_voyager.launch.py'])
    )
    
    play_bag_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration("bag_file")],
        output='screen',
    )
    
    start_rviz_process = ExecuteProcess(
        cmd=['rviz2', '-d', LaunchConfiguration("rviz_config_file")],
        output='screen',
    )
    
    print(str(LaunchConfiguration('start_rviz')))
    
    launch_list = [
        start_rviz_arg,
        play_bag_arg,
        bag_file_arg,
        rviz_config_file_arg,

        tsdf_server_node,
        lidarslam_launch,
        play_bag_process,
        # start_rviz_process,
    ]
    
    if(str(LaunchConfiguration('play_bag')).lower() == "true"): # TODO: fix this.
        launch_list.append(play_bag_process)
    
    if(str(LaunchConfiguration('start_rviz')).lower() == "true"):
        launch_list.append(start_rviz_process)
    
    return LaunchDescription(launch_list)

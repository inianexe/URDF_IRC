"""SLAM only: robot_state_publisher, scan and odometry are supplied separately."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('rover_bringup'))
    clock = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
             parameters=[clock], output='screen',
             arguments=['-configuration_directory', str(share / 'config'),
                        '-configuration_basename', 'rover_2d.lua'],
             remappings=[('scan', '/scan'), ('odom', '/odom')]),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             name='cartographer_occupancy_grid_node', output='screen',
             parameters=[clock, {'resolution': 0.05, 'publish_period_sec': 1.0}]),
    ])

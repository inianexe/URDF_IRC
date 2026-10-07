"""SLAM + Nav2, or AMCL + Nav2 with a saved map. Requires robot/sim separately."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('rover_description'))
    nav = Path(get_package_share_directory('nav2_bringup'))
    slam = Path(get_package_share_directory('slam_toolbox'))
    common = {'use_sim_time': LaunchConfiguration('use_sim_time'),
              'params_file': str(share / 'config/nav2.yaml'),
              'autostart': 'true', 'use_composition': 'False'}
    return LaunchDescription([
        DeclareLaunchArgument('mapping', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=''),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(slam / 'launch/online_async_launch.py')),
            condition=IfCondition(LaunchConfiguration('mapping')),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'slam_params_file': str(share / 'config/slam.yaml')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav / 'launch/localization_launch.py')),
            condition=UnlessCondition(LaunchConfiguration('mapping')),
            launch_arguments={**common, 'map': LaunchConfiguration('map')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav / 'launch/navigation_launch.py')),
                                 launch_arguments=common.items()),
    ])

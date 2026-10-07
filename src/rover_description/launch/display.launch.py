"""Static inspection only. Do not publish GUI joint states on a moving robot."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('rover_description'))
    description = ParameterValue(Command(['xacro ', str(share / 'urdf/rover.urdf.xacro')]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share / 'rviz/rover.rviz')],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])

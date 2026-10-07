from pathlib import Path
import shlex
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, Shutdown
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start_gazebo(context):
    # Direct execution: a shell wrapper could exit on SIGINT while leaving the
    # actual Gazebo server alive. Launch now signals Gazebo itself.
    args=shlex.split(LaunchConfiguration('gz_args').perform(context))
    return [ExecuteProcess(cmd=['gz','sim',*args,'--force-version','8'],
                           output='screen',on_exit=Shutdown())]


def generate_launch_description():
    share = Path(get_package_share_directory('rover_description'))
    simulation = Path(get_package_share_directory('rover_simulation'))
    description = ParameterValue(Command(['xacro ', str(share / 'urdf/rover.urdf.xacro'),
        ' simulation:=true articulated_arm:=', LaunchConfiguration('articulated_arm'),
        ' suspension:=', LaunchConfiguration('suspension'),
        ' lidar_enabled:=', LaunchConfiguration('lidar_enabled')]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('articulated_arm', default_value='false'),
        DeclareLaunchArgument('suspension', default_value='true'),
        DeclareLaunchArgument('lidar_enabled', default_value='true'),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', [str(simulation.parent.parent / 'lib'), ':', EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', default_value='')]),
        DeclareLaunchArgument('spawn_x', default_value='0'),
        DeclareLaunchArgument('spawn_y', default_value='0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0'),
        DeclareLaunchArgument('bridge_config', default_value=str(share / 'config/bridge.yaml')),
        DeclareLaunchArgument('gz_args', default_value=['-r ', str(share / 'worlds/rover_world.sdf')]),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', [str(share.parent), ':', EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')]),
        OpaqueFunction(function=start_gazebo),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create', arguments=['-name', 'irc_rover', '-topic', '/robot_description',
             '-x', LaunchConfiguration('spawn_x'), '-y', LaunchConfiguration('spawn_y'), '-Y', LaunchConfiguration('spawn_yaw'), '-z', '0.03']),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': LaunchConfiguration('bridge_config'), 'use_sim_time': True}]),
    ])

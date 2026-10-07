"""Gazebo-only IRC-inspired off-road driving; no RViz, SLAM or GPU LiDAR."""
from pathlib import Path
import uuid
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    share=Path(get_package_share_directory('rover_bringup'))
    description=Path(get_package_share_directory('rover_description'))
    gui=LaunchConfiguration('gui').perform(context).lower() in ('true','1')
    gpu=LaunchConfiguration('gpu').perform(context)
    vendor=Path('/usr/share/glvnd/egl_vendor.d/10_nvidia.json')
    devices=Path('/proc/driver/nvidia/gpus')
    available=vendor.is_file() and devices.is_dir() and any(devices.iterdir())
    env=[SetEnvironmentVariable('GZ_PARTITION','irc_offroad_'+uuid.uuid4().hex)]
    if gpu=='nvidia' or (gpu=='auto' and available):
        if not available:raise RuntimeError('NVIDIA driver unavailable; use gpu:=system')
        env += [SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD','1'),
                SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME','nvidia'),
                SetEnvironmentVariable('__EGL_VENDOR_LIBRARY_FILENAMES',str(vendor))]
    args=('-r ' if gui else '-r -s ') + str(share/'worlds/irc_offroad.sdf')
    if gui:args+=' --gui-config '+str(share/'config/irc_gui.config')
    return env+[
        LogInfo(msg='IRC-inspired off-road course: Gazebo + DualSense only. RViz, Cartographer and GPU LiDAR are disabled.'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(description/'launch/sim.launch.py')),
            launch_arguments={'gz_args':args,'articulated_arm':'true','suspension':'true',
                'lidar_enabled':'false','spawn_x':LaunchConfiguration('spawn_x'),
                'spawn_y':LaunchConfiguration('spawn_y'),'spawn_yaw':'0',
                'bridge_config':str(share/'config/offroad_bridge.yaml')}.items()),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='arm_bridge',
             parameters=[{'config_file':str(share/'config/arm_bridge.yaml'),'use_sim_time':True}]),
        Node(package='joy',executable='game_controller_node',name='game_controller',
             condition=IfCondition(LaunchConfiguration('joystick')),output='screen',
             parameters=[{'device_id':ParameterValue(LaunchConfiguration('device_id'),value_type=int),
                          'deadzone':.05,'autorepeat_rate':30.,'sticky_buttons':False}]),
        Node(package='rover_bringup',executable='gamepad_teleop',output='screen',
             parameters=[str(share/'config/gamepad.yaml'),{'use_sim_time':True}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('gpu',default_value='auto',choices=['auto','nvidia','system']),
        DeclareLaunchArgument('joystick',default_value='true'),
        DeclareLaunchArgument('device_id',default_value='0'),
        DeclareLaunchArgument('spawn_x',default_value='-15',description='Hardpack starting-grid location'),
        DeclareLaunchArgument('spawn_y',default_value='-11'),
        OpaqueFunction(function=setup),
    ])

"""One command: screenshot-derived maze + rover + arm + DualSense + Cartographer + RViz."""
from pathlib import Path
import uuid
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    share = Path(get_package_share_directory('rover_bringup'))
    description = Path(get_package_share_directory('rover_description'))
    spawn = yaml.safe_load((share / 'config/maze_spawn.yaml').read_text())
    terrain = LaunchConfiguration('terrain').perform(context).lower() in ('true', '1')
    if terrain:
        lane = LaunchConfiguration('terrain_lane').perform(context)
        spawn = {'x': 17., 'y': -9. if lane == 'articulation' else -12., 'yaw': 0.}
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1')
    gpu = LaunchConfiguration('gpu').perform(context)
    vendor_file = Path('/usr/share/glvnd/egl_vendor.d/10_nvidia.json')
    nvidia_devices = Path('/proc/driver/nvidia/gpus')
    available = vendor_file.is_file() and nvidia_devices.is_dir() and any(nvidia_devices.iterdir())
    use_nvidia = gpu == 'nvidia' or (gpu == 'auto' and available)
    rendering = []
    if use_nvidia:
        if not available:
            raise RuntimeError('gpu:=nvidia requested, but NVIDIA GPU/EGL driver was not detected. Use gpu:=system for the system renderer.')
        rendering = [
            LogInfo(msg='GPU: NVIDIA PRIME offload for Gazebo/RViz; NVIDIA EGL for GPU LiDAR.'),
            SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
            SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
            SetEnvironmentVariable('__EGL_VENDOR_LIBRARY_FILENAMES', str(vendor_file)),
        ]
    else:
        rendering = [LogInfo(msg='GPU: using the system/inherited graphics driver.')]
    # EGL renders server-side sensors independently of the desktop window.
    # This does not hide the Gazebo GUI; only -s does that.
    gz_args = ('-r --headless-rendering ' if gui else '-r -s --headless-rendering ') + str(share / 'worlds' / ('terrain_maze.sdf' if terrain else 'reference_maze.sdf'))
    if gui:
        gz_args += ' --gui-config ' + str(share / 'config' / ('terrain_gui.config' if terrain else 'maze_gui.config'))
    # Isolate Gazebo transport from orphaned/other simulations. Every child,
    # including the bridges and GUI, inherits this same launch-local partition.
    # ROS topics remain global: run only one rover launch per ROS_DOMAIN_ID.
    return rendering + [
        SetEnvironmentVariable('GZ_PARTITION', 'irc_rover_' + uuid.uuid4().hex),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(description / 'launch/sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'articulated_arm': 'true',
                              'suspension': LaunchConfiguration('suspension'),
                              'spawn_x': str(spawn['x']), 'spawn_y': str(spawn['y']),
                              'spawn_yaw': str(spawn['yaw'])}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='arm_bridge',
             parameters=[{'config_file': str(share / 'config/arm_bridge.yaml'), 'use_sim_time': True}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/cartographer.launch.py'))),
        # The device and watchdog run on wall time; mapping and physics use /clock.
        Node(package='joy', executable='game_controller_node', name='game_controller',
             condition=IfCondition(LaunchConfiguration('joystick')), output='screen',
             parameters=[{'device_id': ParameterValue(LaunchConfiguration('device_id'), value_type=int),
                          'device_name': LaunchConfiguration('device_name'), 'deadzone': 0.05,
                          'autorepeat_rate': 30.0, 'sticky_buttons': False}]),
        Node(package='rover_bringup', executable='gamepad_teleop', output='screen',
             parameters=[str(share / 'config/gamepad.yaml'), {'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[{'use_sim_time': True}], arguments=['-d', str(share / 'rviz/cartographer.rviz')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('terrain', default_value='false', description='Maze with slopes/pits; spawn in its east test area'),
        DeclareLaunchArgument('terrain_lane', default_value='slopes', choices=['slopes', 'articulation'], description='Starting lane when terrain is enabled'),
        DeclareLaunchArgument('suspension', default_value='true', description='Passive reverse rocker-bogie and coupled differential; false restores rigid wheel mounts'),
        DeclareLaunchArgument('gui', default_value='true', description='Show Gazebo GUI; false uses headless rendering'),
        DeclareLaunchArgument('gpu', default_value='auto', choices=['auto', 'nvidia', 'system'],
                              description='auto selects NVIDIA when its GPU and EGL driver are installed; system inherits graphics settings'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('joystick', default_value='true', description='Start physical SDL controller driver'),
        DeclareLaunchArgument('device_id', default_value='0'),
        DeclareLaunchArgument('device_name', default_value=''),
        OpaqueFunction(function=setup),
    ])

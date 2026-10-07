# Cartographer, maze, RViz and DualSense

The new `rover_bringup` package runs the image-derived maze in Gazebo, your rover
with an approximate two-axis arm, Cartographer 2D, RViz and DualSense controls.
`rover_description` continues to own geometry, joints, collision and physics.

Update: reverse rocker-bogie suspension is now enabled by default. See
[the suspension guide](SUSPENSION_GUIDE.md). Its Gazebo mode uses 3D simulation
ground-truth `/odom` and TF to preserve terrain attitude; this is not encoder-only
localization. Use the 2D map as a mapping demonstration, not pit-safe navigation.

## 1. Build the workspace and connect the controller

```bash
cd /home/inian/ROS2/URDF_IRC
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 run joy joy_enumerate_devices
```

The installation supplies ROS tools, `rosdep` resolves declared dependencies,
`colcon` builds/installs packages, and the overlay makes this workspace discoverable.
The dependency check and build have already passed. For normal use, just source
ROS and the overlay in each terminal, then launch.

Connect the DualSense by USB or pair it using the desktop Bluetooth settings.
This machine detected device **0**, mapped as **PS5 Controller**, and opened it
successfully. Its driver device name is **DualSense Wireless Controller**, while
the mapped SDL display name is **PS5 Controller**. The default `device_id:=0`
avoids that naming difference. With multiple devices, choose the enumerated ID.

We use `joy/game_controller_node`, whose SDL mapping gives consistent button/axis
indices. Do not substitute the generic `joy_node` without recalibrating indices.

## 2. Launch both views and mapping

Stop any previous rover simulation, SLAM Toolbox, AMCL, Nav2 and joystick test
launch before starting this one. Manual teleop is the sole velocity-command
source in this workflow.

```bash
ros2 launch rover_bringup carto_gamepad.launch.py
```

This opens Gazebo with a maze overview and RViz with the live map, scan, robot
and Cartographer trajectory. Zoom towards the rover to inspect the arm. Gazebo
includes entity context-menu and camera-tracking plugins for following the rover.
RViz's fixed frame is `map`; initial missing-map warnings can occur until the
first scans create a submap. Enable `TF tree` in RViz to inspect the frames.

The launch now defaults to `gpu:=auto`: on this laptop it detects the RTX 4050
and enables NVIDIA PRIME offload for Gazebo/RViz and NVIDIA EGL for the LiDAR
renderer. These environment settings apply only to launched processes; they
do not change the desktop GPU or require a reboot. Physics, ROS nodes and
Cartographer still run on the CPU. The server renders its sensors through EGL
even when the Gazebo GUI is visible, avoiding dependence on desktop rendering.

```bash
# Explicitly select this laptop's NVIDIA GPU:
ros2 launch rover_bringup carto_gamepad.launch.py gpu:=nvidia
# Opt out and inherit the system rendering settings:
ros2 launch rover_bringup carto_gamepad.launch.py gpu:=system
# Inspect graphics processes while the simulation runs:
nvidia-smi
```

Gazebo's `~/.gz/rendering/ogre2.log` should report `GL_VENDOR = NVIDIA Corporation`
and an RTX 4050 renderer. Compare simulated time against wall time: a real-time
factor near 1 means the robot is running at real-world speed; 0.1 means it is
ten times slower. A high GPU utilization percentage alone does not prove that
the correct applications are using it.

Useful launch variants:

```bash
# Select a different controller ID if needed:
ros2 launch rover_bringup carto_gamepad.launch.py device_id:=0
# Headless physics server, with RViz still visible:
ros2 launch rover_bringup carto_gamepad.launch.py gui:=false
# Automated testing: no windows or physical controller driver:
ros2 launch rover_bringup carto_gamepad.launch.py gui:=false rviz:=false joystick:=false
```

The last variant still runs the teleop interpreter, which stays at zero without
valid Joy messages. It is not automatic maze exploration.

## 3. Drive like a game

**Centre the left stick and right-stick X, then release both triggers once after
connection or reconnection.** This neutral check prevents startup with a held control.
The drive and arm now arm independently: right-stick-X/trigger neutrality enables
driving, while left-stick neutrality enables the arm. An arm-axis fault no
longer prevents driving. `/teleop/status` explains which group is enabled.

| DualSense control | Behavior |
| --- | --- |
| Right stick left/right | Turn left/right, including turning in place |
| Right stick up/down | Unused; it never commands acceleration or reverse |
| R2 | Proportional forward throttle, up to 0.60 m/s |
| L2 | Proportional reverse throttle, up to 0.35 m/s |
| Both triggers | Zero drive command / brake |
| Left stick left/right | Arm base rotation |
| Left stick up/down | Arm lift joint |
| Release left stick | Hold arm target |
| Circle | Zero drive command and hold arm target; return controls to neutral before resuming |
| Disconnect / no Joy messages for 0.75 s | Zero drive command, hold arm target, require neutral to resume |

Right-stick sideways motion controls yaw, not lateral translation: this is a
skid-steer rover. Linear motion comes only from R2 and L2.
Throttle sets a target speed with a smooth acceleration ramp; releasing it ramps
back to zero rather than maintaining cruise speed. Direction changes pass through
zero. Acceleration is 0.4 m/s², yaw rate is limited to 0.8 rad/s, and arm target
speed to 0.3 rad/s. Circle and the watchdog command zero immediately, but physical
wheel deceleration still has a stopping distance.

The watchdog runs on wall time, so it also works while Gazebo is paused. Arm
integration and normal acceleration use simulation time, avoiding accumulation
while paused. Arm targets initialize from measured joints. Stick deadzones reduce
drift. Joint limits prevent unbounded arm commands; this is joint control, not
inverse kinematics or a collision-aware arm planner.

The mapping in `src/rover_bringup/config/gamepad.yaml` is for SDL: left X/Y are
axes 0/1, right X/Y 2/3, L2/R2 4/5. Left/up are positive. Triggers are **0 released,
-1 pressed**, unlike the +1/-1 convention of some generic joystick drivers.

In a second sourced terminal, move one control at a time and inspect:

```bash
ros2 topic echo /joy sensor_msgs/msg/Joy
ros2 topic echo /cmd_vel geometry_msgs/msg/Twist
ros2 topic echo /joint_states sensor_msgs/msg/JointState --once
ros2 topic echo /teleop/status std_msgs/msg/String
```

If it never arms, check all axes are neutral and Circle is released. If `/joy`
is absent, check device connection/selection. Invalid or short Joy messages are
rejected and cause zero commands. Do not run a second teleop or Nav2 publisher
on `/cmd_vel` during manual mapping.

If the physical left stick is centered but `/joy.axes[0]` remains +1, inspect
the controller connection before changing ROS scaling. During troubleshooting
on this machine, Linux ABS_X itself reported 0 on a 0–255 range (full-left),
while the other stick axes were around 128. That reading originates before
the ROS mapping. Reconnect or compare a USB connection with Bluetooth; do not
calibrate a full-deflection reading as zero, which would disguise a bad input.

## 4. Understand the world and map

`sual-map-construction-in-Gazebo.jpg` is a perspective screenshot, not an SDF
world or a metric occupancy map. The builder uses manually traced wall centerlines
and a four-corner perspective transform. It approximates the visible layout with
**42 wall segments**. No physical scale is supplied, so the chosen size is
**30 x 30 m**, with **0.20 m thick, 1.8 m high walls**. This is an approximate
reconstruction, not a dimensionally verified replica. Walls have both collision
and visual geometry. The spawn is in the central corridor, facing down the image,
with approximately 1.13 m clearance to the closest wall edge.

| Artifact | Meaning |
| --- | --- |
| `src/rover_bringup/worlds/reference_maze.sdf` | Complete physical simulation world |
| `src/rover_bringup/maps/maze_reference.yaml` + `.pgm` | Complete synthetic reference raster, generated from those wall collisions |
| `/map` and maps saved by `save_carto_map` | Cartographer's measured map of the area observed so far |

Cartographer does not consume the screenshot or synthetic raster. It builds
submaps from `/scan` and `/odom`; unknown space remains unknown until observed.
Its map origin is initialized near the rover, not necessarily at Gazebo's world
origin. Do not overlay the synthetic reference in the SLAM map frame without
estimating the coordinate alignment.

Drive slowly down corridors and return through known regions to help loop
closure. Gray space in RViz is unexplored. Manipulating the arm is independent
of SLAM, but changes the robot's physical swept shape.

## 5. Follow the ROS graph

```
DualSense -> game_controller -> /joy -> gamepad_teleop
                                         |
                           /cmd_vel, /arm/{yaw,lift}/command
                                         |
                                 ROS/Gazebo bridges
                                         |
                            wheel drive + arm controllers
                                         |
                /joint_states, /odom, /scan, /imu/data, /clock
                    |            |      |
           robot_state_publisher  +------+--> cartographer_node
                    |                            |
                 /tf, /tf_static        submaps + map->odom TF
                                                 |
                                  cartographer_occupancy_grid_node
                                                 |
                                                /map -> RViz
```

`cartographer_node` estimates 2D trajectory and submaps; the occupancy-grid node
queries them to compose `/map` at 0.05 m/cell. The Lua config uses one LaserScan
stream plus wheel odometry. IMU data is published by the simulator but deliberately
not fused in this 2D setup (`use_imu_data=false`). There are no required point
cloud, GPS, landmark or multi-echo inputs.

| Transform | Sole owner |
| --- | --- |
| `map -> odom` | Cartographer: loop closures and global correction |
| `odom -> base_footprint` | Gazebo differential drive: continuous wheel odometry |
| `base_footprint -> base_link` | Robot state publisher: axle height of 0.15 m |
| `base_link -> six wheel links` | Robot state publisher from measured wheel positions |
| `base_link -> arm_yaw_link -> arm_lift_link` | Robot state publisher from two measured arm positions |
| `base_link -> lidar_link`, `base_link -> imu_link` | Robot state publisher: fixed mounts |

Cartographer sets `tracking_frame=base_link`, `published_frame=odom` and
`provide_odom_frame=false`, leaving the drive's odometry TF intact. Do not run
AMCL or SLAM Toolbox concurrently; they would compete for `map -> odom`.

`robot_state_publisher` consumes the expanded `robot_description` parameter and
`/joint_states`. Fixed transforms go to `/tf_static`; moving joints go to `/tf`.
Gazebo supplies the physical world and sensors; RViz visualizes ROS data.
The arm topics contain `std_msgs/Float64` target angles in radians. The bridge
translates them to Gazebo's two joint position controllers. `/cmd_vel` uses
unstamped `geometry_msgs/Twist`. All simulation consumers use `/clock` through
`use_sim_time=true`; the real controller driver runs on wall time.

Cartographer's services include `/submap_query` (used by the occupancy-grid
node), `/get_trajectory_states`, `/start_trajectory`, `/finish_trajectory` and
`/write_state`. The saver uses WriteState to serialize the graph. No action
servers are added for manual drive or this arm: both use topic commands rather
than FollowJointTrajectory/MoveIt. Nav2 actions are not started in this launch.

Check the chain before diagnosing SLAM:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once --field info
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo base_link arm_lift_link
ros2 service list
```

At real-time speed expect scans near 10 Hz and odometry near 50 Hz. Scan headers
must use `lidar_link`; scans, odometry and TF must share simulation time. Do not
add a static `map -> odom` to conceal missing SLAM transforms.

## 6. Save the map while Cartographer is still running

In a second sourced terminal:

```bash
cd /home/inian/ROS2/URDF_IRC
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run rover_bringup save_carto_map maps/my_maze
```

This saves `my_maze.pbstream` (Cartographer graph and unfinished submaps), and
`my_maze.yaml` + `my_maze.pgm` (occupancy grid for map_server/Nav2). They are
consecutive live snapshots, not a single atomic final optimization. Mapping
continues after saving. Existing files require another prefix or `--overwrite`.
Save before closing the launch; maps are not automatically persisted on shutdown.

`maps/carto_maze_demo.*` was saved from the real Cartographer node during the
short integration test. It is **partial**, not a fully explored maze. Explore
your desired area with the controller and save a new prefix for your final map.

For later saved-map Nav2 use, stop manual teleop and Cartographer before launching
AMCL/Nav2. The existing Nav2 footprint assumes a stowed arm; an extended arm
requires footprint updates and command arbitration. Simultaneous autonomous
navigation and joystick override are not part of this manual mapping launch.

## 7. Arm modeling assumptions

The STL has no CAD joint hierarchy. `prepare_arm.py` extracts 450,897 upper-arm
triangles and retains 981,264 chassis triangles. At zero joint angles they
reproduce the source, apart from an added pivot connector. The original
`chassis.stl` is preserved. The cut is spatial and can include hardware that
would belong to different original CAD links.

The optional proxy adds yaw at `[0.22, -0.045, 0.40]` m relative to `base_link`,
and lift 0.06 m above that. The whole upper assembly moves as one rigid lift
link. Internal elbow, wrist, gripper and linkage constraints are not modeled.
Yaw limits are +/-1.57 rad; lift limits are -0.20 to +0.60 rad. These are estimated,
not specifications of the real arm.

The mobile base is configured as 57 kg without its arm. The provisional
1.5 kg yaw assembly and 6.5 kg lift assembly add 8 kg, giving the IRC 2026
maximum 65 kg with the arm. Inertias and
collision boxes remain provisional. Gazebo uses effort-limited PID position
controllers. The virtual LiDAR is raised 0.25 m in this variant, to 1.35 m above
ground, to clear the proxy arm. This is not a calibrated real sensor mount.

`articulated_arm:=true` enables the arm variant in `rover_description` launches;
the gamepad launch enables it automatically. A faithful hardware arm still needs
CAD link exports, real joint axes/limits, mass properties and motor interfaces.
Suspension is articulated by default; see `SUSPENSION_GUIDE.md`. Self-collision/arm motion
planning is not implemented.

## 8. File guide and small edits

Paths below are relative to `src/rover_bringup/` unless stated otherwise.

| File | Purpose |
| --- | --- |
| `package.xml`, `setup.py`, `setup.cfg`, `resource/rover_bringup` | Dependencies, Python install rules, executable locations and ROS package registration |
| `rover_bringup/__init__.py` | Python module package marker |
| `rover_bringup/control.py` | Testable mapping, neutral arming, rate limits, timeout and joint bounds |
| `rover_bringup/gamepad_teleop.py` | ROS subscriptions/publishers and wall-time watchdog |
| `rover_bringup/save_map.py` | WriteState request plus occupancy map saver |
| `launch/carto_gamepad.launch.py` | Complete Gazebo/Cartographer/RViz/DualSense workflow |
| `launch/cartographer.launch.py` | SLAM-only launch for an already-running robot |
| `config/rover_2d.lua` | Cartographer frames, inputs and scan/submap parameters |
| `config/gamepad.yaml` | DualSense mapping, speeds, limits and timeout |
| `config/arm_bridge.yaml` | Two ROS-to-Gazebo arm command bridges |
| `config/maze_layout.yaml` | Editable source-image trace, scale, wall dimensions and spawn pixel |
| `config/maze_spawn.yaml` | Generated metric spawn and clearance report |
| `config/maze_gui.config` | Gazebo overview camera and interaction plugins |
| `scripts/build_maze.py` | Converts trace into physical SDF and matching reference raster |
| `worlds/reference_maze.sdf` | Generated walls/floor/physics/sensor systems |
| `maps/maze_reference.yaml`, `.pgm` | Generated complete synthetic reference, not SLAM |
| `rviz/cartographer.rviz` | Live map/scan/robot/trajectory view |
| `test/test_control.py` | Controller behavior tests |
| `README.md` | Package quick start |

New description files are `scripts/prepare_arm.py`, `urdf/arm.xacro`, expanded
`urdf/rover_arm.urdf`, and `meshes/arm_assembly.stl`, `chassis_without_arm.stl`,
`arm_report.json`. The report records extraction bounds and assumptions. Existing
`physical.yaml`, `rover.urdf.xacro`, `gazebo.xacro` and `sim.launch.py` now support
optional arm masses/meshes, configurable drive limits and spawn position.

After editing maze scale or wall trace:

```bash
python3 src/rover_bringup/scripts/build_maze.py
colcon build --symlink-install
```

This replaces only generated world/reference/spawn files, not saved SLAM maps.
The maze scale changes, while rover dimensions remain fixed.

After editing robot Xacro/physical properties:

```bash
xacro src/rover_description/urdf/rover.urdf.xacro articulated_arm:=true \
  -o src/rover_description/urdf/rover_arm.urdf
check_urdf src/rover_description/urdf/rover_arm.urdf
colcon test --packages-select rover_bringup --event-handlers console_direct+
colcon test-result --verbose
```

Restart simulation after configuration/model changes. If arm limits change,
update both `arm.xacro` and `gamepad.yaml`. If its pivot changes, update both
the mesh-preparation script and Xacro to keep joint and mesh origins aligned.

References: [ROS joystick mapping](https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md),
[Cartographer configuration](https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html),
[Gazebo joint position controller](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointPositionController.html).

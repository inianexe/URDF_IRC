# Terrain and DualSense: a testable ROS 2 workflow

## 1. Controls

Right stick left/right turns (0.8 rad/s maximum); right-stick up/down is unused.
Left stick controls arm yaw/lift only. R2/L2 are the only forward/reverse
throttles (0.60/0.35 m/s maximum); both together brake. Circle
stops. Release the controls after connecting or stopping to re-arm. Input loss
stops the base after 0.75 wall-clock seconds. Arm and base have independent
neutral gates. A faulty/non-neutral left axis still prevents arm arming.

`config/gamepad.yaml` uses SDL ordering from `joy/game_controller_node`:
left X/Y = 0/1, right X/Y = 2/3, L2/R2 = 4/5. Axis 3 (right Y)
is intentionally ignored. Only the trigger axes generate linear velocity.

## 2. Build and launch

From `/home/inian/ROS2/URDF_IRC`:

```bash
source /opt/ros/jazzy/setup.bash
# On a fresh machine, install declared dependencies first:
rosdep install --from-paths src --ignore-src -r -y
python3 src/rover_bringup/scripts/build_terrain.py
colcon build --symlink-install --packages-up-to rover_description rover_bringup
source install/setup.bash
ros2 launch rover_bringup carto_gamepad.launch.py terrain:=true gpu:=nvidia
```

Use `terrain:=false` for the original maze. The suspension update adds the
`rover_simulation` package; build it through `--packages-up-to` as shown above.
After an unexpected laptop restart, first use `gui:=false rviz:=false` to test
the server alone. NVIDIA selection does not diagnose or fix a system restart.

The rover starts at (17, -12), facing +X in the new eastern test area.
Drive straight to reach a 0.25 m hill centered at x=20 (about 9.5 degrees),
then a 0.18 m deep depression centered at x=25 (about 8.5 degrees).
The second lane around y=-9 has staggered +0.12/-0.10 m wheel-track features.
These are chosen synthetic dimensions, not measurements of real terrain.
Approach slowly. Pits are genuinely below z=0, with no flat floor underneath
their opening. The course uses a closed static triangle collision mesh and
gravity; the tire friction remains the provisional value in `physical.yaml`.

## 3. Packages, files and ROS graph

`rover_description` contains the Xacro links, wheel collision shapes, estimated
inertias, sensor mounts and Gazebo drive plugin. `rover_bringup` contains launch,
mapping and controller configuration. New `scripts/build_terrain.py` generates
`worlds/suspension_terrain.stl` and `worlds/terrain_maze.sdf` from the existing
maze. It opens a gate and adds a separate course without changing the original
maze or synthetic occupancy map. `launch/carto_gamepad.launch.py` selects the
world and spawn through the `terrain` parameter. `control.py` implements the
stick mapping; `test/test_control.py` checks separation, limits and failsafes.

`game_controller` publishes `/joy`; `gamepad_teleop` converts it into `/cmd_vel`
and arm position commands. The bridges deliver these to Gazebo and return
`/joint_states`, `/scan`, `/imu/data`, `/odom` and simulation `/clock`.
`robot_state_publisher` uses the URDF plus joint states for robot TF.
Cartographer consumes scan/odometry and publishes map-to-odom TF; its occupancy
grid node publishes `/map`. Gazebo's drive plugin supplies odom-to-base_footprint;
the URDF connects base_footprint, base_link, wheels, arm and sensors. RViz shows
these frames, robot, scan and map; Gazebo renders and simulates physical terrain.
No new ROS services or actions were added by this change.

## 4. Verify before driving

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest src/rover_bringup/test/test_control.py -q
gz sdf -k src/rover_bringup/worlds/terrain_maze.sdf
ros2 topic echo /joy --once
ros2 topic echo /teleop/status --once
ros2 topic echo /cmd_vel
# Separate terminals:
ros2 topic hz /scan
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo map base_link
```

Right-stick X should change only base angular velocity; right-stick Y should do
nothing. Left-stick input should change only arm targets. Verify this in Gazebo,
and check RobotModel/TF in RViz.

## Important limits: suspension and navigation

The new default is an articulated **reverse rocker-bogie** with a compliant
mechanical differential approximation, using the additional CAD and mass
screenshots. See [SUSPENSION_GUIDE.md](SUSPENSION_GUIDE.md). Select
`terrain_lane:=articulation` to start at the offset wheel-track features.
`suspension:=false` restores the old rigid model. DiffDrive still controls only
wheel rotation; the separate `rover_simulation` plugin couples the passive rockers.
Joint limits, damping, differential stiffness, chassis mass, arm geometry and
inertias remain provisional; these are not calibrated hardware dynamics.

The suspension mode uses 3D Gazebo ground-truth odometry, not wheel-only odometry.
The 2D Cartographer configuration still assumes largely planar mapping.
Its map is not a terrain elevation map; pits may be invisible to
horizontal LiDAR. Do not treat this course as validated autonomous Nav2 terrain
navigation. The old reference occupancy map covers only the flat maze and is
not updated terrain ground truth. Use manual driving for these physics tests.

## RViz jumping / duplicate simulation diagnosis

An orphaned test server and a second maze server were found publishing to the
same Gazebo `/odom` and world pose topics. RViz reported `TF_OLD_DATA` and
Cartographer exited. Both old servers were stopped and a single stack restarted.
The launch now assigns a unique `GZ_PARTITION` to its server, GUI and bridges,
preventing cross-talk with another Gazebo transport session. External Gazebo
CLI tools must use that same partition to inspect it. ROS topics are still
global: do not start two rover stacks in the same `ROS_DOMAIN_ID`.

After rebuilding `rover_bringup` and sourcing the overlay, verification showed
zero backwards clock/odom timestamps over 15 seconds, a live 286 x 463 map,
and approximately 0.87 real-time factor with both GUIs. Idle wheel joint ranges
were below 2e-11 radians; arm lift range was 0.0023 radians. This is a stationary
sanity check, not a validation of suspension dynamics over obstacles.

For debugging, inspect `ros2 node list`, `ros2 topic info /clock -v`, and
`ros2 run tf2_ros tf2_echo map base_link`. One ROS bridge publisher does not
prove there is only one Gazebo source: inspect Gazebo publishers too. Gazebo
supplies odom-to-base TF, and Cartographer supplies map-to-odom; if mapping
crashes, RViz's fixed `map` frame cannot resolve the robot pose correctly.

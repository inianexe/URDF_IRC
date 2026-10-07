# IRC rover: ROS 2 Jazzy description, simulation and navigation

Latest update: the default propulsion is a 25 N·m torque-capped servo model and
wheel inertia uses the honeycomb CAD voids. See the workspace
`ELECTRICAL_WHEELS_BLENDER_PLAN.md`; older baseline notes below are historical.
Battery discharge and elastic tire deformation are still not modeled.

This package converts `CF_circular tube_v2.stl` into a six-wheel rover model.
The supplied Fusion dimension screenshot confirms **300 mm wheel diameter**,
**500 mm adjacent axle spacing**, **1000 mm wheelbase**, and a **600 mm long,
250 mm high chassis section**. It does not specify mass, wheel track, sensor
mounts, motor specifications or suspension joint limits.

The model now defaults to an **articulated reverse rocker-bogie**, with supplied
suspension masses and a compliant left/right differential. See the workspace
`SUSPENSION_GUIDE.md` for current geometry, physics, validation and limitations.
The rigid model remains available with `suspension:=false`. The older conversion
notes below describe that baseline. This is not a calibrated digital twin or a
complete hardware driver; chassis mass and inertias remain estimates.

## 1. Understand what was converted

The source has 2,181,460 triangles. Its axes are CAD X longitudinal, Y vertical,
Z lateral. The mesh preparation script converts millimetres to metres and rotates
to ROS X forward, Y left, Z up (CAD +X is provisionally forward). It spatially
extracts six wheel regions and keeps every triangle exactly once across seven
meshes. This is an approximate geometric split, not the CAD assembly hierarchy.
Wheel regions can contain hub hardware that should eventually be separated in CAD.

Wheel visual centers are normalized to symmetric axles on one horizontal plane.
This moves them a few millimetres from the raw CAD poses. The tread mesh radius is
about 151–152 mm; the collision radius is the dimensioned nominal 150 mm. Thus
tread tips can visually penetrate the ground by about 2 mm. Original source
files are preserved. `meshes/mesh_report.json` records the extraction bounds.

`base_footprint` is the ground projection at the middle axle; `base_link` is
150 mm above it. Positive wheel rotation about +Y drives the rover forward.
Three wheels per side share differential/skid-steer commands in simulation.
Track (0.815 m) and width (0.165 m) are approximate STL measurements.

For IRC 2026, the mobile base is configured to 57 kg and the provisional arm
adds 8 kg, so the deployed base+arm configuration is exactly at the 65 kg cap.
The 0.3 kg LiDAR and 0.05 kg IMU are included in the 57 kg base total. Inertia
uses an equivalent box for `base_link`, component-box and
parallel-axis estimates for suspension links, and the CAD-derived honeycomb
tensor for wheels, all centered at their configured centres of mass.
Replace these with measured/CAD mass properties when available. STL contains
neither material density nor component masses, so it cannot establish them.

Collision geometry uses two coarse body boxes, six wheel cylinders and a sensor
cylinder. These are approximations to the complex chassis/arm, not exact surface
contact geometry. URDF supplies mass, inertia, collision and joint dynamics;
Gazebo adds gravity, contacts, friction, drive and sensors. The simple Gazebo
drive regulates wheel velocity; URDF effort values alone do not model motor
current, gearboxes, torque curves, tires or suspension compliance. This model
does not reproduce rough-terrain suspension behavior. A 2D LiDAR also cannot
observe every low/overhanging obstacle; the virtual sensor is deliberately above
the tall assembly to avoid self-occlusion. Confirm real sensor visibility.

## 2. Workspace and files

This directory is a standard colcon workspace: packages live under `src/`, while
colcon generates `build/`, `install/` and `log/`.

| File/directory in `src/rover_description/` | Purpose |
| --- | --- |
| `package.xml` | Package identity and ROS/system dependencies |
| `CMakeLists.txt` | Installs the description and launch/configuration assets using ament |
| `urdf/rover.urdf.xacro` | Editable master: links, joints, visual/collision shapes, inertia and frames |
| `urdf/rover.urdf` | Expanded plain URDF, usable by URDF readers after sourcing this package |
| `urdf/gazebo.xacro` | Optional Gazebo Harmonic drive, joint state, LiDAR and IMU definitions |
| `meshes/*.stl` | Seven extracted visual meshes, already in metres and ROS axes |
| `meshes/mesh_report.json` | Source triangle count, extraction origins and mesh bounds |
| `config/physical.yaml` | Dimensions, estimated masses/COM, wheel limits, friction and sensor locations |
| `config/irc_2026.yaml` | Machine-readable competition mass, size and task-object limits |
| `scripts/check_irc_2026_compliance.py` | Expands both configurations and checks mass plus sampled moving envelopes |
| `config/bridge.yaml` | Explicit ROS–Gazebo message mappings and directions |
| `config/slam.yaml` | SLAM Toolbox scan matching and mapping parameters |
| `config/nav2.yaml` | Complete Jazzy Nav2 configuration, rover footprint and conservative speeds |
| `launch/display.launch.py` | Publishes description, optional joint sliders and RViz |
| `launch/sim.launch.py` | Gazebo world, spawned rover, state publisher and message bridge |
| `launch/navigation.launch.py` | SLAM+Nav2 or saved-map localization+Nav2 |
| `rviz/rover.rviz` | Model, TF, grid and scan inspection view |
| `worlds/rover_world.sdf` | Offline test room with ground, four walls, obstacle and physics/sensor systems |
| `scripts/prepare_meshes.py` | Reproducible conversion for this specific source STL |
| `scripts/prepare_navigation.py` | Development helper to derive YAML from the installed Jazzy defaults |
| `README.md` | This step-by-step workflow and assumptions |

Edit Xacro/configuration first; plain URDF is a generated snapshot. `physical.yaml`
changes are loaded on launch, not live while running. Collision body boxes live
in Xacro. Update the navigation footprint in BOTH costmaps if geometry changes.
Do not rerun `prepare_navigation.py` after tuning YAML unless you intend to replace
your tuning. Do not scale the prepared meshes by 0.001 again.

## 3. Build and inspect the robot

In each new terminal, source the ROS installation and workspace overlay. The
installation supplies ROS tools; the overlay makes this package discoverable.

```bash
cd /home/inian/ROS2/URDF_IRC
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch rover_description display.launch.py
```

RViz's fixed frame is `base_footprint`. Move the six joint sliders: each wheel
should rotate about its axle. In RobotModel, enable Collision Enabled to compare
the simplified collision geometry with the CAD visual. The 105 MB detailed mesh
may take time to load. Missing wheel transforms mean joint states are missing;
missing meshes usually mean the overlay has not been sourced.

```bash
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo base_link front_left_wheel_link
ros2 run tf2_ros tf2_echo base_link lidar_link
```

Stop display.launch before simulation to avoid duplicate state publishers.
To regenerate and validate the plain URDF after edits:

```bash
xacro src/rover_description/urdf/rover.urdf.xacro \
  -o src/rover_description/urdf/rover.urdf
check_urdf src/rover_description/urdf/rover.urdf
```

## 4. Run physics and verify data before mapping

```bash
ros2 launch rover_description sim.launch.py
```

This launches Gazebo Harmonic and creates the rover slightly above the floor.
The drive plugin publishes wheel odometry and joint states, the sensor plugins
produce scans and IMU readings, and the bridge exposes them as ROS messages.
Gazebo supplies `/clock`; all simulation consumers must use `use_sim_time:=true`.

For a headless server (GPU rendering is still needed for LiDAR):

```bash
ros2 launch rover_description sim.launch.py \
  gz_args:='-r -s --headless-rendering /home/inian/ROS2/URDF_IRC/src/rover_description/worlds/rover_world.sdf'
```

In a sourced second terminal:

```bash
ros2 topic echo /scan --once --field header
ros2 topic echo /odom --once
ros2 topic echo /imu/data --once --field header
ros2 topic hz /joint_states
ros2 run tf2_ros tf2_echo odom base_footprint
```

Expect `lidar_link` in scan headers, `imu_link` in IMU headers, and `odom` to
`base_footprint` from the drive. The IMU is available but is not fused with the
simulated wheel odometry in this baseline. Friction and skid-steer odometry
need calibration; wheel odometry can diverge from physical motion during turns.

To inspect simulation in RViz without adding another robot publisher:

```bash
rviz2 -d src/rover_description/rviz/rover.rviz --ros-args -p use_sim_time:=true
```

## 5. Map, then navigate

With simulation still running, start SLAM and Nav2 in another sourced terminal:

```bash
ros2 launch rover_description navigation.launch.py use_sim_time:=true mapping:=true
ros2 launch nav2_bringup rviz_launch.py use_sim_time:=true
```

These are long-running commands: run RViz in its own terminal. SLAM Toolbox
creates `/map` and `map -> odom`. Select a nearby free location using RViz's
Nav2 Goal tool. Nav2 plans a path, tracks it and outputs `/cmd_vel`; the simulator
moves the wheels and publishes the resulting odometry. Explore progressively,
keeping goals in observed free space. The footprint is 1.36 x 1.02 m with an
additional 0.02 m padding, rather than the default small TurtleBot radius.
Configured speeds are 0.3 m/s and 0.5 rad/s; these are simulation starting points.

Useful checks:

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 topic echo /map --once --field info
ros2 action list -t
ros2 run tf2_ros tf2_echo map base_footprint
```

The lifecycle nodes should be `active`. `/navigate_to_pose` accepts a map-frame
goal and reports feedback/result. `/compute_path_to_pose` requests a plan;
`/follow_path` executes path tracking; `/navigate_through_poses` and
`/follow_waypoints` handle multiple goals. Navigation behaviors expose spin,
backup and wait actions. The stock bringup also starts optional route/docking
servers; no physical dock or route graph is defined by this package.

Save a map using Nav2's map saver (its save-map service is invoked by the CLI):

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f maps/rover_room \
  --ros-args -p use_sim_time:=true
```

Stop only `navigation.launch.py`, keep the robot/simulator running, then start
saved-map navigation:

```bash
ros2 launch rover_description navigation.launch.py use_sim_time:=true \
  mapping:=false map:=/home/inian/ROS2/URDF_IRC/maps/rover_room.yaml
```

Use RViz's **2D Pose Estimate** to initialize AMCL, then send a Nav2 goal. AMCL
now owns `map -> odom`; do not leave SLAM running at the same time. Map server
publishes the saved occupancy grid. Lifecycle services configure/activate Nav2
nodes automatically through the lifecycle manager; no custom services are added.

## 6. Understand the ROS graph and the hardware boundary

```
map --(SLAM Toolbox OR AMCL)--> odom --(drive/estimator)--> base_footprint
                                                            |
                                               robot_state_publisher
                                                            |
                                                        base_link
                                                     /      |       \
                                                6 wheels lidar_link imu_link
```

`robot_state_publisher` reads the `robot_description` parameter (expanded URDF),
publishes the description topic and computes TF using `/joint_states`. Fixed
transforms go on `/tf_static`; wheel motion goes on `/tf`. The GUI joint state
publisher is only for inspection. In simulation the Gazebo joint state plugin
supplies measured joint positions; on hardware your motor driver/controller must.

| Data / node | Role |
| --- | --- |
| `/scan` (`sensor_msgs/LaserScan`) | Range data for SLAM, AMCL, costmaps and collision monitoring |
| `/odom` (`nav_msgs/Odometry`) | Local motion estimate; paired with `odom -> base_footprint` TF |
| `/imu/data` (`sensor_msgs/Imu`) | Angular velocity/acceleration/orientation for a future state estimator |
| `/cmd_vel` (`geometry_msgs/Twist`) | Final commanded forward/angular speed consumed by the drive |
| `/cmd_vel_nav`, `/cmd_vel_smoothed` | Internal Nav2 controller/behavior -> velocity smoother -> collision monitor chain |
| `/map` (`nav_msgs/OccupancyGrid`) | SLAM-generated or saved-map occupancy grid |
| `planner_server`, `controller_server` | Compute route around obstacles, then track it using the local costmap |
| `smoother_server`, `behavior_server` | Smooth paths and execute recovery behaviors |
| `bt_navigator`, `waypoint_follower` | Coordinate navigation actions and waypoint sequences |
| `velocity_smoother`, `collision_monitor` | Limit command changes and stop/slow based on observed collisions |
| `lifecycle_manager_navigation` | Configures and activates Nav2's managed nodes |
| `async_slam_toolbox_node` | Uses scan/odometry to map and localize while mapping |
| `amcl`, `map_server` | Localize against and publish a saved map in localization mode |
| `ros_gz_bridge` | Translates only the configured simulation topics |

For the real rover, launch only the description publisher:

```bash
ros2 launch rover_description display.launch.py gui:=false rviz:=false
```

Then your actual drivers must publish `/joint_states`, `/scan`, `/odom`, and the
odometry TF; they must accept `/cmd_vel`. A hardware controller or an EKF (for
example robot_localization) should own `odom -> base_footprint`, never both.
Add an EKF only after choosing the real encoder/IMU inputs and covariance values.
Set `use_sim_time:=false` for hardware and start navigation with that setting.
Confirm physical wheel track, drive topology, COM/masses, joint limits, LiDAR/IMU
poses and forward direction before transferring this provisional setup to hardware.
There is no motor interface hidden inside a URDF and no real LiDAR driver here.

If scans appear but no map is produced, check scan timestamps, simulation time,
`lidar_link` TF and odometry TF first. If navigation is inactive, inspect launch
errors and lifecycle state. If the rover cannot turn, examine tire contact
friction and the actual drive topology rather than changing the visual mesh.
Gazebo may report that `gz_frame_id` is an extension to SDF; the extension is
intentional and should result in the ROS sensor frame IDs shown above.

Reference workflows: [Nav2 robot setup](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/gazebo/),
[robot_state_publisher](https://github.com/ros/robot_state_publisher),
[Gazebo differential drive](https://gazebosim.org/api/sim/8/DiffDrive_8hh.html).

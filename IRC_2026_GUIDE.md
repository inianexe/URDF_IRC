# IRC 2026 practical implementation

This workspace uses the saved IRC 2026 rulebook as a design gate. The official
PDF remains authoritative; this guide records what is enforced in simulation,
what is approximated, and what still requires physical hardware and testing.

## 1. What changed

| Rulebook constraint | Workspace implementation | Status |
| --- | --- | --- |
| Deployed rover mass <= 65 kg | 57 kg complete mobile base + 8 kg arm; Xacro accounts for every modeled link | Enforced in test; physical rover must be weighed |
| All fielded parts <= 85 kg | Known modeled set is 65 kg; future science modules must be added to the manifest | Passes for current known parts only |
| Length <= 1.5 m, breadth <= 1.2 m | Conservative mesh envelope is sampled at zero and both limits of suspension and arm joints | Enforced in test |
| Exterior, visible, accessible kill switch | Red `kill_switch_link`; `/safety/estop` latches command shutdown | Simulation implemented; hardware power isolation still mandatory |
| Battery power only | No tethered-power model is claimed | Hardware design item |
| Wireless, blind operation to 500 m | Current Gazebo GUI is not a radio-link test | Not implemented |
| Red terrain/objects are no-go | Two collision-free red exclusion disks are visible in Gazebo | Implemented as training cues, not automatic scoring |
| RADO/IDMO object <= 5 kg and <= 0.40 m | Dynamic 4.5 kg, 0.22 m cache | Implemented |
| IDMO handle >= 0.10 m and <= 0.05 m diameter | Cache handle is 0.12 m x 0.04 m | Implemented |
| IDMO panel <= 1.5 m high | 1.40 m panel with representative drawer, button, knob and socket | Geometry implemented; controls are not articulated |
| ABEx sample >= 10 cm deep and >= 10 g | Sampling target and required values are in the terrain manifest | Marker only; sampling system is not implemented |
| 30 minute mission timing | Values are recorded in `irc_2026.yaml` | No scoring/timer node yet |

The earlier 65 kg base plus 8 kg arm configuration was not legal. It is now a
57 kg base plus 8 kg arm. These are design targets, not measured facts. The
current conservative moving envelope is about 1.497 m long and 1.147 m broad.
To remain within the rules at extreme poses, suspension travel is limited to
rocker +/-0.20 rad and bogie +/-0.35 rad; provisional arm yaw is +/-0.90 rad.
Do not widen these limits without rerunning the compliance check.

## 2. ROS 2 project structure and graph

- `rover_description` owns Xacro, meshes, TF structure, inertias and the
  machine-readable `config/irc_2026.yaml` constraints.
- `rover_simulation` owns the Gazebo torque-drive and rocker differential
  system plugins. Six wheel motors are torque-capped by `electrical.yaml`.
- `rover_bringup` owns the Gazebo course, launch files and operator/safety node.

The Gazebo-only control graph is:

```text
/joy (sensor_msgs/Joy) ----------------------+
                                              v
/joint_states ------------------------> gamepad_teleop
/safety/estop (std_msgs/Bool) --------->      |  +--> /arm/yaw/command
/safety/reset_estop (std_srvs/Trigger) ->     |  +--> /arm/lift/command
                                              +-----> /cmd_vel
                                                         |
                                                         v
                                          Gazebo torque-drive plugin
                                                         |
                                  six wheel joints, /odom, /joint_states

robot_state_publisher: robot_description + /joint_states -> /tf, /tf_static
```

`game_controller_node` publishes `/joy`. `gamepad_teleop` is the only manual
owner of `/cmd_vel`: R2 commands forward speed, L2 reverse speed, right-stick X
only commands yaw, and the left stick commands the provisional arm. It publishes
JSON diagnostics on `/teleop/status`. The E-stop topic is level-sensitive on
assertion but its result is latched; publishing `false` cannot cause motion.
The reset service succeeds only after the asserted input has been released, and
the neutral-start gate must then pass again.

TF is `odom -> base_footprint -> base_link`, then branches to six wheel links,
four suspension links, `imu_link`, `lidar_link`, two arm links and
`kill_switch_link`. The off-road launch uses Gazebo ground-truth odometry; never
substitute that transform for a real encoder/IMU estimator.

There are no mission actions in this Gazebo-only launch. The separate navigation
bringup exposes Nav2 actions, but RADO autonomous pickup/delivery logic is not yet
implemented and must not be represented as competition-ready.

## 3. Build in small, testable steps

Start from a fresh terminal:

```bash
cd /home/inian/ROS2/URDF_IRC
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

First verify the description without launching physics:

```bash
ros2 run rover_description check_irc_2026_compliance.py
xacro src/rover_description/urdf/rover.urdf.xacro \
  articulated_arm:=true suspension:=true > /tmp/irc_rover.urdf
check_urdf /tmp/irc_rover.urdf
```

The compliance program expands both the base and arm variants, sums inertial
masses, reads the actual STL bounds, and samples the arm/suspension endpoints.
It should report 57.000 kg, 65.000 kg and `PASS`.

Then launch the Gazebo course (no RViz):

```bash
ros2 launch rover_bringup irc_offroad.launch.py gpu:=nvidia
```

Use `gpu:=system` if the NVIDIA driver is unavailable. In another sourced
terminal, verify the graph before driving:

```bash
ros2 node list
ros2 topic list
ros2 topic hz /joy
ros2 topic echo /teleop/status --once
ros2 topic echo /odom --once
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo base_link kill_switch_link
```

Test the software safety path with the rover stationary:

```bash
ros2 topic pub --once /safety/estop std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /safety/estop std_msgs/msg/Bool '{data: false}'
ros2 service call /safety/reset_estop std_srvs/srv/Trigger '{}'
```

After the first command, `/cmd_vel` is forced to zero and the state remains
`estop_latched` even after the second command. The service clears the latch;
release both triggers and centre the steering stick before trying to drive.

Run the repeatable checks after any geometry, mass or limit change:

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

## 4. Course features and how to train with them

`build_irc_terrain.py` creates a deterministic 36 x 28 m laptop-scale Sproscape
with 124 rocks, four craters, ruts, a gully, blind drop, 15-30 degree formations
and four friction zones. It now also creates:

- two red no-go disks for route-planning and driver-discipline tests;
- a dynamic 4.5 kg cache at the delivery zone;
- a 1.40 m IDMO panel at the servicing pad;
- an ABEx sampling marker on the loose-soil patch.

Rebuild it after editing the source constants:

```bash
source /opt/ros/jazzy/setup.bash
python3 src/rover_bringup/scripts/build_irc_terrain.py
colcon build --symlink-install --packages-select rover_bringup
```

The world tests dimensions and required props. It does not score boundary
crossings, simulate deformable sand, dust, contamination, sample collection or
RF loss. A rigid contact-friction patch cannot predict sinkage or wheel digging.

## 5. Highest-priority work before this is a practical competition rover

1. Replace the two-axis arm proxy with the real arm and gripper links, joint
   origins, limits, motor torques, masses and collision meshes. The current arm
   cannot manipulate the cache or panel.
2. In Blender/CAD, split every moving part at its true joint, set each origin on
   the measured joint axis, apply metre scale and transforms, export one visual
   mesh per link, and create separate low-poly convex collision meshes. Export
   CAD mass, centre of mass and inertia rather than estimating from triangles.
3. Add at least navigation and manipulation cameras, GNSS in WGS84, radio-link
   telemetry, signal-strength logging and a congested-link/dropout test. Operators
   must train without the Gazebo third-person view.
4. Build the exterior hardware kill circuit so it physically isolates battery
   power from every actuator. The ROS latch is a secondary software layer only.
5. Implement the ABEx sampling tool and sealed uncontaminated cache, then model
   the 10 cm insertion stroke and retained sample mass.
6. Calibrate motor torque/speed curves, battery voltage sag, current limits,
   wheel-ground friction and whole-rover COM from measurements. Blender models
   geometry; tire compliance and soil deformation belong in the physics model.
7. Implement and test autonomous RADO delivery with all commands disabled during
   the autonomous phase, plus mission timing, scoring and no-go-zone monitoring.
8. Weigh and measure every fielded configuration. Keep margin below 65 kg and
   1.5 x 1.2 m rather than designing exactly to a penalty boundary.

# Gazebo-only IRC-inspired off-road driving

The drive now uses a 25 N·m per-wheel torque-capped controller and mesh-derived
honeycomb wheel inertia. See [the electrical/wheel update](ELECTRICAL_WHEELS_BLENDER_PLAN.md)
for assumptions, telemetry, remaining calibration and the Blender plan. The
terrain trial recorded below predates this motor-controller change.

This is a deterministic **36 × 28 m scaled interpretation of `terrain.txt`**, not
an official or surveyed IRC arena. Simulating the brief's complete 20,000 m² and
500 m traverse at wheel-scale resolution would be wasteful on this laptop. The
scaled course retains the important dynamics: 15–30° mounds/ridges, a gully,
blind drop, ruts, four 0.30–1.05 m craters and 124 rocks spanning the specified
5–60 cm sizes. Depressions are real collision geometry; no flat plane masks them.

Four colored posts mark Autonomous, Astrobiology, Delivery and Servicing areas.
Hardpack, gravel, loose-soil and servicing contact patches use different friction
and slip values. These remain rigid-contact approximations: Gazebo does not model
soil displacement, wheel sinkage or dust here.

The original maze and suspension test worlds remain available unchanged.

## 1. Launch

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rover_bringup irc_offroad.launch.py gpu:=nvidia
```

This opens **only Gazebo**. There is no RViz, Cartographer, occupancy-grid node,
Nav2, or GPU LiDAR workload. ROS still runs the joystick, rover description and
bridges in the background; these are required for the controls. IMU and simulated
ground-truth odometry remain available for physics diagnostics. NVIDIA selection
is scoped to the launch, not a global laptop setting.

Right-stick left/right steers and right-stick up/down is unused. The left stick
moves arm yaw/lift. R2/L2 are the only forward/reverse throttles; both triggers
brake and Circle stops. Release the active controls after connection to arm them.
Input loss stops driving after 0.75 wall-clock seconds; the motor plugin has an
independent 0.8 s command watchdog. Stop this launch before starting another stack.

## 2. Explore in small steps

The rover starts at **(-15, -11)** facing +X on the hardpack starting grid. The
Gazebo camera initially overlooks the full scaled course.

| Area | Approximate coordinates | Exercise |
| --- | --- | --- |
| Hardpack start | (-15, -11) | High-traction controls and stopping check |
| Delivery gravel | (0, -10) | Slip and high-frequency rock response |
| Rounded mound | (-6, -4) | Approximately 20° articulation climb |
| Loose ABEx soil | (-10, 2) | Low-traction approach test |
| Long gully | x ≈ 3, y = -8…10 | Clearance and chassis-trap test |
| Ridge / blind drop | (7.5…11, 5.5) | Approximately 27° crest and hidden descent |
| Servicing pad | (-11.5, 9.5) | Stable arm-operation surface |
| Large hazard crater | (13, 8) | 1.05 m deep; intentionally not generally traversable |

The perimeter has low barriers. The terrain is not guaranteed traversable at
every heading or speed. Use partial stick input for obstacles; avoid high-speed
side-slope turns. The rover's reverse rocker-bogie and differential remain active.

## 3. Packages and ROS graph

`rover_description` supplies links, joints, inertias and collision geometry.
`rover_simulation` applies passive differential torque at each physics timestep.
`rover_bringup` supplies the course, launch and DualSense control node.

`game_controller` publishes `/joy`; `gamepad_teleop` publishes `/cmd_vel` and the
arm yaw/lift commands. `ros_gz_bridge` and `arm_bridge` connect these to Gazebo.
Gazebo returns `/joint_states`, `/odom`, `/tf`, `/imu/data` and `/clock`.
`robot_state_publisher` creates link TF from the URDF and joint states.
Gazebo supplies `odom → base_footprint`; robot-state TF continues through
`base_link`, the rockers, bogies, wheels, arm and sensors. There is **no `map`
frame or `/map` publisher** in this mode because mapping is intentionally absent.

`/teleop/status` reports controller readiness, `drive_state` and joystick-message
age. `/drive/motor_status` reports per-wheel speed/torque/power plus Gazebo command
age and whether its watchdog is stale. No new ROS services or actions are
introduced. Gazebo's existing world-control services handle pause/reset. Physics
nodes use simulation time; both command watchdogs use wall time.

## 4. Files and rebuilding

- `src/rover_bringup/scripts/build_irc_terrain.py`: deterministic terrain/rock
  generator; fixed seed 20260927. Edit `CRATERS`, `SURFACES`, `height()` or rock placement here.
- `worlds/irc_offroad.sdf`: the world, physical obstacles, sunlight and materials.
- `worlds/irc_ground.stl`: closed collision/render mesh at approximately 0.15 m spacing.
- `worlds/irc_rock_0.stl` through `irc_rock_7.stl`: reusable convex stone shapes.
- `config/irc_terrain_manifest.json`: source assumptions, surfaces, hazards and rock locations.
- `config/irc_gui.config`: Gazebo-only camera and GUI layout.
- `config/offroad_bridge.yaml`: control and diagnostics bridges; no scan bridge.
- `launch/irc_offroad.launch.py`: GPU selection, isolated Gazebo transport,
  model spawn, joystick and arm controls; no mapping/visualization nodes.
- `test/test_irc_terrain.py`: flat spawn, crater depth, slope class, surface
  physics, watertight meshes, rock collisions and disabled LiDAR checks.

Rebuild after edits using the standard ROS 2 workflow:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
python3 src/rover_bringup/scripts/build_irc_terrain.py
colcon build --symlink-install --packages-up-to rover_description rover_bringup
source install/setup.bash
colcon test --packages-select rover_bringup rover_simulation
colcon test-result --verbose
gz sdf -k src/rover_bringup/worlds/irc_offroad.sdf
```

The source scripts use NumPy/SciPy; their dependencies are declared. The build
installs the world/assets and compiles the Gazebo differential plugin first.
`gui:=false joystick:=false` allows repeatable headless tests. The existing
`scripts/validate_suspension.py --drive` helper can record a simulation-only
traverse into JSON; never use this helper with a physical robot.

## 5. Verify without RViz

In another terminal, source ROS and the workspace overlay, then:

```bash
ros2 node list
ros2 topic echo /teleop/status --once
ros2 topic echo /drive/motor_status --once
ros2 topic hz /joy
ros2 topic echo /joint_states --once
ros2 topic echo /odom --once
ros2 topic hz /imu/data
```

There should be no `rviz2` or Cartographer nodes. Watch Gazebo's real-time-factor
display and wheel-ground contact. Suspension state should change over obstacles
and settle when stopped. `/odom` here is simulation ground truth, not measured
encoder localization. There is no `/scan` output in this lightweight mode.

## Limits

Gazebo's SDF validator and automated geometry tests accept this generated world.
The older controlled traverse in `docs/irc_offroad_validation.json` applies only
to the previous 24 × 20 m course and is retained as historical data; the new
routes still require measured drive trials.

This models **rigid ground and fixed rocks**, not deformable tires, sand sinkage,
dust, or calibrated soil traction. Surface friction/slip values are estimates.
Visual rock shapes and collision shapes match, but component inertia, differential
stiffness, damping and joint stops retain the assumptions in
[SUSPENSION_GUIDE.md](SUSPENSION_GUIDE.md). Simulation success does not establish
real-rover obstacle capability or rollover safety.

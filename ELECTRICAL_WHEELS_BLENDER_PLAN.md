# Electrical limits, honeycomb wheels and Blender plan

## 1. What your electrical list establishes

`Electrical_details.txt` states 25 N·m, six wheel motors, three batteries with
sufficient power for simultaneous motor demand, a six-DOF arm, and the need to
account for wheel structure and arm rest-pose CG. It does **not** specify whether
25 N·m is shaft or gearbox-output torque, continuous or peak torque, nor voltage,
current, gearing, battery wiring, capacity, efficiency or a torque-speed curve.

For now, `config/electrical.yaml` explicitly treats 25 N·m as a **provisional
per-wheel output-shaft torque cap**. Confirm this before calling the model
hardware-calibrated. If it is motor-shaft torque instead, output torque depends
on gear ratio and efficiency: `T_wheel = T_motor × ratio × efficiency`.
The three batteries are recorded as metadata with an unconstrained-supply
assumption, as requested. Their mass/distribution, discharge, current limits,
voltage sag, heat, regenerative charging and energy consumption are not modeled.
There is no assumed two-motors-per-battery wiring arrangement.

## 2. Implemented motor behavior

The default simulation now uses `rover_simulation::TorqueDrive` instead of the
Gazebo DiffDrive velocity controller. Each of the six wheel joints gets its own
bounded torque from a proportional speed controller. The target speeds are
computed from `/cmd_vel`, track and wheel radius; acceleration ramps, a speed
target cap and a command timeout remain in place. Speed gain 5 is controller
tuning, **not** a motor torque constant. The existing 12 rad/s target limit is
not inferred from the 25 N·m specification.

The force actuator interface is documented in the
[Gazebo Joint API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1Joint.html).
No competing velocity-drive plugin is loaded in this mode. The passive
suspension differential remains separate and unchanged.

At zero speed request the controller actively brakes, bounded by the same cap.
This is an assumed control behavior, not a verified ESC braking/regeneration mode.
At a given speed, mechanical shaft power is `P = torque × angular_velocity`.
It is **not battery electrical power**; that needs efficiencies and losses.
Similarly, torque divided by radius gives a force demand, not guaranteed
traction. Contact friction and load distribution can cause slip first.

`/drive/motor_status` (`std_msgs/String`, JSON, ~20 simulation Hz) reports:

- Joint order: front/middle/rear left, then front/middle/rear right.
- Per-wheel measured angular speed, applied torque command and shaft power.
- Total positive motoring and negative/braking mechanical power, separately.
- The explicit unconstrained-battery-model assumption.

The reported torque is the commanded actuator torque, not a hardware sensor or
the sum of contact/constraint forces. No battery charge estimate is fabricated.

## 3. Honeycomb wheel: what changed and what has not

`scripts/prepare_wheel_inertia.py` integrates volume and second moments over the
isolated original wheel shell, so voids contribute no material. The result uses
the supplied 3.694552 kg wheel mass plus the 0.054917 kg coupler. It assumes uniform
wheel material density and a centered cylindrical coupler; it is not a composite
hub/tread material model. Small COM offsets (about 0.3 mm laterally) are neglected
when centering the wheel inertial origin. The representative tensor is reused
across the six nominally identical wheels; legacy wheel masses scale it.

Recovered volume is 0.00318897 m³ versus the Fusion screenshot's 0.003187 m³
(~0.062% difference). Approximate assembled inertia at the wheel COM is:
Ixx = 0.026296, Iyy = 0.038922, Izz = 0.026296 kg·m²; the axle is Y.
The complete tensor and assumptions are in `config/wheel_inertia.yaml`.

There are **159 non-manifold edges** in the source wheel shell. Its oriented
boundary contributions cancel and the integrated volume matches CAD closely,
but that does not make it a clean finite-element mesh. Review touching or
overlapping honeycomb junctions before deformable analysis.

Visual honeycomb geometry is retained. **Contact remains a rigid cylinder.**
Cell buckling, tread compliance, impact absorption and a changing contact patch
have not been implemented or validated. Merely using the detailed STL as a
collision mesh would not make it elastic, and could increase simulation cost.

First establish whether the wheel is metal, TPU, rubber, or a multi-material
assembly. For deformable-wheel calibration, obtain material properties, cell
wall dimensions, tread/hub construction, and load-versus-deflection measurements
(including loading/unloading hysteresis and an impact/rebound test if possible).
Then fit a reduced compliant wheel/contact model for Gazebo and compare it with
bench data. A detailed offline structural model may be needed for cell stress
or failure; do not infer these from appearance alone.

## 4. Arm rest pose and CG

The existing arm is a two-axis visual/physics proxy. Your actual **six-DOF arm is
not yet reconstructed**. A physically meaningful rest pose requires all six link
meshes, joint axes/origins/limits, link masses/COMs, motor/gear masses and payload.

The next implementation should calculate whole-rover COM using
`r_COM = sum(m_i × r_i) / sum(m_i)` for candidate configurations. Search for a
folded pose with low height and good projected COM margin inside the **actual
contact support polygon**, subject to collision/joint constraints. Compare level
ground and slopes, with and without payload. Moving an arm generally moves COM;
there is no justified single pose that makes it invariant. Do not relabel the
current proxy pose as a validated six-axis stow pose.

## 5. Blender work plan (installed version: 5.2.2 LTS)

Blender is the asset/inspection/visualization stage; Gazebo remains the ROS
controls and runtime physics stage. No Blender scene has been created in this
turn—this section is the requested plan.

### A. Clean master assembly

Import the reassembled metre-scale package meshes into separate collections:
chassis, rockers, bogies, wheel hubs/treads, arm, sensors and terrain. Keep an
untouched high-detail source collection. Do not rescale package meshes by 0.001;
only original millimetre CAD exports require that conversion. Verify the nominal
300 mm wheel diameter and 500 mm axle spacing. Set origins to recovered pivots,
check normals, duplicate faces and non-manifold cell intersections. Preserve
ROS X-forward/Z-up on export and check a known point after round-tripping.
[Blender STL import/export reference](https://docs.blender.org/manual/nl/5.2/files/import_export/stl.html).

**Deliverable:** a named, correctly scaled master `.blend` assembly plus a mesh
repair report. Do not remove honeycomb walls merely to silence mesh checks.

### B. Separate display meshes from collision meshes

Create reduced visual meshes while preserving wheel silhouette, tread and joint
interfaces; use baked detail for small bolts. Blender's
[Decimate modifier](https://docs.blender.org/manual/en/5.2/modeling/modifiers/generate/decimate.html)
supports polygon reduction. Build simple, separate collision proxies; optionally
test a compound outer-tread approximation against the existing cylinder. Do not
simulate every honeycomb strut in the full six-wheel real-time model initially.

**Acceptance:** dimensions/pivots unchanged; visual quality checked close-up;
contact results and Gazebo real-time factor compared before/after. Never compute
new mass/inertia from a decimated render mesh without quantifying the change.

### C. Wheel deformation study

Build a one-wheel bench scene first: fixed hub, ground/platen, known vertical
loads and a small obstacle. Blender's soft-body system uses spring-like edge
behavior, useful for deformation visualization and qualitative experiments.
[Soft-body settings](https://docs.blender.org/manual/en/latest/physics/soft_body/settings/edges.html).
Its slider values should not be treated as measured material properties.
Use bench data or validated structural analysis to derive the Gazebo compliance
parameters. Deformed mesh animation must not be mistaken for validated traction.

**Deliverable:** comparison plots of force/deflection/contact footprint and a
documented reduced-model fit, not just a convincing animation.

### D. Arm packaging and presentation

Once the real six-link arm data is available, create its hierarchy with joint
origins, inspect folded clearances and visualize the calculated whole-rover COM.
Replay recorded Gazebo joint states for videos of rocker/bogie articulation,
wheel loads and motor saturation. Add terrain materials, lighting, dust visuals
and camera paths for presentation; keep cosmetic effects out of the physics
model unless separately calibrated. Export validated low-cost assets back to
`rover_description`, rebuild, then repeat the same terrain tests.

## 6. ROS 2 package workflow and verification

`rover_simulation/src/torque_drive.cpp` is the Gazebo physics system;
`motor_law.hpp` and `test/test_motor.cpp` test its saturation/braking law.
`rover_description/config/electrical.yaml` stores known/unknown electrical data;
`wheel_inertia.yaml` stores the derived wheel tensor. Xacro loads both and sets
joint limits/inertias and plugin parameters. The bridge configurations expose
the new diagnostics topic. `test/test_electrical.py` checks plugin exclusivity,
limits, wheel inertia and explicit unknown values.

The graph remains joystick `/joy` → teleop `/cmd_vel` → bridge → Gazebo torque
controller → six wheel joints. Gazebo returns `/joint_states` and `/odom`, and
the bridge now also returns `/drive/motor_status`. Existing TF and passive
suspension are unchanged. No new ROS services/actions are introduced. The motor
loop executes at the physics timestep, not through a ROS timer. Parameters are
loaded on launch; edit YAML and restart to change them.

```bash
cd /home/inian/ROS2/URDF_IRC
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to rover_description rover_bringup
source install/setup.bash
colcon test --packages-select rover_simulation rover_bringup
colcon test-result --verbose
ros2 launch rover_bringup irc_offroad.launch.py gpu:=nvidia
```

In a second sourced terminal:

```bash
ros2 topic echo /drive/motor_status --once
ros2 topic echo /joint_states --once
ros2 topic echo /teleop/status --once
```

Check all six torque commands stay within ±25 N·m, turning signs are correct,
and input loss stops the rover. The plain URDF effort tag alone is not the
validation: verify the running force controller and its telemetry. Saturation
tests check the exact ±25 bound; terrain tests cover lower, naturally occurring
loads. Earlier speed-driven terrain results are historical, not validation of
the new electrical model. A full torque-speed/current/battery simulation and a
compliant honeycomb wheel remain subsequent calibration work.

Recorded initial force-drive trial: 8.78 m over the southern hill, 38 simulation
seconds, maximum differential sum error 0.00385 rad. A 237-message telemetry
sample observed peak absolute torque 3.014 N·m and separated motoring/braking
power; that sample does not demonstrate full-load motor performance. Results:
`docs/electrical_drive_validation.json`, `docs/motor_diagnostics_validation.json`.

Final validation: 24 package tests passed. An isolated runtime watchdog test
published motion for 3 wall-clock seconds, then stopped publishing for 4 seconds.
Wheel speed fell from a moving peak of 2.278 rad/s to below 0.05 rad/s (effectively
zero in the recorded sample), with torque commands remaining bounded. See
`docs/motor_watchdog_validation.json`. All temporary test simulations were closed.

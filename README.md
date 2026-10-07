# IRC rover ROS 2 workspace

Latest physics update: [electrical limits, honeycomb wheel inertia and Blender
plan](ELECTRICAL_WHEELS_BLENDER_PLAN.md). Wheel drive now uses a provisional
25 N·m per-wheel torque cap; battery sufficiency remains an assumption and
elastic wheel deformation is not yet modeled.

The rover STL and supplied dimension screenshot have been converted into the
`rover_description` ROS 2 Jazzy package.

The [IRC 2026 implementation guide](IRC_2026_GUIDE.md) maps the saved rulebook
to executable mass/size checks, the latched ROS safety stop, competition task
props and the remaining hardware/CAD work that cannot be inferred from STL.

For **Gazebo-only off-road terrain with rocks, slopes and potholes**, use
`ros2 launch rover_bringup irc_offroad.launch.py gpu:=nvidia` after sourcing ROS
and the overlay. See [the off-road guide](IRC_OFFROAD_GUIDE.md). No RViz or SLAM
is started by this launch.

For **Cartographer + Gazebo maze + RViz + DualSense controls**, follow
[the Cartographer gamepad guide](CARTOGRAPHER_GUIDE.md).
Runtime results and screenshots are in [Cartographer validation](CARTOGRAPHER_VALIDATION.md).

The new [reverse rocker-bogie guide](SUSPENSION_GUIDE.md) covers the reassembled
suspension CAD, supplied component masses, passive differential physics and
terrain validation. Suspension is enabled by default; use `terrain:=true` for
slopes/pits or add `terrain_lane:=articulation` for offset wheel obstacles.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rover_bringup carto_gamepad.launch.py
```

Start with the [step-by-step walkthrough](src/rover_description/README.md).

- [Plain URDF](src/rover_description/urdf/rover.urdf)
- [Editable Xacro](src/rover_description/urdf/rover.urdf.xacro)
- [Physical assumptions and dimensions](src/rover_description/config/physical.yaml)
- [Validation results](VALIDATION.md)

Geometry is detailed; chassis mass, inertia, sensor mounts and contact parameters
remain provisional. Suspension component masses now use the Fusion screenshots,
and its links articulate with a compliant differential. The gamepad variant adds
an approximate two-axis arm. Mapping/navigation
were exercised in Gazebo; real hardware still needs its drivers and calibration.

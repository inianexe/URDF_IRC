# Cartographer maze and DualSense operation

See the workspace's `CARTOGRAPHER_GUIDE.md` for the complete lesson, controls,
ROS graph, map-saving workflow, source files and modeling assumptions.

Quick start after building and sourcing this workspace:

```bash
ros2 launch rover_bringup carto_gamepad.launch.py
```

The launch starts the image-derived maze in Gazebo, the IRC rover with an
approximate two-axis arm, Cartographer 2D, RViz, the SDL DualSense driver and
manual control. It does not start Nav2, AMCL or SLAM Toolbox.

The file `maps/maze_reference.yaml` is synthetic geometry ground truth and is
not Cartographer's measured map. `/map` is always built live from LiDAR scans.

# Validation — 2026-09-26

Environment: installed ROS 2 Jazzy, Gazebo Harmonic; runtime tests isolated with
`ROS_DOMAIN_ID=77` and `GZ_PARTITION=irc_rover_validation`.

- `rosdep install --from-paths src --ignore-src -r -y`: all dependencies satisfied.
- `colcon build --symlink-install`: package built successfully.
- Xacro expansion and `check_urdf`: valid tree, 10 links and 9 joints (6 continuous,
  3 fixed), rooted at `base_footprint`.
- `gz sdf -k` on simulation-expanded URDF: valid; expected extension warning for
  `gz_frame_id`. Sensor messages confirmed the intended frame names at runtime.
- Conversion preserved all 2,181,460 triangles across seven meshes. Wheel
  separation was spatial and approximate; no claim of CAD topology recovery.
- Headless Gazebo spawned the rover and bridged `/clock`, `/joint_states`,
  `/scan`, `/imu/data`, `/odom`, `/tf`, and `/cmd_vel`.
- Received all six wheel joint names, `odom -> base_footprint`, IMU messages in
  `imu_link`, and 720-ray scans in `lidar_link` with finite room/obstacle returns.
- SLAM Toolbox published a 237 x 237 occupancy grid at 0.05 m/cell in the test room.
- Nav2 reported all managed navigation nodes active. A goal at map (1.0, 0.0),
  yaw 0 returned action status 4 (SUCCEEDED), error_code 0. Reported odometry
  moved from approximately (0, 0) to (0.758, 0.026) m, within the configured
  0.25 m goal tolerance. This is an integration test, not a ground-truth accuracy
  measurement or rough-terrain validation.
- RViz rendered the full assembled rover and sensor TF successfully.

Not validated: real rover drivers, physical mass/COM, torque-limited motors,
suspension articulation, tire slip calibration, sensor mounting accuracy,
saved-map AMCL navigation, terrain climbing or hardware stopping performance.
The tested physics assumption is a six-wheel skid-steer rover with rigid
suspension. The supplied dimension drawing defines wheel diameter and spacing,
but does not resolve those remaining hardware parameters.

# Saved Cartographer output

`carto_maze_demo.pbstream`, `.yaml`, and `.pgm` were saved from live Cartographer
during the rover/gamepad integration test. They cover only the portion observed
during that short run, not the entire maze.

The complete synthetic reference is at
`src/rover_bringup/maps/maze_reference.yaml`; it is rasterized world geometry,
not a SLAM result, and uses a different coordinate origin.

Save your own result while mapping with:

```bash
ros2 run rover_bringup save_carto_map maps/my_maze
```

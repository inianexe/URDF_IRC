#!/usr/bin/env python3
"""Derive complete configuration from the installed Jazzy defaults.

Run deliberately after reviewing upgrades; generated YAML is committed as the
editable configuration, not regenerated during normal launch/build.
"""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory


def main():
    config = Path(__file__).resolve().parents[1] / 'config'
    nav = yaml.safe_load((Path(get_package_share_directory('nav2_bringup')) / 'params/nav2_params.yaml').read_text())
    for costmap in ('local_costmap', 'global_costmap'):
        p = nav[costmap][costmap]['ros__parameters']
        p.pop('robot_radius', None)
        p['robot_base_frame'] = 'base_footprint'
        p['footprint'] = '[[0.68, 0.51], [0.68, -0.51], [-0.68, -0.51], [-0.68, 0.51]]'
        p['footprint_padding'] = 0.02
        p['inflation_layer']['inflation_radius'] = 1.0
        if costmap == 'local_costmap':
            p['width'] = 5
            p['height'] = 5
            # Virtual lidar is above 1 m: use a 2D obstacle layer, not the default
            # TurtleBot voxel column (only 0.8 m tall).
            p['plugins'] = ['obstacle_layer', 'inflation_layer']
            p.pop('voxel_layer', None)
            p['obstacle_layer'] = {'plugin': 'nav2_costmap_2d::ObstacleLayer', 'enabled': True,
                'observation_sources': 'scan', 'scan': {'topic': '/scan', 'data_type': 'LaserScan',
                    'clearing': True, 'marking': True, 'max_obstacle_height': 2.0,
                    'obstacle_max_range': 15.0, 'obstacle_min_range': 0.15,
                    'raytrace_max_range': 20.0, 'raytrace_min_range': 0.15}}
    nav['bt_navigator']['ros__parameters']['robot_base_frame'] = 'base_footprint'
    p = nav['controller_server']['ros__parameters']['FollowPath']
    p.update(vx_max=0.3, vx_min=-0.15, vy_max=0.0, wz_max=0.5,
             ax_max=0.3, ax_min=-0.3, az_max=0.5, visualize=False)
    p['CostCritic']['consider_footprint'] = True
    p = nav['velocity_smoother']['ros__parameters']
    p.update(max_velocity=[0.3, 0.0, 0.5], min_velocity=[-0.15, 0.0, -0.5],
             max_accel=[0.3, 0.0, 0.5], max_decel=[-0.3, 0.0, -0.5])
    # Jazzy uses unstamped Twist for the supplied bridge and drive plugin.
    for node in ('controller_server', 'behavior_server', 'velocity_smoother', 'collision_monitor'):
        nav[node]['ros__parameters']['enable_stamped_cmd_vel'] = False
    p = nav['collision_monitor']['ros__parameters']
    p['base_frame_id'] = 'base_footprint'
    p = nav['behavior_server']['ros__parameters']
    p.update(robot_base_frame='base_footprint', max_rotational_vel=0.5,
             min_rotational_vel=0.1, rotational_acc_lim=0.5)
    (config / 'nav2.yaml').write_text('# Derived from installed ROS 2 Jazzy nav2_bringup defaults; tuned provisionally for this rover.\n' + yaml.safe_dump(nav, sort_keys=False))
    slam = yaml.safe_load((Path(get_package_share_directory('slam_toolbox')) / 'config/mapper_params_online_async.yaml').read_text())
    slam['slam_toolbox']['ros__parameters'].update(minimum_travel_distance=0.15,
         minimum_travel_heading=0.15, min_laser_range=0.15, max_laser_range=20.0)
    (config / 'slam.yaml').write_text(yaml.safe_dump(slam, sort_keys=False))


if __name__ == '__main__':
    main()

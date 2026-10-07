#!/usr/bin/env python3
"""Build matching SDF and reference occupancy map from an explicit image trace.

The reference map is synthetic ground truth, NEVER a Cartographer SLAM result.
No image texture is substituted for physical walls: every segment has collision.
"""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import numpy as np
import yaml


def homography(source, target):
    a, b = [], []
    for (x, y), (u, v) in zip(source, target):
        a += [[x, y, 1, 0, 0, 0, -u*x, -u*y], [0, 0, 0, x, y, 1, -v*x, -v*y]]
        b += [u, v]
    return np.append(np.linalg.solve(a, b), 1).reshape(3, 3)


def main():
    package = Path(__file__).resolve().parents[1]
    cfg = yaml.safe_load((package / 'config/maze_layout.yaml').read_text())
    width, height = cfg['size_m']
    corners = np.array(cfg['source_corners_px'], dtype=float)
    target = np.array([[-width/2, height/2], [width/2, height/2],
                       [width/2, -height/2], [-width/2, -height/2]])
    transform = homography(corners, target)

    def project(point):
        q = transform @ np.array([*point, 1.])
        return q[:2] / q[2]

    segments = [(target[i], target[(i+1) % 4]) for i in range(4)]
    segments += [(project(start), project(end)) for start, end in cfg['walls_px']]
    sdf = ET.Element('sdf', version='1.9')
    world = ET.SubElement(sdf, 'world', name='reference_maze')
    ET.SubElement(world, 'gravity').text = '0 0 -9.81'
    physics = ET.SubElement(world, 'physics', name='default', type='ignored')
    ET.SubElement(physics, 'max_step_size').text = '0.001'
    ET.SubElement(physics, 'real_time_factor').text = '1'
    for filename, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                           ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors'), ('imu', 'Imu')]:
        plugin = ET.SubElement(world, 'plugin', filename=f'gz-sim-{filename}-system', name=f'gz::sim::systems::{name}')
        if filename == 'sensors':
            ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    scene = ET.SubElement(world, 'scene')
    ET.SubElement(scene, 'ambient').text = '0.65 0.65 0.65 1'
    ET.SubElement(scene, 'background').text = '0.72 0.77 0.83 1'
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    ET.SubElement(light, 'pose').text = '0 0 15 0 0 0'
    ET.SubElement(light, 'direction').text = '-0.5 0.1 -0.9'
    ET.SubElement(light, 'diffuse').text = '0.8 0.8 0.8 1'
    ET.SubElement(light, 'cast_shadows').text = 'true'

    def box(name, center, dimensions, yaw=0, color='0.30 0.13 0.10 1'):
        model = ET.SubElement(world, 'model', name=name)
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = ' '.join(map(str, [*center, 0, 0, yaw]))
        link = ET.SubElement(model, 'link', name='link')
        for kind in ['collision', 'visual']:
            element = ET.SubElement(link, kind, name=kind)
            geom = ET.SubElement(element, 'geometry')
            ET.SubElement(ET.SubElement(geom, 'box'), 'size').text = ' '.join(map(str, dimensions))
            if kind == 'visual':
                material = ET.SubElement(element, 'material')
                ET.SubElement(material, 'ambient').text = color
                ET.SubElement(material, 'diffuse').text = color

    box('floor', [0, 0, -0.1], [width+2, height+2, 0.2], color='0.68 0.70 0.71 1')
    thickness, wall_height = cfg['wall_thickness'], cfg['wall_height']
    for i, (a, b) in enumerate(segments):
        d = b-a
        center = (a+b)/2
        box(f'wall_{i:02}', [*center, wall_height/2],
            [float(np.linalg.norm(d))+thickness, thickness, wall_height], math.atan2(d[1], d[0]))
    (package / 'worlds').mkdir(exist_ok=True)
    ET.indent(sdf)
    ET.ElementTree(sdf).write(package / 'worlds/reference_maze.sdf', encoding='utf-8', xml_declaration=True)

    # Rasterize exactly the same oriented collision boxes at cell centers.
    resolution = cfg['resolution']
    extent_x, extent_y = width+2, height+2
    nx, ny = int(round(extent_x/resolution)), int(round(extent_y/resolution))
    xs = (np.arange(nx)+0.5)*resolution - extent_x/2
    ys = extent_y/2 - (np.arange(ny)+0.5)*resolution
    xx, yy = np.meshgrid(xs, ys)
    raster = np.full((ny, nx), 205, dtype=np.uint8)
    raster[(abs(xx) < width/2) & (abs(yy) < height/2)] = 254
    for a, b in segments:
        d = b-a
        length = np.linalg.norm(d)
        direction = d/length
        cx, cy = (a+b)/2
        along = (xx-cx)*direction[0] + (yy-cy)*direction[1]
        across = -(xx-cx)*direction[1] + (yy-cy)*direction[0]
        raster[(abs(along) <= (length+thickness)/2) & (abs(across) <= thickness/2)] = 0
    maps = package / 'maps'
    maps.mkdir(exist_ok=True)
    with (maps / 'maze_reference.pgm').open('wb') as f:
        f.write(f'P5\n# Synthetic reference, not a SLAM result\n{nx} {ny}\n255\n'.encode())
        f.write(raster.tobytes())
    (maps / 'maze_reference.yaml').write_text(yaml.safe_dump({
        'image': 'maze_reference.pgm', 'mode': 'trinary', 'resolution': resolution,
        'origin': [-extent_x/2, -extent_y/2, 0.], 'negate': 0,
        'occupied_thresh': 0.65, 'free_thresh': 0.196}, sort_keys=False))
    spawn = project(cfg['spawn_image_px'])
    distances = []
    for a, b in segments:
        d = b-a
        closest = a + np.clip(np.dot(spawn-a, d)/np.dot(d, d), 0, 1)*d
        distances.append(float(np.linalg.norm(spawn-closest))-thickness/2)
    # Circumscribed body radius, including the original arm in its home pose.
    assert min(distances) > 0.90, f'Spawn too close to a wall: {min(distances)} m'
    (package / 'config/maze_spawn.yaml').write_text(yaml.safe_dump({
        'x': float(spawn[0]), 'y': float(spawn[1]), 'yaw': cfg['spawn_yaw'],
        'nearest_wall_clearance_m': min(distances), 'wall_count': len(segments)}, sort_keys=False))
    print(f'Built {len(segments)} walls; spawn {spawn}; clearance {min(distances):.3f} m')


if __name__ == '__main__':
    main()

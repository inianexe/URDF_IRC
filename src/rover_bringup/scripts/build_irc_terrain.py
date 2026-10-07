#!/usr/bin/env python3
"""Build a deterministic, scaled Sproscape-style Gazebo terrain.

The source brief is ``terrain.txt`` in the workspace root. Its 20,000 m² arena
and 500 m traverse are intentionally scaled to a 36 x 28 m dynamics course so
the mesh remains usable on a laptop. Soil deformation and dust are not modeled;
surface patches approximate them with friction and slip.
"""
from pathlib import Path
import json
import math
import struct
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull

PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = Path(__file__).resolve().parents[3]
SIZE = (36.0, 28.0)
SPAWN = (-15.0, -11.0, 0.0)
SEED = 20260927

# x, y, radius, depth. The largest crater is an explicit high-risk obstacle.
CRATERS = [
    (-7.0, 5.5, 1.35, .30), (1.0, 8.5, 1.75, .48),
    (11.5, -5.5, 2.20, .72), (13.0, 8.0, 2.80, 1.05),
]
PITS = CRATERS  # Compatibility for analysis tools written for the old course.

SURFACES = [
    {'name': 'hardpack_start', 'center': [-14.0, -10.5], 'size': [6.0, 5.0],
     'mu': 1.10, 'slip': .002, 'color': '.49 .34 .20 1'},
    {'name': 'gravel_delivery', 'center': [0.0, -10.0], 'size': [8.0, 4.0],
     'mu': .58, 'slip': .035, 'color': '.39 .37 .33 1'},
    {'name': 'loose_soil_abex', 'center': [-10.0, 2.0], 'size': [7.0, 4.0],
     'mu': .38, 'slip': .080, 'color': '.55 .40 .27 1'},
    {'name': 'servicing_pad', 'center': [-11.5, 9.5], 'size': [7.0, 4.0],
     'mu': 1.00, 'slip': .003, 'color': '.46 .39 .29 1'},
]

# Red terrain is a strict no-go cue in the rulebook. These disks are visual
# overlays only: crossing them is a driver/autonomy failure, not a wall impact.
NO_GO_AREAS = [
    {'name': 'red_no_go_alpha', 'center': [4.0, 8.0], 'radius': 1.0},
    {'name': 'red_no_go_bravo', 'center': [12.0, 2.5], 'radius': .8},
]


def bump(value, center, radius):
    """Cosine bump with zero value and gradient at its boundary."""
    t = abs(value-center)/radius
    return .5*(1+math.cos(math.pi*t)) if t < 1 else 0.


def patch_mask(x, y, patch, feather=.65):
    """One inside a surface patch, smoothly falling to zero outside it."""
    cx, cy = patch['center']; sx, sy = patch['size']
    dx = max(0., abs(x-cx)-sx/2)
    dy = max(0., abs(y-cy)-sy/2)
    if dx >= feather or dy >= feather:
        return 0.
    return .5*(1+math.cos(math.pi*dx/feather)) * .5*(1+math.cos(math.pi*dy/feather))


def height(x, y):
    """Continuous collision height: mounds, ridge, ruts, craters and gully."""
    rough = (.016*math.sin(1.9*x)*math.sin(1.4*y)
             + .010*math.sin(4.1*x+.7*y)
             + .006*math.cos(6.2*y-.4*x))
    # Max analytical centre-line grades are approximately 20, 27 and 30 deg.
    mound = .82*bump(x, -6., 3.7)*bump(y, -4., 4.2)
    ridge = .95*bump(x, 7.5, 4.4)*bump(y, 5.5, 3.0)
    cross_slope = .42*bump(x, -1.5, 4.0)*bump(y, 3.5, 4.5)
    gully = -.52*bump(x, 3.0, 1.55)*bump(y, 1.0, 9.0)
    ruts = -.075*bump(x, 7.0, 5.0)*(bump(y, -8.55, .25)+bump(y, -7.70, .25))
    craters = sum(-depth*bump(math.hypot(x-cx, y-cy), 0., radius)
                  for cx, cy, radius, depth in CRATERS)
    # A depression immediately beyond the ridge creates a sensor-hidden drop.
    blind_drop = -.42*bump(x, 11.2, 1.65)*bump(y, 5.5, 2.5)
    value = rough+mound+ridge+cross_slope+gully+ruts+craters+blind_drop
    # Contact patches are flat so a single collision material is unambiguous.
    flatten = max(patch_mask(x, y, patch) for patch in SURFACES)
    return value*(1-flatten)


def write_stl(path, triangles):
    with path.open('wb') as stream:
        stream.write(b'Scaled terrain.txt Sproscape course; metres'.ljust(80, b' '))
        stream.write(struct.pack('<I', len(triangles)))
        for triangle in triangles:
            a, b, c = np.asarray(triangle, dtype=float)
            normal = np.cross(b-a, c-a)
            length = np.linalg.norm(normal)
            if not length:
                raise ValueError(f'Degenerate terrain triangle: {triangle}')
            normal /= length
            stream.write(struct.pack('<12fH', *normal, *a, *b, *c, 0))


def make_ground():
    # 0.15 m grid resolves wheel-scale ruts without the cost of a 20,000 m² mesh.
    xs = np.linspace(-SIZE[0]/2, SIZE[0]/2, 241)
    ys = np.linspace(-SIZE[1]/2, SIZE[1]/2, 188)
    triangles = []

    def quad(a, b, c, d):
        triangles.extend([(a, b, c), (a, c, d)])

    for x0, x1 in zip(xs, xs[1:]):
        for y0, y1 in zip(ys, ys[1:]):
            quad((x0, y0, height(x0, y0)), (x1, y0, height(x1, y0)),
                 (x1, y1, height(x1, y1)), (x0, y1, height(x0, y1)))
    perimeter = ([(x, ys[0]) for x in xs]+[(xs[-1], y) for y in ys[1:]]
                 + [(x, ys[-1]) for x in reversed(xs[:-1])]
                 + [(xs[0], y) for y in reversed(ys[1:-1])])
    bottom = -1.5
    for (x0, y0), (x1, y1) in zip(perimeter, perimeter[1:]+perimeter[:1]):
        quad((x0, y0, height(x0, y0)), (x0, y0, bottom),
             (x1, y1, bottom), (x1, y1, height(x1, y1)))
        triangles.append(((0, 0, bottom), (x1, y1, bottom), (x0, y0, bottom)))
    write_stl(PACKAGE/'worlds/irc_ground.stl', triangles)
    return len(triangles)


def add_model(world, name, pose, geometry, color, friction=.8, slip=0., collision=True):
    element = ET.SubElement(world, 'model', name=name)
    ET.SubElement(element, 'static').text = 'true'
    ET.SubElement(element, 'pose').text = ' '.join(map(str, pose))
    link = ET.SubElement(element, 'link', name='link')
    kinds = ['visual'] if not collision else ['collision', 'visual']
    for kind in kinds:
        child = ET.SubElement(link, kind, name=kind)
        shape = ET.SubElement(child, 'geometry')
        if geometry[0] == 'mesh':
            mesh = ET.SubElement(shape, 'mesh')
            ET.SubElement(mesh, 'uri').text = geometry[1]
            ET.SubElement(mesh, 'scale').text = ' '.join(map(str, geometry[2]))
        elif geometry[0] == 'cylinder':
            cylinder = ET.SubElement(shape, 'cylinder')
            ET.SubElement(cylinder, 'radius').text = str(geometry[1][0])
            ET.SubElement(cylinder, 'length').text = str(geometry[1][1])
        else:
            ET.SubElement(ET.SubElement(shape, 'box'), 'size').text = ' '.join(map(str, geometry[1]))
        if kind == 'visual':
            material = ET.SubElement(child, 'material')
            ET.SubElement(material, 'ambient').text = color
            ET.SubElement(material, 'diffuse').text = color
        else:
            ode = ET.SubElement(ET.SubElement(ET.SubElement(child, 'surface'), 'friction'), 'ode')
            ET.SubElement(ode, 'mu').text = str(friction)
            ET.SubElement(ode, 'mu2').text = str(friction)
            ET.SubElement(ode, 'slip1').text = str(slip)
            ET.SubElement(ode, 'slip2').text = str(slip)


def add_box_shape(link, kind, name, size, pose, color=None):
    """Add one locally posed box collision or visual to an existing link."""
    element = ET.SubElement(link, kind, name=name)
    ET.SubElement(element, 'pose').text = ' '.join(map(str, pose))
    geometry = ET.SubElement(element, 'geometry')
    ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = ' '.join(map(str, size))
    if color is not None:
        material = ET.SubElement(element, 'material')
        ET.SubElement(material, 'ambient').text = color
        ET.SubElement(material, 'diffuse').text = color


def add_cylinder_shape(link, kind, name, radius, length, pose, color=None):
    element = ET.SubElement(link, kind, name=name)
    ET.SubElement(element, 'pose').text = ' '.join(map(str, pose))
    geometry = ET.SubElement(element, 'geometry')
    cylinder = ET.SubElement(geometry, 'cylinder')
    ET.SubElement(cylinder, 'radius').text = str(radius)
    ET.SubElement(cylinder, 'length').text = str(length)
    if color is not None:
        material = ET.SubElement(element, 'material')
        ET.SubElement(material, 'ambient').text = color
        ET.SubElement(material, 'diffuse').text = color


def add_idmo_panel(world, x, y):
    """A 1.40 m-high approach/vision target with representative controls."""
    model = ET.SubElement(world, 'model', name='idmo_control_panel')
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{x} {y} {height(x, y)} 0 0 0'
    link = ET.SubElement(model, 'link', name='panel')
    add_box_shape(link, 'collision', 'panel_collision', [.60, .10, 1.40], [0, 0, .70, 0, 0, 0])
    add_box_shape(link, 'visual', 'panel_visual', [.60, .10, 1.40], [0, 0, .70, 0, 0, 0], '.56 .58 .60 1')
    # Drawer, button, rotary knob and three-pin socket are perception targets.
    add_box_shape(link, 'visual', 'drawer', [.42, .03, .22], [0, -.066, 1.08, 0, 0, 0], '.18 .20 .22 1')
    add_cylinder_shape(link, 'visual', 'button', .035, .035, [-.18, -.075, .75, math.pi/2, 0, 0], '.92 .58 .05 1')
    add_cylinder_shape(link, 'visual', 'knob', .045, .05, [.04, -.085, .75, math.pi/2, 0, 0], '.08 .12 .18 1')
    for i, px in enumerate((.16, .22, .28)):
        add_cylinder_shape(link, 'visual', f'socket_pin_{i}', .012, .02,
                           [px, -.07, .45 + (.04 if i == 1 else 0), math.pi/2, 0, 0], '.08 .08 .08 1')


def add_cache(world, x, y):
    """Dynamic 4.5 kg cache with a 12 cm x 4 cm rule-compliant handle."""
    model = ET.SubElement(world, 'model', name='idmo_cache')
    ET.SubElement(model, 'static').text = 'false'
    ET.SubElement(model, 'pose').text = f'{x} {y} {height(x, y)+.01} 0 0 0'
    link = ET.SubElement(model, 'link', name='cache')
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'pose').text = '0 0 .06 0 0 0'
    ET.SubElement(inertial, 'mass').text = '4.5'
    inertia = ET.SubElement(inertial, 'inertia')
    # Box inertia for the dominant 0.22 x 0.12 x 0.12 m cache body.
    for key, value in [('ixx', .0108), ('iyy', .02355), ('izz', .02355),
                       ('ixy', 0), ('ixz', 0), ('iyz', 0)]:
        ET.SubElement(inertia, key).text = str(value)
    add_box_shape(link, 'collision', 'body_collision', [.22, .12, .12], [0, 0, .06, 0, 0, 0])
    add_box_shape(link, 'visual', 'body_visual', [.22, .12, .12], [0, 0, .06, 0, 0, 0], '.12 .32 .72 1')
    add_cylinder_shape(link, 'collision', 'handle_collision', .02, .12, [0, 0, .15, 0, math.pi/2, 0])
    add_cylinder_shape(link, 'visual', 'handle_visual', .02, .12, [0, 0, .15, 0, math.pi/2, 0], '.08 .08 .10 1')


def make_rock_meshes(rng):
    for i in range(8):
        points = rng.normal(size=(24, 3))
        points /= np.linalg.norm(points, axis=1)[:, None]
        points *= rng.uniform(.72, 1., size=(24, 1))
        hull = ConvexHull(points); faces = []
        for face, equation in zip(hull.simplices, hull.equations):
            triangle = points[face]
            if np.dot(np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0]), equation[:3]) < 0:
                triangle = triangle[[0, 2, 1]]
            faces.append(triangle)
        write_stl(PACKAGE/f'worlds/irc_rock_{i}.stl', faces)


def main():
    if not (WORKSPACE/'terrain.txt').is_file():
        raise FileNotFoundError('Expected workspace terrain.txt source brief')
    triangle_count = make_ground()
    rng = np.random.default_rng(SEED)
    make_rock_meshes(rng)

    sdf = ET.Element('sdf', version='1.9')
    world = ET.SubElement(sdf, 'world', name='irc_offroad')
    ET.SubElement(world, 'gravity').text = '0 0 -9.81'
    physics = ET.SubElement(world, 'physics', name='default', type='ignored')
    ET.SubElement(physics, 'max_step_size').text = '.001'
    ET.SubElement(physics, 'real_time_factor').text = '1'
    for filename, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                           ('scene-broadcaster', 'SceneBroadcaster'), ('imu', 'Imu')]:
        ET.SubElement(world, 'plugin', filename=f'gz-sim-{filename}-system',
                      name=f'gz::sim::systems::{name}')
    scene = ET.SubElement(world, 'scene')
    ET.SubElement(scene, 'ambient').text = '.60 .57 .51 1'
    ET.SubElement(scene, 'background').text = '.65 .76 .86 1'
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    ET.SubElement(light, 'pose').text = '0 0 20 0 0 0'
    ET.SubElement(light, 'direction').text = '-.6 .3 -.8'
    ET.SubElement(light, 'diffuse').text = '.95 .88 .73 1'
    ET.SubElement(light, 'cast_shadows').text = 'true'

    add_model(world, 'earth', [0]*6, ('mesh', 'irc_ground.stl', [1, 1, 1]),
              '.48 .34 .20 1', friction=.82, slip=.008)

    # Thin, non-overlapping contact layers approximate the brief's soil changes.
    for patch in SURFACES:
        cx, cy = patch['center']; sx, sy = patch['size']
        # Keep the contact face 5 mm above the base mesh. This avoids coplanar
        # collision ambiguity while remaining a negligible terrain step.
        add_model(world, patch['name'], [cx, cy, -.020, 0, 0, 0],
                  ('box', [sx, sy, .05]), patch['color'], patch['mu'], patch['slip'])

    rocks = []
    # Three populations cover 5-10 cm cobbles, 10-30 cm stones and 30-60 cm boulders.
    populations = [(55, .025, .05), (45, .05, .15), (24, .15, .30)]
    for count, low, high in populations:
        for _ in range(count):
            for _attempt in range(500):
                x, y = rng.uniform([-16.5, -12.5], [16.5, 12.5])
                radius = float(rng.uniform(low, high))
                if math.hypot(x-SPAWN[0], y-SPAWN[1]) < 3.0:
                    continue
                if any(math.hypot(x-cx, y-cy) < crater_radius+.5
                       for cx, cy, crater_radius, _depth in CRATERS):
                    continue
                if any(math.hypot(x-r['x'], y-r['y']) < radius+r['radius']+.08 for r in rocks):
                    continue
                break
            else:
                raise RuntimeError('No collision-free rock placement found')
            rx = radius*float(rng.uniform(.75, 1.15))
            ry = radius*float(rng.uniform(.75, 1.15))
            rz = radius*float(rng.uniform(.60, 1.05))
            yaw = float(rng.uniform(-math.pi, math.pi))
            z = height(x, y)+rz*.72
            tone = float(rng.uniform(.28, .50))
            add_model(world, f'rock_{len(rocks):03}', [x, y, z, 0, 0, yaw],
                      ('mesh', f'irc_rock_{len(rocks)%8}.stl', [rx, ry, rz]),
                      f'{tone} {tone*.92} {tone*.78} 1', friction=.92)
            rocks.append({'x': float(x), 'y': float(y), 'z': z, 'radius': radius,
                          'diameter_m': 2*radius, 'scale': [rx, ry, rz]})

    # Zone posts make the four mission regions legible without RViz.
    zone_markers = [
        ('autonomous', [10., -10.5], '.95 .58 .08 1'),
        ('astrobiology', [-14., 4.5], '.18 .70 .34 1'),
        ('delivery', [4.8, -11.5], '.14 .45 .90 1'),
        ('servicing', [-15., 9.5], '.74 .30 .82 1'),
    ]
    for name, (x, y), color in zone_markers:
        add_model(world, f'{name}_marker', [x, y, height(x, y)+.75, 0, 0, 0],
                  ('cylinder', [.07, 1.5]), color)

    # Explicit IRC task cues: red no-go areas, a physical cache and an IDMO
    # panel whose top is below the rulebook's 1.5 m maximum.
    for area in NO_GO_AREAS:
        x, y = area['center']
        add_model(world, area['name'], [x, y, height(x, y)+.008, 0, 0, 0],
                  ('cylinder', [area['radius'], .015]), '.92 .02 .02 .72', collision=False)
    add_cache(world, 2.5, -10.0)
    add_idmo_panel(world, -12.0, 9.5)
    add_model(world, 'abex_sampling_target', [-10.0, 2.0, .02, 0, 0, 0],
              ('cylinder', [.18, .04]), '.10 .70 .30 1')

    # Perimeter barriers protect the finite collision mesh edges.
    for name, pose, size in [
        ('north', [0, 13.7, .35, 0, 0, 0], [35.6, .2, 1.0]),
        ('south', [0, -13.7, .35, 0, 0, 0], [35.6, .2, 1.0]),
        ('east', [17.7, 0, .35, 0, 0, 0], [.2, 27.6, 1.0]),
        ('west', [-17.7, 0, .35, 0, 0, 0], [.2, 27.6, 1.0]),
    ]:
        add_model(world, name, pose, ('box', size), '.30 .26 .20 1')

    ET.indent(sdf)
    ET.ElementTree(sdf).write(PACKAGE/'worlds/irc_offroad.sdf', encoding='utf-8', xml_declaration=True)
    manifest = {
        'source': 'terrain.txt',
        'source_context': {'arena_area_m2': 20000, 'traverse_radius_m': 500},
        'simulation_scale_note': 'Feature-complete 36x28 m dynamics course; not a surveyed competition map',
        'seed': SEED, 'size_m': list(SIZE), 'spawn': list(SPAWN),
        'ground_triangles': triangle_count, 'craters': CRATERS,
        'formations': {
            'mounds_and_ridges': 'approximately 20-30 degree maximum centre-line grades',
            'gully': {'center_x': 3.0, 'depth_m': .52},
            'blind_drop': {'center': [11.2, 5.5], 'depth_m': .42},
            'wheel_ruts_depth_m': .075,
        },
        'surface_zones': SURFACES,
        'surface_model_limit': 'Rigid contact approximation; no soil sinkage, displacement, or dust',
        'mission_zones': ['autonomous', 'astrobiology', 'delivery', 'servicing'],
        'no_go_areas': NO_GO_AREAS,
        'task_props': {
            'idmo_panel': {'top_height_m': 1.40, 'maximum_allowed_m': 1.50,
                           'controls': ['drawer', 'button', 'knob', 'three_pin_socket']},
            'idmo_cache': {'mass_kg': 4.5, 'largest_dimension_m': .22,
                           'handle_length_m': .12, 'handle_diameter_m': .04},
            'abex_sampling_target': {'required_depth_m': .10,
                                     'required_sample_mass_kg': .010},
        },
        'rocks': rocks,
    }
    (PACKAGE/'config/irc_terrain_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    gui = (PACKAGE/'config/maze_gui.config').read_text().replace(
        '<camera_pose>-13 -19 24 0 0.806 0.970</camera_pose>',
        '<camera_pose>-23 -24 22 0 0.68 0.68</camera_pose>')
    (PACKAGE/'config/irc_gui.config').write_text(gui)
    print(f'Built terrain.txt course: {SIZE[0]:.0f}x{SIZE[1]:.0f} m, '
          f'{triangle_count} ground triangles, {len(rocks)} rocks, {len(CRATERS)} craters')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Fail when the modeled rover violates measurable IRC 2026 hard limits.

Mass comes from the expanded URDF. Length and breadth are conservative visual
mesh envelopes sampled at the endpoints and zero of every suspension/arm joint.
This validates the digital model only; the competition rover must still be
weighed and measured in every fielded configuration.
"""
from itertools import product
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
XACRO = PACKAGE / 'urdf/rover.urdf.xacro'
PHYSICAL = PACKAGE / 'config/physical.yaml'
RULES = PACKAGE / 'config/irc_2026.yaml'
STL_DTYPE = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)),
                      ('attribute', '<u2')])


def xyz(text):
    return np.array([float(v) for v in (text or '0 0 0').split()])


def rotation_rpy(rpy):
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp, cp*sr, cp*cr],
    ])


def transform(origin=None):
    matrix = np.eye(4)
    if origin is not None:
        matrix[:3, :3] = rotation_rpy(xyz(origin.get('rpy')))
        matrix[:3, 3] = xyz(origin.get('xyz'))
    return matrix


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    c, s, q = np.cos(angle), np.sin(angle), 1-np.cos(angle)
    matrix = np.eye(4)
    matrix[:3, :3] = [[c+x*x*q, x*y*q-z*s, x*z*q+y*s],
                      [y*x*q+z*s, c+y*y*q, y*z*q-x*s],
                      [z*x*q-y*s, z*y*q+x*s, c+z*z*q]]
    return matrix


def corners(low, high):
    return np.array([[x, y, z, 1.] for x in (low[0], high[0])
                     for y in (low[1], high[1]) for z in (low[2], high[2])])


def visual_bounds(visual, mesh_cache):
    geometry = visual.find('geometry')
    mesh = geometry.find('mesh')
    if mesh is not None:
        uri = mesh.attrib['filename']
        prefix = 'package://rover_description/'
        if not uri.startswith(prefix):
            raise ValueError(f'Unsupported mesh URI: {uri}')
        path = PACKAGE / uri[len(prefix):]
        if path not in mesh_cache:
            triangles = np.memmap(path, mode='r', offset=84, dtype=STL_DTYPE)
            vertices = triangles['vertices']
            mesh_cache[path] = (vertices.min(axis=(0, 1)), vertices.max(axis=(0, 1)))
        low, high = (v.copy() for v in mesh_cache[path])
        scale = xyz(mesh.attrib.get('scale', '1 1 1'))
        return low*scale, high*scale
    box = geometry.find('box')
    if box is not None:
        half = xyz(box.attrib['size'])/2
        return -half, half
    cylinder = geometry.find('cylinder')
    if cylinder is not None:
        radius = float(cylinder.attrib['radius'])
        half = np.array([radius, radius, float(cylinder.attrib['length'])/2])
        return -half, half
    raise ValueError('Unsupported visual geometry')


def expand(arm):
    return ET.fromstring(subprocess.check_output([
        'xacro', str(XACRO), f'physical_config:={PHYSICAL}',
        f'articulated_arm:={str(arm).lower()}', 'suspension:=true']))


def analyze(arm):
    root = expand(arm)
    joints = {j.find('child').attrib['link']: j for j in root.findall('joint')}
    sampled = []
    for joint in joints.values():
        if joint.attrib['type'] != 'revolute':
            continue
        limit = joint.find('limit')
        sampled.append((joint.attrib['name'], [float(limit.attrib['lower']), 0.,
                                               float(limit.attrib['upper'])]))

    links = {link.attrib['name']: link for link in root.findall('link')}
    mesh_cache = {}
    local_visuals = {}
    for name, link in links.items():
        local_visuals[name] = [(transform(v.find('origin').attrib if v.find('origin') is not None else None),
                                *visual_bounds(v, mesh_cache)) for v in link.findall('visual')]

    maximum = np.zeros(3)
    worst = [{}, {}, {}]
    sampled_names = [item[0] for item in sampled]
    samples = [item[1] for item in sampled]
    for values in product(*samples):
        angles = dict(zip(sampled_names, values))
        poses = {}

        def link_pose(name):
            if name in poses:
                return poses[name]
            joint = joints.get(name)
            if joint is None:
                poses[name] = np.eye(4)
                return poses[name]
            pose = link_pose(joint.find('parent').attrib['link'])
            origin = joint.find('origin')
            pose = pose @ transform(origin.attrib if origin is not None else None)
            if joint.attrib['type'] == 'revolute':
                pose = pose @ axis_rotation(xyz(joint.find('axis').attrib['xyz']),
                                            angles[joint.attrib['name']])
            poses[name] = pose
            return pose

        points = []
        for name, visuals in local_visuals.items():
            pose = link_pose(name)
            for local_pose, low, high in visuals:
                points.append((pose @ local_pose @ corners(low, high).T).T[:, :3])
        cloud = np.concatenate(points)
        extent = cloud.max(axis=0)-cloud.min(axis=0)
        for axis in range(3):
            if extent[axis] > maximum[axis]:
                maximum[axis] = extent[axis]
                worst[axis] = angles.copy()

    mass = sum(float(i.find('mass').attrib['value'])
               for i in root.findall('link/inertial'))
    return mass, maximum, len(list(product(*samples))), worst


def main():
    rules = yaml.safe_load(RULES.read_text())
    passed = True
    all_parts = 0.
    for arm in (False, True):
        mass, envelope, poses, worst = analyze(arm)
        all_parts = max(all_parts, mass)
        label = 'base+arm' if arm else 'mobile base'
        print(f'{label}: mass={mass:.3f} kg, maximum sampled envelope='
              f'{envelope[0]:.3f} x {envelope[1]:.3f} x {envelope[2]:.3f} m '
              f'over {poses} joint poses')
        print(f'  worst length pose: {worst[0]}')
        print(f'  worst breadth pose: {worst[1]}')
        passed &= mass <= rules['mass']['maximum_deployed_kg'] + 1e-9
        passed &= envelope[0] <= rules['size']['maximum_length_m'] + 1e-9
        passed &= envelope[1] <= rules['size']['maximum_breadth_m'] + 1e-9
    passed &= all_parts <= rules['mass']['maximum_all_fielded_parts_kg'] + 1e-9
    print(f'known fielded-parts total={all_parts:.3f} kg; '
          f"limits={rules['mass']['maximum_deployed_kg']:.1f} kg deployed, "
          f"{rules['mass']['maximum_all_fielded_parts_kg']:.1f} kg all parts")
    if not passed:
        print('IRC 2026 modeled compliance check: FAIL', file=sys.stderr)
        return 1
    print('IRC 2026 modeled compliance check: PASS')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

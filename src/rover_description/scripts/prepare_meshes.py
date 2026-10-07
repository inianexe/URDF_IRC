#!/usr/bin/env python3
"""Reproducible, approximate spatial split of this specific CAD export.

All triangles are preserved, exactly once. This is NOT semantic CAD separation:
wheel selection regions may include hub hardware. Original STL is never edited.
"""
import argparse
import json
from pathlib import Path
import struct
import numpy as np

DTYPE = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stl', type=Path)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1] / 'meshes')
    args = parser.parse_args()
    with args.stl.open('rb') as stream:
        stream.read(80)
        count, = struct.unpack('<I', stream.read(4))
    if args.stl.stat().st_size != 84 + count * 50:
        raise ValueError('Expected a binary STL with 50-byte triangle records')
    raw = np.memmap(args.stl, mode='r', offset=84, dtype=DTYPE, shape=(count,))
    centers = raw['vertices'].mean(axis=1)
    assigned = np.zeros(count, dtype=bool)
    args.output.mkdir(parents=True, exist_ok=True)
    report = {'source': args.stl.name, 'source_triangles': count,
              'units_assumption': 'millimetres', 'cad_to_ros': 'X=x, Y=-z, Z=y; translated and scaled by 0.001',
              'wheel_split': 'approximate spatial regions, not original CAD bodies', 'parts': {}}
    # Common base_link origin: mid axle longitudinal center, center between sides,
    # and 150 mm above the lowest CAD point. CAD +Y is up, CAD -Z is left.
    base_cad = np.array([-37.85843, -293.17752, -229.65])
    rotation = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])

    def save(name, mask, origin):
        part = np.array(raw[mask], copy=True)
        part['vertices'] = ((part['vertices'] - origin) @ rotation.T) * 0.001
        part['normal'] = part['normal'] @ rotation.T
        part['attribute'] = 0
        with (args.output / (name + '.stl')).open('wb') as out:
            out.write(b'rover_description: metres, ROS axes'.ljust(80, b' '))
            out.write(struct.pack('<I', len(part)))
            part.tofile(out)
        report['parts'][name] = {'triangles': len(part), 'cad_origin_mm': origin.tolist(),
                                'local_bounds_m': [part['vertices'].min(axis=(0, 1)).tolist(),
                                                   part['vertices'].max(axis=(0, 1)).tolist()]}

    for position, x in [('rear', -538.), ('middle', -38.), ('front', 462.)]:
        for side, z in [('left', -641.), ('right', 182.)]:
            mask = ((abs(centers[:, 0] - x) < 160) & (centers[:, 1] < -130)
                    & (abs(centers[:, 2] - z) < 85) & ~assigned)
            vertices = raw['vertices'][mask]
            origin = (vertices.min(axis=(0, 1)) + vertices.max(axis=(0, 1))) / 2
            save(position + '_' + side + '_wheel', mask, origin)
            assigned |= mask
    save('chassis', ~assigned, base_cad)
    assert sum(p['triangles'] for p in report['parts'].values()) == count
    (args.output / 'mesh_report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()

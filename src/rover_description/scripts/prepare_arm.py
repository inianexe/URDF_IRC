#!/usr/bin/env python3
"""Optional 2-DOF arm proxy: spatial cut above the CAD turntable.

Keeps the original chassis.stl intact. NOT a reconstruction of CAD joints.
At zero joint angles the two derived meshes exactly reproduce chassis.stl.
"""
from pathlib import Path
import json
import struct
import numpy as np


def main():
    folder = Path(__file__).resolve().parents[1] / 'meshes'
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    mesh = np.memmap(folder / 'chassis.stl', mode='r', offset=84, dtype=dtype)
    centers = mesh['vertices'].mean(axis=1)
    mask = (centers[:, 2] > 0.425) & (abs(centers[:, 1]) < 0.20)
    # Estimated yaw pivot on turntable, lift pivot 60 mm above it.
    pivot = np.array([0.22, -0.045, 0.46])
    report = {'method': 'approximate spatial cut, two-axis rigid upper arm proxy',
              'yaw_origin': [0.22, -0.045, 0.40], 'lift_origin_relative_to_yaw': [0., 0., 0.06],
              'source_triangles': len(mesh), 'parts': {}}
    for name, selected, origin in [('chassis_without_arm', ~mask, np.zeros(3)), ('arm_assembly', mask, pivot)]:
        part = np.array(mesh[selected], copy=True)
        part['vertices'] -= origin
        with (folder / (name + '.stl')).open('wb') as f:
            f.write(b'IRC rover approximate arm split, metres'.ljust(80, b' '))
            f.write(struct.pack('<I', len(part)))
            part.tofile(f)
        report['parts'][name] = {'triangles': len(part), 'origin': origin.tolist(),
                                'bounds': [part['vertices'].min(axis=(0, 1)).tolist(), part['vertices'].max(axis=(0, 1)).tolist()]}
    assert sum(p['triangles'] for p in report['parts'].values()) == len(mesh)
    (folder / 'arm_report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Generate a closed, low-poly collision terrain and a maze variant.

The original maze/map are preserved. Terrain heights are chosen test fixtures,
not surveyed ground. Run build_maze.py first if the maze layout changes.
"""
from pathlib import Path
import math
import struct
import xml.etree.ElementTree as ET


def height(x, y):
    # Two lanes: broad hill/dip and offset wheel obstacles for articulation.
    def tent(value, center, half_width):
        return max(0., 1. - abs(value-center)/half_width)
    broad = min(1., max(0., (-10.5-y)/.5))
    hill = .25 * tent(x, 20., 1.5)
    pit = -.18 * tent(x, 25., 1.2)
    # Separate left/right wheel tracks create unequal terrain heights.
    offset = (.12 * tent(x, 20., .8) * tent(y, -8.6, .35)
              - .10 * tent(x, 23., .8) * tent(y, -9.4, .35))
    return broad * (hill + pit) + offset


def main():
    package = Path(__file__).resolve().parents[1]
    # 0.1 m cells; top plus bottom and sides form a watertight mesh.
    xs = [16.+i*.1 for i in range(131)]
    ys = [-15.+i*.1 for i in range(81)]
    triangles = []
    def quad(a, b, c, d):
        triangles.extend([(a, b, c), (a, c, d)])
    for x0, x1 in zip(xs, xs[1:]):
        for y0, y1 in zip(ys, ys[1:]):
            quad((x0,y0,height(x0,y0)), (x1,y0,height(x1,y0)),
                 (x1,y1,height(x1,y1)), (x0,y1,height(x0,y1)))
            quad((x0,y0,-.5), (x0,y1,-.5), (x1,y1,-.5), (x1,y0,-.5))
    perimeter = ([(x,ys[0]) for x in xs] + [(xs[-1],y) for y in ys[1:]]
                 + [(x,ys[-1]) for x in reversed(xs[:-1])]
                 + [(xs[0],y) for y in reversed(ys[1:-1])])
    for (x0,y0), (x1,y1) in zip(perimeter, perimeter[1:]+perimeter[:1]):
        quad((x0,y0,height(x0,y0)), (x0,y0,-.5),
             (x1,y1,-.5), (x1,y1,height(x1,y1)))
    mesh = package / 'worlds/suspension_terrain.stl'
    with mesh.open('wb') as stream:
        stream.write(b'Generated suspension test terrain'.ljust(80,b'\0'))
        stream.write(struct.pack('<I',len(triangles)))
        for triangle in triangles:
            a,b,c = triangle
            u,v = [b[i]-a[i] for i in range(3)], [c[i]-a[i] for i in range(3)]
            n = [u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
            norm = math.sqrt(sum(q*q for q in n))
            stream.write(struct.pack('<12fH',*[q/norm for q in n],*a,*b,*c,0))
    tree = ET.parse(package / 'worlds/reference_maze.sdf')
    world = tree.getroot().find('world')
    # Keep the same world name for existing bridge/service compatibility.
    world.remove(world.find("model[@name='wall_01']"))
    def box(name, xyz, size):
        model = ET.SubElement(world,'model',name=name)
        ET.SubElement(model,'static').text='true'
        ET.SubElement(model,'pose').text=' '.join(map(str,[*xyz,0,0,0]))
        link = ET.SubElement(model,'link',name='link')
        for kind in ('collision','visual'):
            item=ET.SubElement(link,kind,name=kind)
            ET.SubElement(ET.SubElement(ET.SubElement(item,'geometry'),'box'),'size').text=' '.join(map(str,size))
    # Four-metre gate in the east maze wall; original floor reaches x=16.
    box('east_wall_north',[15,2, .9],[.2,26,1.8])
    box('terrain_north_rail',[22.5,-6.9,.3],[13.2,.2,.6])
    box('terrain_south_rail',[22.5,-15.1,.3],[13.2,.2,.6])
    box('terrain_end_rail',[29.1,-11,.3],[.2,8.4,.6])
    model=ET.SubElement(world,'model',name='suspension_test_ground')
    ET.SubElement(model,'static').text='true'
    link=ET.SubElement(model,'link',name='ground')
    for kind in ('collision','visual'):
        item=ET.SubElement(link,kind,name=kind)
        geom=ET.SubElement(item,'geometry')
        ET.SubElement(ET.SubElement(geom,'mesh'),'uri').text='suspension_terrain.stl'
        if kind == 'visual':
            material=ET.SubElement(item,'material')
            ET.SubElement(material,'ambient').text='.48 .40 .24 1'
            ET.SubElement(material,'diffuse').text='.48 .40 .24 1'
    ET.indent(tree)
    tree.write(package/'worlds/terrain_maze.sdf',encoding='utf-8',xml_declaration=True)
    gui=(package/'config/maze_gui.config').read_text()
    (package/'config/terrain_gui.config').write_text(gui.replace(
        '<camera_pose>-13 -19 24 0 0.806 0.970</camera_pose>',
        '<camera_pose>13 -21 10 0 0.639 0.837</camera_pose>'))
    print(f'Generated terrain variant: {len(triangles)} triangles; spawn (17,-12), yaw 0')


if __name__ == '__main__':
    main()

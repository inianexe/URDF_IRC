"""Check generated collision terrain, including genuine below-ground pits."""
import importlib.util
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('build_terrain', PACKAGE/'scripts/build_terrain.py')
terrain = importlib.util.module_from_spec(spec)
spec.loader.exec_module(terrain)


def test_terrain_heights_and_flat_entry():
    assert terrain.height(17, -12) == 0
    assert terrain.height(20, -12) == .25
    assert terrain.height(25, -12) == -.18
    assert terrain.height(20, -8.6) == .12
    assert terrain.height(23, -9.4) == -.10


def test_collision_mesh_is_closed_and_nondegenerate():
    data = (PACKAGE/'worlds/suspension_terrain.stl').read_bytes()
    count, = struct.unpack_from('<I', data, 80)
    assert len(data) == 84 + count*50
    edges = {}
    for i in range(count):
        values = struct.unpack_from('<12fH', data, 84+i*50)
        assert sum(v*v for v in values[:3]) > .99
        points = [tuple(values[j:j+3]) for j in (3,6,9)]
        for a,b in zip(points,points[1:]+points[:1]):
            key=tuple(sorted((a,b)))
            edges[key]=edges.get(key,0)+1
    assert all(n == 2 for n in edges.values())


def test_variant_preserves_original_and_has_collision():
    world=ET.parse(PACKAGE/'worlds/terrain_maze.sdf').getroot().find('world')
    assert world.find("model[@name='suspension_test_ground']/link/collision/geometry/mesh/uri") is not None
    floor=world.find("model[@name='floor']/link/collision/geometry/box/size").text
    assert float(floor.split()[0])/2 <= 16  # Cannot cover the pit at x=25.
    original=ET.parse(PACKAGE/'worlds/reference_maze.sdf').getroot().find('world')
    assert original.find("model[@name='wall_01']") is not None

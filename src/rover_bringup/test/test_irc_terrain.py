import importlib.util
import json
from pathlib import Path
import struct
import subprocess
import xml.etree.ElementTree as ET
import numpy as np

PACKAGE=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('build_irc',PACKAGE/'scripts/build_irc_terrain.py')
terrain=importlib.util.module_from_spec(spec);spec.loader.exec_module(terrain)


def test_flat_staging_real_craters_and_challenging_slopes():
    for x in [-15.5, -15., -14.5]:
        for y in [-11.5, -11., -10.5]:assert terrain.height(x,y)==0
    for x,y,r,d in terrain.CRATERS:
        rim=np.mean([terrain.height(x+r*np.cos(a),y+r*np.sin(a)) for a in np.linspace(0,2*np.pi,16)])
        assert rim-terrain.height(x,y)>d*.7
    assert terrain.height(-6,-4)>.75
    assert terrain.height(7.5,5.5)>.85
    # Numerical centre-line slope verifies the brief's 15-30 degree hill class.
    xs=np.linspace(-9.7,-2.3,150)
    grades=np.abs(np.gradient([terrain.height(x,-4) for x in xs],xs))
    assert np.degrees(np.arctan(grades.max()))>15


def test_collision_meshes_closed_and_finite():
    for path in (PACKAGE/'worlds').glob('irc_*.stl'):
        data=path.read_bytes();count,=struct.unpack_from('<I',data,80)
        assert len(data)==84+50*count
        edges={}
        for i in range(count):
            v=struct.unpack_from('<12fH',data,84+50*i)
            assert np.isfinite(v).all() and sum(t*t for t in v[:3])>.99
            points=[tuple(v[j:j+3]) for j in (3,6,9)]
            for a,b in zip(points,points[1:]+points[:1]):
                edge=tuple(sorted((a,b)));edges[edge]=edges.get(edge,0)+1
        assert all(n==2 for n in edges.values()),path.name


def test_world_obstacles_and_no_covering_floor():
    world=ET.parse(PACKAGE/'worlds/irc_offroad.sdf').getroot().find('world')
    models=world.findall('model')
    assert sum(m.attrib['name'].startswith('rock_') for m in models)==124
    assert world.find('.//plane') is None
    noncolliding={'red_no_go_alpha','red_no_go_bravo'}
    assert all(m.find('link/collision') is not None
               for m in models if m.attrib['name'] not in noncolliding)
    manifest=json.loads((PACKAGE/'config/irc_terrain_manifest.json').read_text())
    assert manifest['source']=='terrain.txt'
    assert manifest['size_m']==[36.,28.]
    assert len(manifest['surface_zones'])==4
    assert {z['name'] for z in manifest['surface_zones']}=={
        'hardpack_start','gravel_delivery','loose_soil_abex','servicing_pad'}
    assert all(np.hypot(r['x']+15,r['y']+11)>=3. for r in manifest['rocks'])


def test_surface_physics_and_mission_markers():
    world=ET.parse(PACKAGE/'worlds/irc_offroad.sdf').getroot().find('world')
    names={m.attrib['name'] for m in world.findall('model')}
    assert {'autonomous_marker','astrobiology_marker','delivery_marker','servicing_marker'} <= names
    assert {'red_no_go_alpha','red_no_go_bravo','idmo_cache',
            'idmo_control_panel','abex_sampling_target'} <= names
    for patch in terrain.SURFACES:
        model=world.find(f"model[@name='{patch['name']}']")
        ode=model.find('link/collision/surface/friction/ode')
        assert float(ode.findtext('mu'))==patch['mu']
        assert float(ode.findtext('slip1'))==patch['slip']

    manifest=json.loads((PACKAGE/'config/irc_terrain_manifest.json').read_text())
    props=manifest['task_props']
    assert props['idmo_panel']['top_height_m'] <= props['idmo_panel']['maximum_allowed_m']
    cache=props['idmo_cache']
    assert cache['mass_kg'] < 5 and cache['largest_dimension_m'] < .4
    assert cache['handle_length_m'] >= .1 and cache['handle_diameter_m'] <= .05
    assert len(manifest['no_go_areas']) == 2


def test_offroad_has_no_lidar_sensor():
    path=PACKAGE.parent/'rover_description/urdf/rover.urdf.xacro'
    root=ET.fromstring(subprocess.check_output(['xacro',str(path),'simulation:=true','lidar_enabled:=false']))
    assert not root.findall(".//sensor[@type='gpu_lidar']")
    assert root.findall(".//sensor[@type='imu']")

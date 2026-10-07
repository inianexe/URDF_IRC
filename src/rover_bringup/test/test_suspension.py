from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
import pytest

DESCRIPTION=Path(__file__).resolve().parents[2]/'rover_description'
STL_DTYPE=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])


@pytest.mark.parametrize('arm', ['true','false'])
@pytest.mark.parametrize('suspension', ['true','false'])
def test_model_tree_inertia_and_terrain_tf(arm,suspension):
    root=ET.fromstring(subprocess.check_output([
        'xacro',str(DESCRIPTION/'urdf/rover.urdf.xacro'),
        'simulation:=true','articulated_arm:='+arm,'suspension:='+suspension]))
    links={link.attrib['name'] for link in root.findall('link')}
    joints={joint.attrib['name']:joint for joint in root.findall('joint')}
    children=[j.find('child').attrib['link'] for j in joints.values()]
    assert len(children)==len(set(children))==len(links)-1
    for j in joints.values():
        assert j.find('parent').attrib['link'] in links
    for inertial in root.findall('link/inertial'):
        assert float(inertial.find('mass').attrib['value'])>0
        d={k:float(v) for k,v in inertial.find('inertia').attrib.items()}
        tensor=np.array([[d['ixx'],d['ixy'],d['ixz']],
                         [d['ixy'],d['iyy'],d['iyz']],
                         [d['ixz'],d['iyz'],d['izz']]])
        eig=np.linalg.eigvalsh(tensor)
        assert min(eig)>0 and eig[-1]<=sum(eig[:2])+1e-8
    total=sum(float(i.find('mass').attrib['value']) for i in root.findall('link/inertial'))
    expected=57.+(8. if arm=='true' else 0.)
    assert abs(total-expected)<1e-6
    assert total <= 65.0
    assert 'kill_switch_link' in links
    if suspension=='true':
        for side in ['left','right']:
            assert joints[f'front_{side}_wheel_joint'].find('parent').attrib['link']==f'{side}_bogie_link'
            assert joints[f'middle_{side}_wheel_joint'].find('parent').attrib['link']==f'{side}_bogie_link'
            assert joints[f'rear_{side}_wheel_joint'].find('parent').attrib['link']==f'{side}_rocker_link'
        plugins=root.findall('gazebo/plugin')
        assert any(p.attrib['filename']=='librover_differential.so' for p in plugins)
        assert sum(p.findtext('tf_topic')=='/tf' for p in plugins)==1
        assert any(p.findtext('dimensions')=='3' for p in plugins)
    else:
        assert 'left_rocker_joint' not in joints


def test_motor_output_hardware_faces_outboard_toward_wheels():
    """Catch the handedness error that made both motor flanges face inboard."""
    for kind in ['rocker','bogie']:
        for side,expected_sign in [('left',1),('right',-1)]:
            mesh=np.memmap(DESCRIPTION/'meshes'/f'{side}_{kind}.stl',
                           mode='r',offset=84,dtype=STL_DTYPE)
            centers=mesh['vertices'].mean(axis=1)
            wheel_region=((centers[:,0]<-.42)&(centers[:,2]<-.20)
                          if kind=='rocker' else
                          (abs(centers[:,0])>.18)&(centers[:,2]<-.08))
            y=centers[wheel_region,1]
            assert len(y)>1000
            # The flange/output shells must extend farther outboard than
            # inboard from the suspension link's wheel-axis centreline.
            if expected_sign>0:
                assert y.max()>.08 and y.min()>-.04
            else:
                assert y.min()<-.08 and y.max()<.04

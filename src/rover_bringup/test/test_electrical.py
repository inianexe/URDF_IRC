from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import yaml
import numpy as np

DESCRIPTION=Path(__file__).resolve().parents[2]/'rover_description'


def test_force_drive_replaces_velocity_drive():
    root=ET.fromstring(subprocess.check_output(['xacro',str(DESCRIPTION/'urdf/rover.urdf.xacro'),'simulation:=true']))
    plugins=root.findall('gazebo/plugin')
    motor=[p for p in plugins if p.attrib['filename']=='librover_torque_drive.so']
    assert len(motor)==1 and float(motor[0].findtext('torque_limit'))==25
    assert not any(p.attrib['filename']=='gz-sim-diff-drive-system' for p in plugins)
    assert sum(p.findtext('tf_topic')=='/tf' for p in plugins)==1
    for joint in root.findall('joint'):
        if '_wheel_joint' in joint.attrib['name']:assert float(joint.find('limit').attrib['effort'])==25


def test_honeycomb_inertia_and_explicit_unknowns():
    wheel=yaml.safe_load((DESCRIPTION/'config/wheel_inertia.yaml').read_text())
    assert abs(wheel['volume_m3']-.003187)/.003187<.005
    assert wheel['deformation_modeled'] is False
    ii=wheel['inertia'];tensor=np.array([[ii[0],ii[1],ii[2]],[ii[1],ii[3],ii[4]],[ii[2],ii[4],ii[5]]])
    ev=np.linalg.eigvalsh(tensor)
    assert ev.min()>0 and ev[-1]<sum(ev[:2])
    electrical=yaml.safe_load((DESCRIPTION/'config/electrical.yaml').read_text())
    assert electrical['wheel_motor_count']==6 and electrical['battery_count']==3
    assert electrical['voltage_v'] is None and electrical['gear_ratio'] is None
    assert electrical['command_timeout_seconds']==.8


def test_motor_watchdog_is_observable():
    source=(DESCRIPTION.parent/'rover_simulation/src/torque_drive.cpp').read_text()
    assert 'command_stale' in source and 'command_age_seconds' in source

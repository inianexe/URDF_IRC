#!/usr/bin/env python3
"""Reassemble this exploded CAD export into reverse rocker-bogie rigid links.

Run inspect_suspension.py first. Source STL and existing meshes stay untouched.
Masses are transcribed from the supplied Fusion property screenshots. Equivalent
component-box inertias are estimates, not the unexpanded CAD inertia tensors.
"""
from pathlib import Path
import json
import struct
import numpy as np
import yaml
from inspect_suspension import DTYPE, ROOT


def write_mesh(path, part):
    with path.open('wb') as f:
        f.write(b'Reassembled reverse rocker bogie: SI units'.ljust(80,b' '))
        f.write(struct.pack('<I',len(part)))
        part.tofile(f)


def main():
    pkg=ROOT/'src/rover_description'
    raw=np.memmap(ROOT/'INRB_suspension_load_cf_circular_right.stl',mode='r',offset=84,dtype=DTYPE)
    labels=np.load(ROOT/'docs/suspension_shell_labels.npy')
    # Ring centers recovered from coplanar circular mesh edges (mm).
    holder=np.array([-249.938,-144.446,20.])
    bogie_ring=np.array([-245.704,-117.099,18.254])
    shift=holder-bogie_ring
    rocker_pivot=np.array([0.,0.,20.])
    # +X rover points toward the bogie pair. The exploded source's motor output
    # hardware is handed opposite to the assembled wheel side, so the right
    # link needs a Y reflection and the left link retains the transformed hand.
    # This puts each output flange outboard, toward its wheel, on both sides.
    rotation=np.array([[-1.,0.,0.],[0.,0.,-1.],[0.,1.,0.]])
    wheel_mass=3.694552 + .054917
    groups={
        'rocker': [([16],.291695),([28],.205123),([14],.313882),
                   ([31],.238941),([32,33,34,35,37,39,40,41],.190443)],
        'bogie': [([13],.317213),([11],.090510),([15],.090510),
                  ([1],.238941),([17],.238941),
                  ([2,3,4,5,7,9,10,12],.190443),
                  ([19,20,21,22,24,26,27,29],.190443)]}
    bogie_ids={i for ids,_ in groups['bogie'] for i in ids}|{6,8,23,25}
    report={'source':'INRB_suspension_load_cf_circular_right.stl',
            'orientation':'bogie pair forward (+ROS X); motor output flanges face outboard toward wheels',
            'bogie_reassembly_translation_mm':shift.tolist(),
            'rocker_pivot_cad_mm':rocker_pivot.tolist(), 'bogie_pivot_cad_mm':holder.tolist(),
            'inertia_method':'mass-weighted component bounding boxes; approximate',
            'wheel_mass_kg':wheel_mass,'links':{}}
    for kind,items in groups.items():
        pivot=rocker_pivot if kind=='rocker' else holder
        ids=[i for group,_ in items for i in group]
        for side,mirror in [('right',-1),('left',1)]:
            pieces=[]; boxes=[]
            for shell_ids,mass in items:
                part=np.array(raw[np.isin(labels,shell_ids)],copy=True)
                if kind=='bogie':part['vertices']+=shift
                part['vertices']=(part['vertices']-pivot)@rotation.T*.001
                part['normal']=part['normal']@rotation.T
                part['vertices'][:,:,1]*=mirror;part['normal'][:,1]*=mirror
                if mirror==1:part['vertices']=part['vertices'][:,[0,2,1],:]
                lo,hi=part['vertices'].min((0,1)),part['vertices'].max((0,1))
                boxes.append((mass,(hi+lo)/2,hi-lo));pieces.append(part)
            mass=sum(a for a,_,_ in boxes)
            com=sum(a*c for a,c,_ in boxes)/mass
            inertia=np.zeros((3,3))
            for m,c,size in boxes:
                x,y,z=size;d=c-com
                inertia+=np.diag([m*(y*y+z*z)/12,m*(x*x+z*z)/12,m*(x*x+y*y)/12])
                inertia+=m*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
            name=f'{side}_{kind}'
            write_mesh(pkg/'meshes'/f'{name}.stl',np.concatenate(pieces))
            report['links'][name]={'mass':mass,'com':com.tolist(),
                'inertia':[float(inertia[i,j]) for i,j in [(0,0),(0,1),(0,2),(1,1),(1,2),(2,2)]]}
    # Original meshes include fixed suspension hardware. Remove that envelope
    # only in new derived chassis meshes, avoiding doubled moving/static visuals.
    for src in ['chassis','chassis_without_arm']:
        mesh=np.memmap(pkg/'meshes'/f'{src}.stl',mode='r',offset=84,dtype=DTYPE)
        c=mesh['vertices'].mean(1)
        remove=(abs(c[:,1])>.24)&(c[:,2]<.425)
        write_mesh(pkg/'meshes'/f'{src}_suspended.stl',np.array(mesh[~remove]))
        report[src+'_removed_triangles']=int(remove.sum())
    # Wheel visuals keep the assembled rover's centered tire meshes. Coupler
    # mass is included, but its small visual shell is omitted rather than left
    # on the wrong rigid link. No exploded wheels are added a second time.
    cfg={'rocker_x':0.,'rocker_y':.300,'rocker_z':.28878,
         'bogie_x':.249938,'bogie_z':-.144446,
         'front_x':.500018,'middle_x':-.000012,'rear_x':-.499980,
         'wheel_z':-.28878,'wheel_mass':wheel_mass,
         'rocker_limit':.60,'bogie_limit':.75,'joint_damping':.4,
         'joint_friction':.02,'differential_stiffness':1500.,
         'differential_damping':60.,'differential_max_torque':200.,
         'links':report['links']}
    (pkg/'config/suspension.yaml').write_text('# CAD geometry/masses; inertia, limits and damping are approximate.\n'+yaml.safe_dump(cfg,sort_keys=False))
    (pkg/'meshes/suspension_report.json').write_text(json.dumps(report,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(11,7))
    for name,origin in [('right_rocker',[0,0,.28878]),
                        ('right_bogie',[.249938,0,.144334])]:
        mesh=np.memmap(pkg/'meshes'/f'{name}.stl',mode='r',offset=84,dtype=DTYPE)
        pts=mesh['vertices'].reshape(-1,3)[::80]+origin
        ax.scatter(pts[:,0],pts[:,2],s=.8,label=name)
    for x in [.500018,-.000012,-.499980]:
        ax.add_patch(plt.Circle((x,0),.15,fill=False,color='black'))
        ax.plot(x,0,'k+')
    ax.plot([0,.249938],[.28878,.144334],'ro',label='recovered pivots')
    ax.axhline(-.15,color='gray');ax.set_aspect('equal');ax.grid()
    ax.set_xlabel('ROS X / m (front toward bogie pair →)');ax.set_ylabel('Z relative to base_link / m')
    ax.legend();ax.set_title('Reassembled right suspension, neutral pose')
    fig.tight_layout();fig.savefig(ROOT/'docs/suspension_assembled.png',dpi=140)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()

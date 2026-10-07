#!/usr/bin/env python3
"""Uniform-material volume inertia from the isolated honeycomb wheel CAD shell.

This accounts for geometric voids, NOT elastic deformation or material grading.
Run inspect_suspension.py first. A central coupler is approximated separately.
"""
import json
from pathlib import Path
import numpy as np
import yaml
from inspect_suspension import ROOT, DTYPE


def main():
    raw=np.memmap(ROOT/'INRB_suspension_load_cf_circular_right.stl',mode='r',offset=84,dtype=DTYPE)
    labels=np.load(ROOT/'docs/suspension_shell_labels.npy')
    v=np.array(raw['vertices'][labels==0],dtype=float)*.001
    center=(v.min((0,1))+v.max((0,1)))/2;v-=center
    # Verify oriented closure. CAD honeycomb junctions include four-face edges;
    # these are not manifold/FEA-ready, but their signed boundary must cancel.
    _,inverse=np.unique(v.reshape(-1,3),axis=0,return_inverse=True)
    faces=inverse.reshape(-1,3)
    directed=np.concatenate([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]])
    edges=np.sort(directed,axis=1)
    _,edge_index,counts=np.unique(edges,axis=0,return_inverse=True,return_counts=True)
    winding=np.bincount(edge_index,weights=np.where(directed[:,0]<directed[:,1],1,-1))
    assert np.all(winding==0),'Open/inconsistently wound shell; do not infer volume inertia'
    a,b,c=v[:,0],v[:,1],v[:,2]
    vol=np.einsum('ij,ij->i',a,np.cross(b,c))/6
    volume=vol.sum();summed=a+b+c
    assert abs(abs(volume)-.003187)/.003187 < .005,'Volume differs from supplied Fusion screenshot'
    com=np.einsum('i,ij->j',vol,summed)/(4*volume)
    second=(np.einsum('i,ij,ik->jk',vol,summed,summed)
            +sum(np.einsum('i,ij,ik->jk',vol,q,q) for q in [a,b,c]))/20/volume
    covariance=second-np.outer(com,com)
    unit_inertia=np.trace(covariance)*np.eye(3)-covariance
    # CAD Z axle -> ROS Y axle. Recenter the wheel at its COM for this model.
    rot=np.array([[1,0,0],[0,0,-1],[0,1,0]])
    unit_inertia=rot@unit_inertia@rot.T
    wheel_mass=3.694552;coupler_mass=.054917
    cr=.0284;cw=.0264
    coupler=np.diag([coupler_mass*(3*cr*cr+cw*cw)/12,
                     coupler_mass*cr*cr/2,coupler_mass*(3*cr*cr+cw*cw)/12])
    inertia=wheel_mass*unit_inertia+coupler
    assert np.linalg.eigvalsh(inertia).min()>0
    result={'source':'INRB_suspension_load_cf_circular_right.stl, isolated shell 0',
      'method':'closed-mesh uniform-density volume integral plus centered cylindrical coupler estimate',
      'deformation_modeled':False,'volume_m3':abs(float(volume)),
      'nonmanifold_edges':int(np.count_nonzero(counts!=2)),
      'wheel_mass_kg':wheel_mass,'coupler_mass_kg':coupler_mass,
      'cad_com_offset_from_bbox_m':com.tolist(),
      'reference_mass':wheel_mass+coupler_mass,
      'inertia':[float(inertia[i,j]) for i,j in [(0,0),(0,1),(0,2),(1,1),(1,2),(2,2)]]}
    (ROOT/'src/rover_description/config/wheel_inertia.yaml').write_text(yaml.safe_dump(result,sort_keys=False))
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()

#!/usr/bin/env python3
"""Inventory disconnected STL shells; coordinates are original CAD millimetres."""
from pathlib import Path
import json
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[3]
DTYPE=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])

def main():
    source=ROOT/'INRB_suspension_load_cf_circular_right.stl'
    raw=np.memmap(source,mode='r',offset=84,dtype=DTYPE)
    vertices,inverse=np.unique(np.round(raw['vertices'].reshape(-1,3),3),axis=0,return_inverse=True)
    triangles=inverse.reshape(-1,3)
    edges=np.concatenate([triangles[:,[0,1]],triangles[:,[1,2]],triangles[:,[2,0]]])
    graph=coo_matrix((np.ones(len(edges),dtype=np.uint8),(edges[:,0],edges[:,1])),shape=(len(vertices),len(vertices)))
    count,labels=connected_components(graph,directed=False)
    face_labels=labels[triangles[:,0]]
    np.save(ROOT/'docs/suspension_shell_labels.npy',face_labels)
    report=[]
    fig,ax=plt.subplots(figsize=(14,9))
    for label in range(count):
        pts=vertices[labels==label]
        num=int(np.count_nonzero(face_labels==label))
        lo,hi=pts.min(0),pts.max(0)
        report.append({'id':label,'triangles':num,'min':lo.tolist(),'max':hi.tolist(),'center':((lo+hi)/2).tolist()})
        if num>200:
            sample=pts[::max(1,len(pts)//1000)]
            ax.scatter(sample[:,0],sample[:,1],s=.4)
            ax.text(*((lo+hi)/2)[:2],str(label),fontsize=7)
    ax.set_aspect('equal'); ax.set_xlabel('CAD X / mm'); ax.set_ylabel('CAD Y / mm'); ax.grid()
    fig.savefig(ROOT/'docs/suspension_shells.png',dpi=130)
    (ROOT/'docs/suspension_shells.json').write_text(json.dumps(report,indent=2))
    print('Shells',count)
    for p in report:
        if p['triangles']>200:print(p)

if __name__=='__main__':main()

"""Streaming voxel map with pose SLERP, scan parity and acquisition-time provenance.

No per-point deskew: each scan uses its header pose. relative_time_s and stamp_s
are voxel means (not original individual returns). count weights allow incremental
voxel aggregation without changing centroids. Input bags and trajectories are read-only.
"""
import argparse
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from .gnss_gt import bag_reader
from .traj_eval import load_tum

VOXEL_M = 0.1
MERGE_EVERY_SCANS = 20
MAX_VOXELS = 3_000_000


def voxel_reduce(rows, voxel=VOXEL_M):
    """Nx6: xyz, relative-time, header-stamp, weight. Weighted voxel means."""
    if voxel<=0: raise ValueError('Voxel size must be positive')
    if not len(rows): return np.empty((0,6))
    keys=np.floor(rows[:,:3]/voxel).astype(np.int64)
    _, inverse=np.unique(keys,axis=0,return_inverse=True)
    n=int(inverse.max())+1
    if n>MAX_VOXELS: raise MemoryError(f'Voxel budget exceeded ({n}); increase voxel size')
    weights=np.bincount(inverse,weights=rows[:,5],minlength=n)
    means=np.column_stack([np.bincount(inverse,weights=rows[:,k]*rows[:,5],minlength=n)/weights for k in range(5)])
    return np.column_stack((means,weights))


def cloud_rows(msg):
    names=['x','y','z','time']; fields={f.name:f for f in msg.fields}
    if not set(names)<=fields.keys(): raise ValueError('Cloud missing xyz/time')
    if any(fields[n].datatype!=7 or fields[n].count!=1 for n in names): raise ValueError('Expected float32 xyz/time')
    endian='>' if msg.is_bigendian else '<'
    dt=np.dtype(dict(names=names,formats=[endian+'f4']*4,
                     offsets=[fields[n].offset for n in names],itemsize=msg.point_step))
    a=np.ndarray((msg.height,msg.width),dtype=dt,buffer=msg.data,strides=(msg.row_step,msg.point_step)).reshape(-1)
    xyz=np.column_stack([a[n] for n in names[:3]])
    mask=np.isfinite(xyz).all(axis=1)&np.isfinite(a['time'])&(np.linalg.norm(xyz,axis=1)>0)
    return xyz[mask],a['time'][mask]


def build_map(bag, trajectory, scan_filter='all', voxel=VOXEL_M, windows=None, alignment=None):
    if scan_filter not in ('all','even','odd'): raise ValueError('Invalid scan filter')
    trajectory=np.asarray(trajectory)
    interp=Slerp(trajectory[:,0], Rotation.from_quat(trajectory[:,4:]))
    reader=bag_reader(bag,['/points_raw'])
    windows=windows or [(float(trajectory[0,0]),float(trajectory[-1,0]))]
    accumulated=[np.empty((0,6)) for _ in windows]; pending=[[] for _ in windows]
    scanned=0; used=[0]*len(windows)
    while reader.has_next():
        _,raw,_=reader.read_next()
        index=scanned; scanned+=1
        if scan_filter!='all' and index%2 != (0 if scan_filter=='even' else 1): continue
        msg=deserialize_message(raw,PointCloud2)
        t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if t<trajectory[0,0] or t>trajectory[-1,0]: continue
        targets=[i for i,(lo,hi) in enumerate(windows) if lo<=t<=hi]
        if not targets: continue
        xyz,relative=cloud_rows(msg)
        rotation=interp([t]).as_matrix()[0]
        translation=np.array([np.interp(t,trajectory[:,0],trajectory[:,k]) for k in range(1,4)])
        xyz=xyz@rotation.T+translation
        if alignment is not None:
            r,trans=alignment; xyz=xyz@np.asarray(r).T+np.asarray(trans)
        rows=voxel_reduce(np.column_stack((xyz,relative,np.full(len(xyz),t),np.ones(len(xyz)))),voxel)
        for i in targets:
            pending[i].append(rows); used[i]+=1
            if len(pending[i])>=MERGE_EVERY_SCANS:
                accumulated[i]=voxel_reduce(np.concatenate([accumulated[i],*pending[i]]),voxel); pending[i]=[]
    for i in range(len(windows)):
        if pending[i]: accumulated[i]=voxel_reduce(np.concatenate([accumulated[i],*pending[i]]),voxel)
        if not len(accumulated[i]): raise ValueError(f'No scans covered window {windows[i]}')
    return accumulated,dict(scans_read=scanned,scans_used=used,voxel_m=voxel,scan_filter=scan_filter,
                            deskew=False,relative_time='voxel weighted mean of original scan-relative seconds')


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--bag',required=True); p.add_argument('--traj',required=True)
    p.add_argument('--out',required=True); p.add_argument('--scan-filter',choices=['all','even','odd'],default='all')
    p.add_argument('--voxel',type=float,default=VOXEL_M)
    a=p.parse_args(); maps,meta=build_map(a.bag,load_tum(a.traj),a.scan_filter,a.voxel)
    import json
    target=Path(a.out); target.parent.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(target,xyz=maps[0][:,:3],relative_time_s=maps[0][:,3],stamp_s=maps[0][:,4],count=maps[0][:,5],metadata=json.dumps(meta))

"""Local differential Gaussian entropy, minimum covariance eigenvalue and overlap."""
import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from tools.pcd_to_tum import read_pcd
from .build_map_from_poses import voxel_reduce

@dataclass(frozen=True)
class MapConfig:
    voxel_m: float = 0.1
    radius_m: float = 0.3
    min_neighbors: int = 6
    covariance_floor_m2: float = 1e-9
    query_chunk: int = 2000
    double_wall_m: float = 0.2
    overlap_gate_m: float = 1.0


def read_map(path, r, t, bounds, cfg=MapConfig()):
    a=read_pcd(path)
    xyz=np.column_stack([a[k] for k in ('x','y','z')]).astype(float)
    xyz=xyz[np.isfinite(xyz).all(axis=1)]@np.asarray(r).T+np.asarray(t)
    mask=np.all((xyz[:,:2]>=bounds[0])&(xyz[:,:2]<=bounds[1]),axis=1)
    xyz=xyz[mask]
    if not len(xyz): raise ValueError('No map points inside GT XY bounding box')
    return voxel_reduce(np.column_stack((xyz,np.zeros((len(xyz),2)),np.ones(len(xyz)))),cfg.voxel_m)[:,:3]


def local_quality(xyz,cfg=MapConfig()):
    tree=cKDTree(xyz); entropy=[]; variance=[]
    for start in range(0,len(xyz),cfg.query_chunk):
        groups=tree.query_ball_point(xyz[start:start+cfg.query_chunk],cfg.radius_m,workers=1)
        for ids in groups:
            if len(ids)<cfg.min_neighbors: continue
            cov=np.cov(xyz[ids],rowvar=False,ddof=1)
            eigen=np.maximum(np.linalg.eigvalsh(cov),cfg.covariance_floor_m2)
            entropy.append(0.5*np.log((2*np.pi*np.e)**3*np.prod(eigen)))
            variance.append(eigen[0])
    if not entropy: raise ValueError('No neighborhoods with sufficient neighbors')
    return dict(mme_nats=float(np.mean(entropy)),mpv_m2=float(np.mean(variance)),
                eligible_points=len(entropy),total_points=len(xyz),eligible_fraction=len(entropy)/len(xyz),
                radius_m=cfg.radius_m,min_neighbors=cfg.min_neighbors,covariance_floor_m2=cfg.covariance_floor_m2,
                formula='MME=mean(0.5 ln((2 pi e)^3 det(C))); MPV=mean(lambda_min(C)), sample covariance; lower is locally sharper. Density/scene dependent.')


def overlap(a,b,cfg=MapConfig()):
    distances=[]; result={}
    for label,x,y in [('A_to_B',a,b),('B_to_A',b,a)]:
        d=cKDTree(y).query(x,k=1,workers=1)[0]
        inside=d<=cfg.overlap_gate_m
        selected=d[inside]
        if not len(selected): raise ValueError('Empty bidirectional overlap')
        result[label]=dict(total_points=len(d),overlap_points=int(inside.sum()),overlap_fraction=float(inside.mean()))
        distances.append(selected)
    d=np.concatenate(distances)
    result.update(median_m=float(np.median(d)),p95_m=float(np.percentile(d,95)),double_wall_ratio=float(np.mean(d>cfg.double_wall_m)),
                  threshold_m=cfg.double_wall_m,overlap_gate_m=cfg.overlap_gate_m,
                  note='Symmetric nearest neighbors restricted to <=1 m mutual spatial support per direction. >0.2 m is a discrepancy proxy, not a semantic wall classifier.')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--map',required=True); p.add_argument('--alignment',required=True)
    p.add_argument('--gt',required=True); p.add_argument('--out',required=True)
    a=p.parse_args(); align=json.loads(Path(a.alignment).read_text())['se3']; gt=np.loadtxt(a.gt)
    xyz=read_map(a.map,align['R'],align['t'],(gt[:,1:3].min(axis=0),gt[:,1:3].max(axis=0)))
    Path(a.out).write_text(json.dumps(local_quality(xyz),indent=2))

"""Open3D GICP with a fixed target and cached target correspondences.

The public registration_generalized_icp API rebuilds the target tree on each call
and hides iteration counts. We expose its ICP outer loop, calling the identical
Open3D TransformationEstimationForGeneralizedICP solver with a persistent scipy
cKDTree. Covariances are the GICP planar epsilon/1/1 model from 20-NN normals.
The reference public API is cross-checked by a synthetic regression test.
"""
import argparse
import copy
import csv
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from tools.eval.gnss_gt import bag_reader
from tools.eval.build_map_from_poses import cloud_rows
from tools.eval.traj_eval import load_tum

@dataclass(frozen=True)
class Config:
    min_range_m: float = 1.
    max_range_m: float = 80.
    scan_voxel_m: float = .3
    map_voxel_m: float = .2
    correspondence_m: tuple = (1., .5)
    max_iterations: int = 30
    convergence_fitness: float = 1e-6
    convergence_rmse: float = 1e-6
    min_fitness: float = .3
    max_rmse_m: float = .3
    lost_failures: int = 5
    covariance_knn: int = 20
    covariance_epsilon: float = .001
    robustness_scans: int = 30
    success_position_m: float = .3
    success_yaw_deg: float = 1.


def output_dir(path):
    path=Path(path).resolve(); root=Path('results/loc').resolve()
    if path!=root and root not in path.parents:
        raise ValueError('Outputs must be under results/loc/')
    path.mkdir(parents=True,exist_ok=True)
    return path


def cloud(xyz, cfg=Config(), downsample=False):
    xyz=np.asarray(xyz,dtype=float)
    if xyz.ndim!=2 or xyz.shape[1]!=3 or not np.isfinite(xyz).all():
        raise ValueError('Expected finite Nx3 points')
    p=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    if downsample: p=p.voxel_down_sample(cfg.scan_voxel_m)
    if len(p.points)<cfg.covariance_knn: raise ValueError('Too few points for GICP')
    p.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(cfg.covariance_knn))
    normals=np.asarray(p.normals)
    cov=np.eye(3)[None]-(1-cfg.covariance_epsilon)*normals[:,:,None]*normals[:,None,:]
    p.covariances=o3d.utility.Matrix3dVector(cov)
    return p


def query_scans(bag, cfg=Config()):
    reader=bag_reader(bag,['/points_raw']); previous=-np.inf
    while reader.has_next():
        _,raw,_=reader.read_next(); start=perf_counter()
        msg=deserialize_message(raw,PointCloud2)
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if stamp<=previous: raise ValueError('Query scan stamps must increase')
        previous=stamp
        xyz,_=cloud_rows(msg); r=np.linalg.norm(xyz,axis=1)
        p=cloud(xyz[(r>=cfg.min_range_m)&(r<=cfg.max_range_m)],cfg,True)
        yield stamp,p,(perf_counter()-start)*1000


def poses_at(trajectory, times):
    times=np.asarray(times)
    if np.any(times<trajectory[0,0]) or np.any(times>trajectory[-1,0]):
        raise ValueError('Initialization/reference outside map-pose interval')
    out=np.repeat(np.eye(4)[None],len(times),axis=0)
    out[:,:3,:3]=Slerp(trajectory[:,0],Rotation.from_quat(trajectory[:,4:]))(times).as_matrix()
    out[:,:3,3]=np.column_stack([np.interp(times,trajectory[:,0],trajectory[:,i]) for i in (1,2,3)])
    return out


def predict(history, stamp):
    """World linear velocity + constant body angular velocity, actual dt."""
    if not history: raise ValueError('Prediction requires an initialized pose')
    t1,p1=history[-1]
    if stamp<t1: raise ValueError('Prediction time precedes history')
    if len(history)<2: return p1.copy()
    t0,p0=history[-2]
    if t1<=t0: raise ValueError('Nonpositive history time step')
    ratio=(stamp-t1)/(t1-t0)
    p=p1.copy(); p[:3,3]+=ratio*(p1[:3,3]-p0[:3,3])
    delta=Rotation.from_matrix(p0[:3,:3].T@p1[:3,:3]).as_rotvec()
    p[:3,:3]=p1[:3,:3]@Rotation.from_rotvec(delta*ratio).as_matrix()
    return p


def accepted(fitness, rmse, cfg=Config()):
    return bool(np.isfinite([fitness,rmse]).all() and fitness>=cfg.min_fitness and 0<=rmse<=cfg.max_rmse_m)


class Localizer:
    def __init__(self,target,cfg=Config()):
        self.cfg=cfg; self.target=target
        self.tree=cKDTree(np.asarray(target.points))
        self.estimator=o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(cfg.covariance_epsilon)

    def correspondences(self,source,distance):
        d,j=self.tree.query(np.asarray(source.points),distance_upper_bound=distance,workers=1)
        good=np.isfinite(d)
        pairs=np.column_stack((np.flatnonzero(good),j[good])).astype(np.int32)
        fitness=float(good.mean()); rmse=float(np.sqrt(np.mean(d[good]**2))) if good.any() else float('inf')
        return o3d.utility.Vector2iVector(pairs),fitness,rmse

    def register(self,source,initial):
        start=perf_counter(); moving=copy.deepcopy(source); moving.transform(initial)
        pose=initial.copy(); iterations=0; stages=[]
        for distance in self.cfg.correspondence_m:
            pairs,fitness,rmse=self.correspondences(moving,distance)
            count=0; converged=False
            for _ in range(self.cfg.max_iterations):
                if len(pairs)<6: break
                update=self.estimator.compute_transformation(moving,self.target,pairs)
                if not np.isfinite(update).all(): break
                pose=update@pose; moving.transform(update); count+=1
                pairs,new_fitness,new_rmse=self.correspondences(moving,distance)
                converged=(abs(new_fitness-fitness)<self.cfg.convergence_fitness and
                           abs(new_rmse-rmse)<self.cfg.convergence_rmse)
                fitness,rmse=new_fitness,new_rmse
                if converged: break
            iterations+=count; stages.append(dict(iterations=count,converged=converged))
        return pose,dict(fitness=fitness,inlier_rmse_m=rmse,iterations=iterations,
                         coarse_iterations=stages[0]['iterations'],fine_iterations=stages[1]['iterations'],
                         coarse_converged=stages[0]['converged'],fine_converged=stages[1]['converged'],
                         registration_ms=(perf_counter()-start)*1000)


def track(localizer, scans, initial):
    history=[]; failures=0
    for index,(stamp,source,preprocess_ms) in enumerate(scans):
        start=perf_counter()
        prediction=predict(history,stamp) if history else initial.copy()
        pose,detail=localizer.register(source,prediction)
        ok=accepted(detail['fitness'],detail['inlier_rmse_m'],localizer.cfg)
        failures=0 if ok else failures+1
        if not ok: pose=prediction
        history.append((stamp,pose)); history=history[-2:]
        row=dict(index=index,timestamp=stamp,**detail,preprocess_ms=preprocess_ms,
                 total_ms=preprocess_ms+(perf_counter()-start)*1000,accepted=ok,
                 consecutive_failures=failures,lost=failures>=localizer.cfg.lost_failures)
        yield stamp,pose,row
        if row['lost']: break


def tum_row(stamp,pose):
    return np.r_[stamp,pose[:3,3],Rotation.from_matrix(pose[:3,:3]).as_quat()]


def write_csv(path,rows):
    if not rows: raise ValueError('Cannot write empty table')
    with Path(path).open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prior',required=True); p.add_argument('--query-bag',required=True)
    p.add_argument('--initial-traj',required=True); p.add_argument('--out',default='results/loc/standalone')
    a=p.parse_args(); out=output_dir(a.out)
    target=cloud(np.load(a.prior)['xyz']); scans=iter(query_scans(a.query_bag)); first=next(scans)
    from itertools import chain
    initial=poses_at(load_tum(a.initial_traj),[first[0]])[0]
    result=list(track(Localizer(target),chain([first],scans),initial))
    np.savetxt(out/'loc_traj_tum.txt',np.array([tum_row(t,p) for t,p,_ in result]),fmt='%.9f')
    write_csv(out/'per_scan.csv',[r for _,_,r in result])

if __name__=='__main__': main()

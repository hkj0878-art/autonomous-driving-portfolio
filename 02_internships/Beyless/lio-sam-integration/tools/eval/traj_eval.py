"""Position-only SE3 ATE, joint antenna lever arm fit and distance-based RPE."""
import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

@dataclass(frozen=True)
class EvalConfig:
    max_gt_gap_s: float = 0.2
    lever_warning_m: float = 3.0
    segment_lengths_m: tuple = (10., 25., 50., 100.)
    tangent_baseline_m: float = 5.0
    motion_yaw_min_step_m: float = 0.02


def load_tum(path):
    a = np.loadtxt(path, ndmin=2)
    if a.shape[1] != 8 or len(a)<3 or not np.isfinite(a).all() or (np.diff(a[:,0])<=0).any():
        raise ValueError(f'Invalid TUM trajectory: {path}')
    if not np.allclose(np.linalg.norm(a[:,4:],axis=1),1,atol=1e-4):
        raise ValueError(f'Invalid quaternion: {path}')
    return a


def associate(est, gt, max_gap=0.2):
    t = est[:,0]; j=np.searchsorted(gt[:,0], t, side='right')
    j=np.clip(j,1,len(gt)-1)
    mask=(t>=gt[0,0]) & (t<=gt[-1,0]) & ((gt[j,0]-gt[j-1,0])<=max_gap)
    idx=np.flatnonzero(mask); j=j[mask]
    f=(t[mask]-gt[j-1,0])/(gt[j,0]-gt[j-1,0])
    points=gt[j-1,1:4]*(1-f[:,None])+gt[j,1:4]*f[:,None]
    if len(points)<3: raise ValueError('Fewer than three synchronized poses')
    return est[mask], points, idx


def umeyama(x, y):
    """Return rigid R,t taking x to y; no scale fit, determinant +1."""
    x=np.asarray(x); y=np.asarray(y)
    if x.shape != y.shape or x.ndim!=2 or x.shape[1]!=3 or len(x)<3:
        raise ValueError('Alignment needs paired Nx3 points')
    xc=x-x.mean(axis=0); yc=y-y.mean(axis=0)
    u, _, vt=np.linalg.svd(yc.T@xc/len(x))
    sign=np.eye(3); sign[-1,-1]=np.linalg.det(u@vt)
    r=u@sign@vt; t=y.mean(axis=0)-r@x.mean(axis=0)
    return r,t


def fit_lever(x, rotations, y):
    r,t=umeyama(x,y)
    def residual(v):
        rp=Rotation.from_rotvec(v[:3]).as_matrix()
        return ((x+np.einsum('nij,j->ni',rotations,v[6:]))@rp.T+v[3:6]-y).ravel()
    solution=least_squares(residual, np.r_[Rotation.from_matrix(r).as_rotvec(),t,[0.,0.,0.]],
                           xtol=1e-11, ftol=1e-11, gtol=1e-11, max_nfev=1000)
    if not solution.success: raise ValueError(f'Lever arm solver failed: {solution.message}')
    singular=np.linalg.svd(solution.jac,compute_uv=False)
    return (Rotation.from_rotvec(solution.x[:3]).as_matrix(),solution.x[3:6],solution.x[6:],
            dict(jacobian_singular_values=singular.tolist(), jacobian_condition=float(singular[0]/max(singular[-1],1e-15)),
                 note='Planar motion weakly observes vertical lever arm; joint fit may absorb trajectory errors.'))


def yaw_only(rotations):
    """ENU body heading: strip LiDAR mounting roll/pitch after alignment."""
    yaw = np.arctan2(rotations[:, 1, 0], rotations[:, 0, 0])
    return Rotation.from_euler('z', yaw).as_matrix()


def fit_lever_2d(x, rotations, y):
    """Joint rigid alignment and horizontal yaw-body lever; lz fixed to zero.

    p_gnss = R_align p_est + t + Rz(yaw(R_align R_est)) [lx,ly,0].
    SE3 without lever is retained independently for maps and evo validation.
    """
    r, t = umeyama(x, y)
    def residual(v):
        rp = Rotation.from_rotvec(v[:3]).as_matrix()
        body = yaw_only(np.einsum('ij,njk->nik', rp, rotations))
        return (x @ rp.T + v[3:6] + body @ np.r_[v[6:8], 0.] - y).ravel()
    solution = least_squares(residual, np.r_[Rotation.from_matrix(r).as_rotvec(), t, 0., 0.],
                             xtol=1e-11, ftol=1e-11, gtol=1e-11, max_nfev=1000)
    if not solution.success:
        raise ValueError(f'Horizontal lever solver failed: {solution.message}')
    singular = np.linalg.svd(solution.jac, compute_uv=False)
    return (Rotation.from_rotvec(solution.x[:3]).as_matrix(), solution.x[3:6],
            np.r_[solution.x[6:8], 0.], dict(
                model='R_align p_est + t + Rz(yaw(R_align R_est)) [lx,ly,0]',
                dof=2, jacobian_singular_values=singular.tolist(),
                jacobian_condition=float(singular[0]/max(singular[-1], 1e-15)),
                note='Yaw-only body removes approximately 9.3 degree LiDAR mounting pitch; in-sample nuisance fit, not independent antenna calibration.'))


def stats(v):
    v=np.asarray(v)
    return dict(rmse=float(np.sqrt(np.mean(v*v))), mean=float(v.mean()), median=float(np.median(v)),
                max=float(v.max()), std=float(v.std()))


def tangent_yaw(points, baseline=5.):
    s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(points[:,:2],axis=0),axis=1))]
    left=np.clip(np.searchsorted(s,s-baseline/2,side="right")-1,0,len(s)-1)
    right=np.clip(np.searchsorted(s,s+baseline/2),0,len(s)-1)
    delta=points[right,:2]-points[left,:2]
    if np.any(np.linalg.norm(delta,axis=1)<1e-6): raise ValueError('GNSS tangent undefined')
    return np.unwrap(np.arctan2(delta[:,1],delta[:,0]))


def rpe(points, est_rotation, gt_positions, cfg):
    """All start poses, first end pose at or beyond target GT arc length (KITTI)."""
    est_rotation=yaw_only(est_rotation)
    s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(gt_positions,axis=0),axis=1))]
    yaw=tangent_yaw(gt_positions,cfg.tangent_baseline_m)
    gr=Rotation.from_euler('z',yaw).as_matrix(); result={}
    for length in cfg.segment_lengths_m:
        translation=[]; angles=[]; actual=[]
        for i in range(len(s)):
            j=int(np.searchsorted(s,s[i]+length))
            if j>=len(s): continue
            d=s[j]-s[i]
            gt_delta=gr[i].T@(gt_positions[j]-gt_positions[i])
            est_delta=est_rotation[i].T@(points[j]-points[i])
            gt_rel=gr[i].T@gr[j]; est_rel=est_rotation[i].T@est_rotation[j]
            err=est_rel.T@gt_rel
            translation.append(np.linalg.norm(est_delta-gt_delta)/d*100)
            angles.append(abs(np.arctan2(err[1,0],err[0,0]))*180/np.pi/d*100)
            actual.append(d)
        result[str(int(length))]=dict(count=len(actual), translation_percent=stats(translation) if actual else None,
                                       yaw_deg_per_100m=stats(angles) if actual else None,
                                       mean_actual_length_m=float(np.mean(actual)) if actual else None)
    return result


def evaluate(est, gt, out=None, config=EvalConfig()):
    matched,y,idx=associate(est,gt,config.max_gt_gap_s); x=matched[:,1:4]
    rot=Rotation.from_quat(matched[:,4:]).as_matrix()
    r,t=umeyama(x,y); rb,tb,l,diagnostic=fit_lever_2d(x,rot,y)
    r3,t3,l3,diagnostic3=fit_lever(x,rot,y)
    aligned=x@r.T+t
    antenna=x@rb.T+tb+yaw_only(np.einsum('ij,njk->nik',rb,rot))@l
    antenna3=(x+rot@l3)@r3.T+t3
    rpe_result=rpe(antenna,np.einsum('ij,njk->nik',rb,rot),y,config)
    rpe_se3=rpe(aligned,np.einsum('ij,njk->nik',r,rot),y,config)
    for result in (rpe_result,rpe_se3):
        for length, value in result.items():
            if float(length)<25: value['yaw_deg_per_100m']=None
    ea=np.linalg.norm(aligned-y,axis=1); eb=np.linalg.norm(antenna-y,axis=1)
    ea2=np.linalg.norm((aligned-y)[:,:2],axis=1); eb2=np.linalg.norm((antenna-y)[:,:2],axis=1)
    from evo.core.geometry import umeyama_alignment
    er,et,_=umeyama_alignment(x.T,y.T,with_scale=False)
    evo_rmse=float(np.sqrt(np.mean(np.sum((x@er.T+et-y)**2,axis=1))))
    diff=abs(evo_rmse-stats(ea)['rmse'])
    if diff>=0.001: raise AssertionError(f'evo ATE mismatch {diff}')
    endpoint_error=(aligned[-1]-aligned[0])-(y[-1]-y[0])
    metrics=dict(input_count=len(est), matched_count=len(matched), excluded_count=len(est)-len(matched),
                 interval_s=[float(matched[0,0]),float(matched[-1,0])],
                 primary_metric="2D horizontal ATE after SE3 + yaw-only horizontal 2-DoF lever fit; no scale",
                 se3=dict(R=r.tolist(),t=t.tolist(),ate_m=stats(ea),ate_3d_m=stats(ea),ate_2d_m=stats(ea2)),
                 se3_lever=dict(R=rb.tolist(),t=tb.tolist(),lever_arm_body_m=l.tolist(),lever_norm_m=float(np.linalg.norm(l)),
                                warning=bool(np.linalg.norm(l)>config.lever_warning_m),ate_m=stats(eb),ate_3d_m=stats(eb),ate_2d_m=stats(eb2),diagnostic=diagnostic),
                 se3_lever_3d_diagnostic=dict(R=r3.tolist(),t=t3.tolist(),lever_arm_body_m=l3.tolist(),
                     ate_3d_m=stats(np.linalg.norm(antenna3-y,axis=1)),diagnostic=diagnostic3,
                     note='Legacy unconstrained 3-DoF fit: diagnostic only, excluded from primary metrics.'),
                 endpoint=dict(gt_displacement_m=float(np.linalg.norm(y[-1]-y[0])),
                               est_displacement_m=float(np.linalg.norm(aligned[-1]-aligned[0])),
                               displacement_error_m=float(np.linalg.norm(endpoint_error)),
                               displacement_error_vector_enu_m=endpoint_error.tolist()),
                 rpe=rpe_result, rpe_se3_diagnostic=rpe_se3,
                 rpe_note='Yaw-only aligned body frame removes approximately 9.3 degree LiDAR mounting pitch. GT uses 5 m centered tangent yaw, flat roll/pitch approximation. Translation: 10/25/50/100 m; yaw: >=25 m only. Primary RPE uses lever2D-corrected antenna positions and the jointly fitted alignment yaw, matching primary ATE. SE3-only RPE is retained in rpe_se3_diagnostic; it retains antenna/body origin mismatch. Lever alignment and calibration are in-sample, so these metrics are optimistic. All start poses overlap and are correlated.',
                 evo_crosscheck=dict(ate_rmse_m=evo_rmse, absolute_difference_m=diff),
                 online_note='Online trajectory is causal accumulated poses; it is not a saved before-loop trajectory. Final vs online does not isolate a single loop factor.')
    if out is not None:
        out=Path(out); out.mkdir(parents=True,exist_ok=True)
        (out/'metrics.json').write_text(json.dumps(metrics,indent=2,allow_nan=False))
        np.savetxt(out/'error_vs_time.csv',np.column_stack((matched[:,0],ea,eb,ea2,eb2)),delimiter=',',
                   header='timestamp,se3_error_3d_m,se3_lever_error_3d_m,se3_error_2d_m,se3_lever_error_2d_m',comments='')
    return metrics,dict(t=matched[:,0],gt=y,aligned=aligned,antenna=antenna,error=eb2)


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--traj',required=True); p.add_argument('--gt',required=True); p.add_argument('--out',required=True)
    a=p.parse_args(); evaluate(load_tum(a.traj),load_tum(a.gt),a.out)

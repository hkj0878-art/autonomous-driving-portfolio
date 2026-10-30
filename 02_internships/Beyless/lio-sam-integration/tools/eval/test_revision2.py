"""Regression tests kept in tools/eval to respect revision write scope."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from .traj_eval import (EvalConfig, evaluate, fit_lever_2d, rpe, tangent_yaw,
                        yaw_only)
from .report_rev2 import noise_floor


def curved_track():
    theta=np.linspace(0,2*np.pi,301)
    p=np.column_stack((40*np.cos(theta),30*np.sin(theta),np.zeros(len(theta))))
    yaw=tangent_yaw(p,EvalConfig().tangent_baseline_m)
    return p,yaw


def test_pitch_roll_does_not_create_rpe():
    p,yaw=curved_track()
    body=Rotation.from_euler('z',yaw).as_matrix()
    tilted=body@Rotation.from_euler('xyz',[1.2,9.3,0],degrees=True).as_matrix()
    result=rpe(p,tilted,p,EvalConfig())
    for v in result.values():
        assert v['translation_percent']['mean']<1e-10
        assert v['yaw_deg_per_100m']['mean']<1e-10
    # Verify this regression would detect the original full-rotation error.
    initial=body[0].T@(p[100]-p[0])
    wrong=tilted[0].T@(p[100]-p[0])
    assert np.linalg.norm(wrong-initial)>1


def test_lever2d_recovers_offset_after_world_alignment():
    p,yaw=curved_track()
    world_body=Rotation.from_euler('z',yaw).as_matrix()
    mounting=Rotation.from_euler('xyz',[.4,9.3,0],degrees=True).as_matrix()
    align=Rotation.from_euler('xyz',[.12,-.08,.6]).as_matrix()
    translation=np.array([4.,-7.,.5]);lever=np.array([-1.5,.25,0.])
    x=(p-translation)@align
    est_rot=align.T@world_body@mounting
    gt=p+world_body@lever
    r,t,l,diag=fit_lever_2d(x,est_rot,gt)
    np.testing.assert_allclose(r,align,atol=1e-8)
    np.testing.assert_allclose(t,translation,atol=1e-8)
    np.testing.assert_allclose(l,lever,atol=1e-8)
    assert diag['dof']==2 and l[2]==0
    # Extracting yaw in the unaligned frame would give a different offset model.
    assert np.max(abs(yaw_only(est_rot)-yaw_only(align@est_rot)))>.1


def test_metrics_primary_rpe_uses_same_antenna_and_alignment_as_ate():
    p,yaw=curved_track();rot=Rotation.from_euler('z',yaw).as_matrix()
    tilted=rot@Rotation.from_euler('y',9.3,degrees=True).as_matrix()
    times=np.arange(len(p))*.1
    est=np.column_stack((times,p,Rotation.from_matrix(tilted).as_quat()))
    # GNSS position includes an actual horizontal antenna offset.
    g=p+rot@np.array([-1.5,.2,0.])
    gt=np.column_stack((times,g,Rotation.from_euler('z',yaw).as_quat()))
    m,s=evaluate(est,gt)
    np.testing.assert_allclose(m['se3_lever']['lever_arm_body_m'],[-1.5,.2,0.],atol=1e-7)
    assert m['se3_lever']['ate_2d_m']['rmse']<1e-7
    assert m['rpe']['10']['yaw_deg_per_100m'] is None
    rb=np.array(m['se3_lever']['R'])
    expected=rpe(s['antenna'],rb@tilted,g,EvalConfig())
    assert m['rpe']['100']['translation_percent']['mean']==pytest.approx(expected['100']['translation_percent']['mean'])
    assert 'se3_lever_3d_diagnostic' in m and 'rpe_se3_diagnostic' in m
    assert '9.3' in m['rpe_note'] and '5 m' in m['rpe_note']


def test_noise_floor_is_reproducible_and_nonzero():
    p,_=curved_track();series={'synthetic':{'final':{'gt':p}}}
    a=noise_floor(series);b=noise_floor(series)
    assert len(a) == len(b) == 7
    # SIMD/reduction paths can differ by a few ULPs even with identical RNG draws.
    # Keep exact metadata checks and require numerical repeatability tightly.
    for first, second in zip(a, b):
        assert first.keys() == second.keys()
        for key in first:
            if key in ('clean_gt_mean', 'noise_mean', 'noise_std_sample'):
                assert first[key] == pytest.approx(second[key], rel=1e-12, abs=1e-12)
            else:
                assert first[key] == second[key]
    for row in a:
        assert row['trials']==10 and row['sigma_m']==.014
        assert row['clean_gt_mean']<1e-10
        assert row['noise_mean']>0 and row['noise_std_sample']>0
        assert row['segment_m']>=25 or row['metric']=='translation_percent'

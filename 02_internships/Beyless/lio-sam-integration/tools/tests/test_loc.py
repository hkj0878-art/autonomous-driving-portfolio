"""GICP recovery and public API equivalence, state/prediction boundary behavior."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parents[2]))
import numpy as np
import open3d as o3d
import pytest
from scipy.spatial.transform import Rotation
from tools.loc.localize import Config, Localizer, accepted, cloud, predict, track, poses_at, output_dir


def test_known_rigid_transform_and_public_gicp_equivalence():
    rng=np.random.default_rng(45)
    # Three differently oriented, finite planes constrain all six pose DoF.
    uv=rng.uniform(-3,3,(1000,2))
    xyz=np.r_[np.c_[uv,np.zeros(1000)],np.c_[uv[:,0],np.full(1000,3.),uv[:,1]],
              np.c_[np.full(1000,-3.),uv]]
    truth=np.eye(4); truth[:3,:3]=Rotation.from_euler('xyz',[2,-3,4],degrees=True).as_matrix()
    truth[:3,3]=[.12,-.08,.1]
    source=cloud((xyz-truth[:3,3])@truth[:3,:3]); target=cloud(xyz)
    loc=Localizer(target); result,detail=loc.register(source,np.eye(4))
    np.testing.assert_allclose(result,truth,atol=1e-4)
    reference=np.eye(4)
    for distance in loc.cfg.correspondence_m:
        reference=o3d.pipelines.registration.registration_generalized_icp(
            source,target,distance,reference,loc.estimator,
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=loc.cfg.max_iterations)).transformation
    np.testing.assert_allclose(result,reference,atol=1e-6)
    assert detail['iterations']>0 and accepted(detail['fitness'],detail['inlier_rmse_m'])


def test_constant_velocity_actual_dt_and_rotation():
    a=np.eye(4); b=np.eye(4); b[:3,3]=[1,2,0]
    b[:3,:3]=Rotation.from_euler('z',10,degrees=True).as_matrix()
    c=predict([(1.,a),(1.5,b)],2.5)
    np.testing.assert_allclose(c[:3,3],[3,6,0])
    np.testing.assert_allclose(c[:3,:3],Rotation.from_euler('z',30,degrees=True).as_matrix())
    np.testing.assert_array_equal(predict([(1.,a)],2.),a)
    with pytest.raises(ValueError): predict([(1.,a),(1.,b)],2.)


@pytest.mark.parametrize('fitness,rmse,ok',[(.3,.3,True),(.299,.1,False),(.8,.301,False),
    (np.nan,.1,False),(.8,np.inf,False),(.8,-.1,False),(0,0,False)])
def test_failure_thresholds(fitness,rmse,ok):
    assert accepted(fitness,rmse)==ok


def test_rejection_keeps_prediction_and_fifth_consecutive_failure_lost():
    class Fake:
        cfg=Config()
        def register(self,source,initial):
            bad=initial.copy(); bad[0,3]=100
            return bad,dict(fitness=.1,inlier_rmse_m=.5)
    rows=list(track(Fake(),[(i,None,0) for i in range(10)],np.eye(4)))
    assert len(rows)==5 and rows[-1][2]['lost']
    for _,pose,row in rows:
        np.testing.assert_array_equal(pose,np.eye(4)); assert not row['accepted']


def test_pose_interpolation_and_output_guard():
    traj=np.array([[0,0,0,0,0,0,0,1],[1,1,2,0,0,0,0,1],[2,2,4,0,0,0,0,1]])
    np.testing.assert_allclose(poses_at(traj,[.5])[0,:3,3],[.5,1,0])
    with pytest.raises(ValueError): poses_at(traj,[-1])
    with pytest.raises(ValueError): output_dir('results/eval')

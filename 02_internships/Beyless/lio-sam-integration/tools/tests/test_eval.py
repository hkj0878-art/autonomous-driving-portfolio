import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from tools.eval.traj_eval import umeyama, fit_lever, associate, evaluate, rpe, EvalConfig
from tools.eval.gnss_gt import to_enu
from tools.eval.build_map_from_poses import voxel_reduce
from tools.eval.map_quality import local_quality, overlap


def test_umeyama_exact_and_reflection():
    x=np.random.default_rng(8).normal(size=(100,3))
    r=Rotation.from_euler('xyz',[.2,-.4,.8]).as_matrix();t=np.array([2.,5.,-3.])
    recovered,translation=umeyama(x,x@r.T+t)
    np.testing.assert_allclose(recovered,r,atol=1e-12);np.testing.assert_allclose(translation,t,atol=1e-12)
    reflected=x.copy();reflected[:,0]*=-1
    rr,_=umeyama(x,reflected);assert np.linalg.det(rr)==pytest.approx(1)
    rr,tt=umeyama(x,2*x);assert np.linalg.norm(x@rr.T+tt-2*x)>1


def test_joint_lever_known_6d_motion():
    rng=np.random.default_rng(9);x=rng.normal(size=(150,3))*10
    body=Rotation.random(150,random_state=rng).as_matrix()
    r=Rotation.from_euler('xyz',[.2,-.15,.5]).as_matrix();t=np.array([4.,-8.,2.]);lever=np.array([1.,-.3,.8])
    y=(x+np.einsum('nij,j->ni',body,lever))@r.T+t
    rr,tt,ll,_=fit_lever(x,body,y)
    np.testing.assert_allclose(rr,r,atol=1e-8);np.testing.assert_allclose(tt,t,atol=1e-8);np.testing.assert_allclose(ll,lever,atol=1e-8)


def test_gt_interpolation_and_gap_rejection():
    gt=np.zeros((4,8));gt[:,0]=[1,1.1,1.5,1.6];gt[:,1]=[0,1,5,6]
    est=np.zeros((6,8));est[:,0]=[.9,1.025,1.05,1.3,1.55,1.7]
    matched,y,idx=associate(est,gt)
    np.testing.assert_array_equal(idx,[1,2,4]);np.testing.assert_allclose(y[:,0],[.25,.5,5.5])


def test_evaluate_zero_error_and_evo(tmp_path):
    t=np.arange(101)*.1;xyz=np.column_stack((t*2,np.sin(t),np.cos(t)))
    yaw=np.arctan2(np.gradient(xyz[:,1]),np.gradient(xyz[:,0]))
    tum=np.column_stack((t,xyz,Rotation.from_euler('z',yaw).as_quat()))
    m,_=evaluate(tum,tum,tmp_path)
    assert m['se3']['ate_m']['rmse']<1e-10
    assert m['evo_crosscheck']['absolute_difference_m']<.001
    assert (tmp_path/'metrics.json').exists()


def test_rpe_straight_line_and_scale():
    g=np.column_stack((np.arange(151.),np.zeros((151,2))));rot=np.tile(np.eye(3),(151,1,1))
    perfect=rpe(g,rot,g,EvalConfig());scaled=rpe(g*1.1,rot,g,EvalConfig())
    for k in perfect:
        assert perfect[k]['translation_percent']['mean']<1e-10
        assert scaled[k]['translation_percent']['mean']==pytest.approx(10)
        assert perfect[k]['yaw_deg_per_100m']['mean']==0


def test_enu_axes():
    origin=np.array([0.,0.,0.]);lla=np.array([[0,0,0],[0,.00001,0],[.00001,0,0],[0,0,1]])
    p=to_enu(lla,origin)
    np.testing.assert_allclose(p[0],0,atol=1e-10)
    assert p[1,0]>1 and abs(p[1,1])<1e-10
    assert p[2,1]>1 and abs(p[2,0])<1e-10
    assert p[3,2]==pytest.approx(1)


def test_weighted_voxel_preserves_time_and_centroid():
    rows=np.array([[.01,0,0,.01,100,1],[.03,0,0,.03,102,1],[.21,0,0,.02,101,1]])
    full=voxel_reduce(rows);partial=voxel_reduce(np.concatenate((voxel_reduce(rows[:2]),rows[2:])))
    np.testing.assert_allclose(full,partial);np.testing.assert_allclose(full[0],[.02,0,0,.02,101,2])


def test_plane_quality_and_overlap():
    grid=np.arange(-.5,.51,.1);x,y=np.meshgrid(grid,grid);plane=np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size)))
    quality=local_quality(plane);assert quality['mpv_m2']==pytest.approx(1e-9)
    assert np.isfinite(quality['mme_nats'])
    same=overlap(plane,plane);assert same['p95_m']==0 and same['double_wall_ratio']==0
    shifted=overlap(plane,plane+[0,0,.25]);assert shifted['median_m']==pytest.approx(.25);assert shifted['double_wall_ratio']==1


def test_builder_slerp_parity_and_provenance(monkeypatch):
    from tools.eval import build_map_from_poses as module
    from sensor_msgs.msg import PointCloud2, PointField
    from rclpy.serialization import serialize_message
    raw=[]
    for i in range(3):
        m=PointCloud2();m.header.stamp.sec=10+i;m.height=1;m.width=1;m.point_step=16;m.row_step=16
        m.fields=[PointField(name=n,offset=j*4,datatype=7,count=1) for j,n in enumerate(['x','y','z','time'])]
        m.data=np.array([1,0,0,.025],dtype='<f4').tobytes();raw.append(('/points_raw',serialize_message(m),0))
    class Reader:
        def __init__(self):self.rows=iter(raw);self.count=0
        def has_next(self):return self.count<len(raw)
        def read_next(self):self.count+=1;return next(self.rows)
    monkeypatch.setattr(module,'bag_reader',lambda *args:Reader())
    trajectory=np.column_stack(([10.,12.],np.zeros((2,3)),Rotation.from_euler('z',[0,np.pi]).as_quat()))
    clouds,meta=module.build_map('unused',trajectory,'odd',voxel=.01)
    assert meta['scans_used']==[1]
    np.testing.assert_allclose(clouds[0][0,:3],[0,1,0],atol=1e-8)
    assert clouds[0][0,3]==pytest.approx(.025);assert clouds[0][0,4]==11


def test_horizontal_filter_retains_normal_vertical_sigma(monkeypatch,tmp_path):
    from tools.eval import gnss_gt as module
    from sensor_msgs.msg import NavSatFix
    from rclpy.serialization import serialize_message
    rows=[]
    for i in range(5):
        m=NavSatFix();m.header.stamp.sec=10+i;m.status.status=2
        m.latitude=37.+i*.00001;m.longitude=127.;m.altitude=30.
        m.position_covariance_type=2
        m.position_covariance=[.014**2,0.,0.,0.,(.06 if i==3 else .014)**2,0.,0.,0.,.058**2]
        if i==4:m.status.status=0
        rows.append(('/gps/fix',serialize_message(m),0))
    class Reader:
        def __init__(self):self.i=0
        def has_next(self):return self.i<len(rows)
        def read_next(self):r=rows[self.i];self.i+=1;return r
    monkeypatch.setattr(module,'bag_reader',lambda *a:Reader())
    gt,meta=module.extract('unused',tmp_path)
    assert len(gt)==3 and meta['rejected_covariance']==1 and meta['rejected_status']==1
    assert meta['sigma_axes']=='xy' and meta['vertical_sigma_m']['mean']==pytest.approx(.058)


def test_2d_ate_uses_aligned_horizontal_components():
    t=np.arange(201)*.1
    xyz=np.column_stack((t*2,5*np.sin(t/4),np.zeros(len(t))))
    q=Rotation.from_euler('z',np.arctan2(np.gradient(xyz[:,1]),np.gradient(xyz[:,0]))).as_quat()
    gt=np.column_stack((t,xyz,q));est=gt.copy();est[:,3]=.2*np.sin(t*2)
    m,series=evaluate(est,gt)
    for key,values in [('se3',series['aligned']),('se3_lever',series['antenna'])]:
        delta=values-series['gt']
        assert m[key]['ate_2d_m']['rmse']==pytest.approx(np.sqrt(np.mean(np.sum(delta[:,:2]**2,axis=1))))
        assert m[key]['ate_3d_m']['rmse']==pytest.approx(np.sqrt(np.mean(np.sum(delta**2,axis=1))))
        assert m[key]['ate_3d_m']['rmse']>m[key]['ate_2d_m']['rmse']


def test_tangent_sparse_last_segment_has_previous_support():
    from tools.eval.traj_eval import tangent_yaw
    points=np.column_stack(([0.,1.5,3.,4.2],np.zeros((4,2))))
    np.testing.assert_allclose(tangent_yaw(points),0)

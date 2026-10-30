import sys
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).parents[1]))
from pcd_to_tum import convert

@pytest.mark.parametrize('mode',['ascii','binary'])
def test_pose_pcd_quaternion(tmp_path,mode):
    dtype=np.dtype([(k,'<f8' if k=='time' else '<f4') for k in ('x','y','z','roll','pitch','yaw','time')])
    a=np.array([(1,2,3,.3,-.4,1.2,1760000000.125),(4,5,6,-1.,.8,-2.,1760000000.625)],dtype=dtype)
    header='VERSION .7\nFIELDS x y z roll pitch yaw time\nSIZE 4 4 4 4 4 4 8\nTYPE F F F F F F F\nCOUNT 1 1 1 1 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA '+mode+'\n'
    src=tmp_path/'poses.pcd'; dst=tmp_path/'tum.txt'
    with src.open('wb') as f:
        f.write(header.encode())
        if mode=='binary': f.write(a.tobytes())
        else: np.savetxt(f,np.column_stack([a[n] for n in a.dtype.names]),fmt='%.12f')
    out=convert(src,dst)
    expected=Rotation.from_euler('xyz',np.column_stack([a[k] for k in ('roll','pitch','yaw')])).as_matrix()
    np.testing.assert_allclose(Rotation.from_quat(out[:,4:]).as_matrix(),expected,atol=1e-10)
    np.testing.assert_allclose(out[:,0],a['time'],atol=0,rtol=0)
    np.testing.assert_allclose(out[:,1:4],[[1,2,3],[4,5,6]])

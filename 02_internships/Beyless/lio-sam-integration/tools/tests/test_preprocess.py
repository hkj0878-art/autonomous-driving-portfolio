import importlib.util
from pathlib import Path

import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

spec = importlib.util.spec_from_file_location('preprocess', Path(__file__).parents[1]/'preprocess_bag.py')
p = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p)


def cloud(rows, bigendian=False, organized=False):
    dtype = np.dtype({'names': list(p.INPUT_FIELDS),
                      'formats': [('>' if bigendian else '<')+v[1] for v in p.INPUT_FIELDS.values()],
                      'offsets': [0,4,8,12,16,18], 'itemsize': 26})
    a = np.array(rows, dtype=dtype)
    width, height = (2, len(a)//2) if organized else (len(a), 1)
    data = b''.join(a[i*width:(i+1)*width].tobytes()+b'PAD!' for i in range(height))
    return PointCloud2(header=Header(stamp=Time(sec=200, nanosec=500_000_000), frame_id='front_lidar'),
                      width=width, height=height, point_step=26, row_step=width*26+4,
                      fields=[PointField(name=n, offset=o, datatype=p.INPUT_FIELDS[n][0], count=1)
                              for n,o in zip(p.INPUT_FIELDS,[0,4,8,12,16,18])],
                      is_bigendian=bigendian, data=data)


@pytest.mark.parametrize('bigendian', [False, True])
def test_time_layout_order_and_padding(bigendian):
    msg = cloud([(1,2,3,4,127,100), (2,3,4,5,0,100.025),
                 (3,4,5,6,12,100.010), (4,5,6,7,11,100.020)], bigendian, True)
    out, offset, counts = p.convert_cloud(msg)
    a = np.frombuffer(out.data, dtype=p.OUTPUT_DTYPE)
    assert offset == 100.5
    np.testing.assert_allclose(a['time'], [0,.025,.010,.020], atol=1e-8)
    np.testing.assert_array_equal(a['ring'], [127,0,12,11])
    np.testing.assert_array_equal(a['x'], [1,2,3,4])
    assert [(f.name,f.offset,f.datatype,f.count) for f in out.fields] == [
        ('x',0,7,1),('y',4,7,1),('z',8,7,1),('intensity',12,7,1),('ring',16,4,1),('time',18,7,1)]
    assert out.point_step == 22 and out.row_step == 88 and out.width == 4 and out.height == 1
    assert out.is_dense and not out.is_bigendian
    assert out.header == msg.header
    assert counts == {'input':4,'kept':4,'nonfinite':0,'origin':0,'near':0}


def test_filter_and_time_origin_before_removal():
    msg = cloud([(0,0,0,1,0,100), (.1,0,0,1,1,100.001),
                 (float('nan'),1,1,1,2,100.002), (1,float('inf'),1,1,3,100.003),
                 (1,1,1,float('nan'),4,100.004), (1,1,1,1,5,float('nan')),
                 (.5,0,0,2,6,100.010), (1,0,0,3,7,100.025)])
    out, offset, counts = p.convert_cloud(msg)
    a = np.frombuffer(out.data, dtype=p.OUTPUT_DTYPE)
    assert offset == 100.5
    np.testing.assert_allclose(a['time'], [.010,.025], atol=1e-8)
    np.testing.assert_array_equal(a['ring'], [6,7])
    assert counts == {'input':8,'kept':2,'nonfinite':4,'origin':1,'near':1}


@pytest.mark.parametrize('kind,match', [('missing','Missing'), ('type','Invalid'), ('length','length'),
                                       ('ring','ring outside'), ('duration','Point time'), ('empty','no valid')])
def test_explicit_errors(kind, match):
    msg = cloud([(1,0,0,1,0,100), (2,0,0,1,1,100.01)])
    if kind == 'missing': msg.fields.pop()
    elif kind == 'type': msg.fields[0].datatype = PointField.FLOAT64
    elif kind == 'length': msg.data = msg.data[:-10]
    elif kind == 'ring': msg = cloud([(1,0,0,1,128,100)])
    elif kind == 'duration': msg = cloud([(1,0,0,1,0,100),(1,0,0,1,0,100.1)])
    elif kind == 'empty': msg = cloud([(0,0,0,1,0,100)])
    with pytest.raises(ValueError, match=match): p.convert_cloud(msg)


def test_protect_input_and_existing_output(tmp_path):
    source = tmp_path/'original'
    source.mkdir()
    for target in (source, source/'child', tmp_path):
        with pytest.raises(ValueError, match='disjoint'):
            p.preprocess(source, target, overwrite=True)
    target = tmp_path/'existing'
    target.mkdir()
    with pytest.raises(FileExistsError, match='overwrite'):
        p.preprocess(source, target)


def test_report_handles_empty_rings(tmp_path):
    out, offset, removed = p.convert_cloud(cloud([(1,0,0,1,0,100), (1,1,0,1,0,100.01)]))
    counts = {'/points_raw':2, '/imu_raw':2, '/gps/fix':2}
    stamps = {n:[1_000_000_000,1_010_000_000] for n in counts}
    p.write_report(tmp_path, counts, removed, stamps, [offset,offset],
                   [p.ring_stats(out)], [(37,127,10),(37.00001,127.00001,10)])
    report = (tmp_path/'data_report.md').read_text()
    assert '| 127 | 0 | 0 | N/A | N/A | N/A | 0 |' in report
    assert (tmp_path/'gnss_traj.png').stat().st_size > 0
    assert (tmp_path/'timing.png').stat().st_size > 0


def test_even_odd_selection():
    even = [i for i in range(7) if p.keep_scan(i,2,0)]
    odd = [i for i in range(7) if p.keep_scan(i,2,1)]
    assert even == [0,2,4,6] and odd == [1,3,5]
    assert sorted(even+odd)==list(range(7))
    assert all(p.keep_scan(i,1,0) for i in range(7))
    for n,k in [(0,0),(2,2),(2,-1)]:
        with pytest.raises(ValueError): p.keep_scan(0,n,k)

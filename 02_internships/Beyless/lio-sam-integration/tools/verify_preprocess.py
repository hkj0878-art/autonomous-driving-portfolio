#!/usr/bin/env python3
"""Independent streaming acceptance checks for TASK-02's delivered dataset."""
import hashlib
import json
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Imu, NavSatFix

ROOT = Path(__file__).resolve().parents[1]
EXPECTED = {'/points_raw': 2178, '/imu_raw': 10884, '/gps/fix': 3298}
SAMPLE_INDICES = (0, 1089, 2177)
MAPPING = {'/front/lidar_point_synced': '/points_raw', '/imu/data_synced': '/imu_raw', '/fix_synced': '/gps/fix'}
CLASSES = {'/points_raw': PointCloud2, '/imu_raw': Imu, '/gps/fix': NavSatFix}
OUT_DTYPE = np.dtype({'names':['x','y','z','intensity','ring','time'],
                      'formats':['<f4','<f4','<f4','<f4','<u2','<f4'],
                      'offsets':[0,4,8,12,16,18], 'itemsize':22})


def read(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'), rosbag2_py.ConverterOptions('',''))
    while r.has_next():
        yield r.read_next()


def verify():
    counts = dict.fromkeys(EXPECTED, 0)
    output_hashes = {n: hashlib.sha256() for n in ('/imu_raw','/gps/fix')}
    input_hashes = {n: hashlib.sha256() for n in output_hashes}
    samples, maximum_delta, min_time, max_time = {}, 0, float('inf'), 0
    for name, raw, record_ns in read(ROOT/'data/lio_bag'):
        assert name in EXPECTED, name
        msg = deserialize_message(raw, CLASSES[name])
        header_ns = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
        maximum_delta = max(maximum_delta, abs(record_ns-header_ns))
        assert record_ns == header_ns
        if name in output_hashes:
            output_hashes[name].update(raw)
        else:
            assert msg.header.frame_id == 'front_lidar'
            assert [(f.name,f.offset,f.datatype,f.count) for f in msg.fields] == [
                ('x',0,7,1),('y',4,7,1),('z',8,7,1),('intensity',12,7,1),('ring',16,4,1),('time',18,7,1)]
            assert msg.is_dense and not msg.is_bigendian
            assert msg.point_step == 22 and msg.row_step == msg.width*22 and msg.height == 1
            assert len(msg.data) == msg.row_step
            a = np.frombuffer(msg.data,dtype=OUT_DTYPE)
            for field in ('x','y','z','intensity','time'):
                assert np.isfinite(a[field]).all()
            r2 = sum(a[n].astype(np.float64)**2 for n in ('x','y','z'))
            assert (r2 >= .5**2).all()
            assert (a['ring'] < 128).all()
            assert (a['time'] >= 0).all() and (a['time'] < .05).all()
            min_time = min(min_time, float(a['time'].min()))
            max_time = max(max_time, float(a['time'].max()))
            if counts[name] in SAMPLE_INDICES:
                samples[counts[name]] = (a.copy(), header_ns, msg.header.frame_id)
        counts[name] += 1
    assert counts == EXPECTED, counts
    source_count, source_counts, sample_results = 0, dict.fromkeys(EXPECTED, 0), []
    for name, raw, _ in read(ROOT/'data/synced_bag'):
        out = MAPPING[name]
        source_counts[out] += 1
        if out in input_hashes:
            input_hashes[out].update(raw)
            if out == '/imu_raw':
                assert deserialize_message(raw, Imu).header.frame_id == 'imu_link'
            continue
        if source_count in samples:
            msg = deserialize_message(raw, PointCloud2)
            fields = {f.name:f for f in msg.fields}
            dt = np.dtype({'names':list(fields), 'formats':[
                ('>' if msg.is_bigendian else '<')+('u2' if n=='ring' else 'f8' if n=='timestamp' else 'f4') for n in fields],
                'offsets':[f.offset for f in fields.values()], 'itemsize':msg.point_step})
            a = np.ndarray((msg.height,msg.width),dtype=dt,buffer=msg.data,strides=(msg.row_step,msg.point_step)).reshape(-1)
            finite = np.logical_and.reduce([np.isfinite(a[n]) for n in ('x','y','z','intensity','timestamp')])
            r2 = sum(a[n].astype(np.float64)**2 for n in ('x','y','z'))
            kept = a[finite & (r2 >= .25)]
            converted, header_ns, frame = samples[source_count]
            assert header_ns == msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            assert frame == msg.header.frame_id
            assert len(kept) == len(converted)
            for field in ('x','y','z','intensity','ring'):
                np.testing.assert_array_equal(converted[field], kept[field])
            relative = kept['timestamp'] - a['timestamp'][np.isfinite(a['timestamp'])].min()
            np.testing.assert_array_equal(converted['time'], relative.astype(np.float32))
            sample_results.append({'index': source_count, 'input_points':len(a), 'output_points':len(kept),
                                   'time_min_s':float(converted['time'].min()), 'time_max_s':float(converted['time'].max()),
                                   'fields_order_values_header_preserved':True})
        source_count += 1
    assert source_counts == EXPECTED
    for name in input_hashes:
        assert input_hashes[name].digest() == output_hashes[name].digest(), name
    result = {'counts':counts,'source_counts':source_counts,'max_bag_header_delta_ns':maximum_delta,
              'all_clouds_checked':source_count,'point_time_min_s':min_time,'point_time_max_s':max_time,
              'unchanged_payload_sha256':{n:h.hexdigest() for n,h in output_hashes.items()},'samples':sample_results}
    (ROOT/'results/data_report/verification.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    verify()

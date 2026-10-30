#!/usr/bin/env python3
"""Stream a ROS 2 sqlite bag into LIO-SAM's Velodyne layout.

Run with /opt/ros/humble/setup.bash sourced. Input is opened read-only.
"""
import argparse
from copy import deepcopy
from collections import Counter
import json
from pathlib import Path
import shutil
import tempfile

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2, PointField

MIN_RANGE_M = 0.5
OFFSET_TOLERANCE_S = 0.001
MAX_POINT_TIME_S = 0.05
RING_COUNT = 128
AZIMUTH_GAP_EPS_DEG = 1e-5
WGS84_SEMI_MAJOR_M = 6378137.0
WGS84_ECCENTRICITY_SQUARED = 6.69437999014e-3
OUTPUT_POINT_STEP = 22
NS_PER_SECOND = 1_000_000_000
TOPICS = {
    '/front/lidar_point_synced': ('/points_raw', PointCloud2),
    '/imu/data_synced': ('/imu_raw', Imu),
    '/fix_synced': ('/gps/fix', NavSatFix),
}
INPUT_FIELDS = {'x': (PointField.FLOAT32, 'f4'), 'y': (PointField.FLOAT32, 'f4'),
                'z': (PointField.FLOAT32, 'f4'), 'intensity': (PointField.FLOAT32, 'f4'),
                'ring': (PointField.UINT16, 'u2'), 'timestamp': (PointField.FLOAT64, 'f8')}
OUTPUT_DTYPE = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
                         'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
                         'offsets': [0, 4, 8, 12, 16, 18], 'itemsize': OUTPUT_POINT_STEP})


def stamp_ns(msg):
    return msg.header.stamp.sec * NS_PER_SECOND + msg.header.stamp.nanosec


def point_array(msg):
    fields = {f.name: f for f in msg.fields}
    missing = set(INPUT_FIELDS) - fields.keys()
    if missing:
        raise ValueError(f'Missing PointCloud2 fields: {sorted(missing)}')
    endian = '>' if msg.is_bigendian else '<'
    for name, (datatype, fmt) in INPUT_FIELDS.items():
        f = fields[name]
        if f.datatype != datatype or f.count != 1 or f.offset < 0 or f.offset + np.dtype(fmt).itemsize > msg.point_step:
            raise ValueError(f'Invalid PointCloud2 field: {name}')
    if msg.width == 0 or msg.height == 0:
        raise ValueError('Empty input cloud')
    if msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.height * msg.row_step:
        raise ValueError('Invalid PointCloud2 row_step/data length')
    dtype = np.dtype({'names': list(INPUT_FIELDS),
                      'formats': [endian + v[1] for v in INPUT_FIELDS.values()],
                      'offsets': [fields[n].offset for n in INPUT_FIELDS], 'itemsize': msg.point_step})
    return np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                      strides=(msg.row_step, msg.point_step)).reshape(-1)


def convert_cloud(msg):
    """Return converted cloud, clock offset, and disjoint removal counts.

    Time origin uses all finite input timestamps before removing invalid returns.
    """
    a = point_array(msg)
    finite_time = np.isfinite(a['timestamp'])
    if not finite_time.any():
        raise ValueError('Cloud has no finite point timestamps')
    origin = float(a['timestamp'][finite_time].min())
    finite = finite_time.copy()
    for name in ('x', 'y', 'z', 'intensity'):
        finite &= np.isfinite(a[name])
    zero = finite & (a['x'] == 0) & (a['y'] == 0) & (a['z'] == 0)
    distance2 = sum(a[n].astype(np.float64)**2 for n in ('x', 'y', 'z'))
    near = finite & ~zero & (distance2 < MIN_RANGE_M**2)
    valid = finite & ~zero & ~near
    kept = a[valid]
    if not len(kept):
        raise ValueError('Cloud has no valid points after filtering')
    if np.any(kept['ring'] >= RING_COUNT):
        raise ValueError('ring outside [0, 127]')
    out = np.empty(len(kept), dtype=OUTPUT_DTYPE)
    for name in ('x', 'y', 'z', 'intensity', 'ring'):
        out[name] = kept[name]
    out['time'] = kept['timestamp'] - origin
    if np.any(out['time'] < 0) or np.any(out['time'] >= MAX_POINT_TIME_S):
        raise ValueError('Point time outside [0, 0.05) seconds')
    result = PointCloud2(header=deepcopy(msg.header), height=1, width=len(out),
                        fields=[PointField(name=n, offset=OUTPUT_DTYPE.fields[n][1],
                                           datatype=PointField.UINT16 if n == 'ring' else PointField.FLOAT32, count=1)
                                for n in OUTPUT_DTYPE.names],
                        is_bigendian=False, point_step=OUTPUT_POINT_STEP,
                        row_step=OUTPUT_POINT_STEP*len(out), data=out.tobytes(), is_dense=True)
    result.header.frame_id = 'front_lidar'
    counts = {'input': len(a), 'kept': len(out), 'nonfinite': int((~finite).sum()),
              'origin': int(zero.sum()), 'near': int(near.sum())}
    return result, stamp_ns(msg)/NS_PER_SECOND - origin, counts


def reader(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    return r


def ring_stats(cloud):
    a = np.frombuffer(cloud.data, dtype=OUTPUT_DTYPE)
    stats = []
    for ring in range(RING_COUNT):
        p = a[a['ring'] == ring]
        az = np.degrees(np.arctan2(p['y'], p['x']))
        gaps = np.diff(np.sort(az))
        gaps = gaps[gaps > AZIMUTH_GAP_EPS_DEG]
        corr = float(np.corrcoef(az, p['time'])[0, 1]) if len(p) > 1 and np.std(az) > 0 and np.std(p['time']) > 0 else None
        stats.append((len(p), float(np.median(gaps)) if len(gaps) else None,
                      float(np.ptp(az)) if len(p) else None, corr,
                      int(np.sum(np.diff(p['time']) < 0))))
    return stats


def write_report(directory, counts, removed, stamps, offsets, rings, gnss):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    directory.mkdir(parents=True, exist_ok=True)
    o = np.array(offsets)
    stats = {'message_counts': dict(counts), 'points': dict(removed),
             'offset_mean_s': float(o.mean()), 'offset_std_s': float(o.std()),
             'offset_max_deviation_s': float(np.max(np.abs(o-o.mean()))),
             'offset_span_s': float(np.ptp(o)), 'rings': []}
    rows = []
    for r in range(RING_COUNT):
        samples = [scan[r] for scan in rings]
        def median(i):
            values = [s[i] for s in samples if s[i] is not None]
            return float(np.median(values)) if values else None
        entry = {'ring': r, 'total_points': sum(s[0] for s in samples),
                 'points_per_scan_median': median(0), 'azimuth_step_deg_median': median(1),
                 'azimuth_span_deg_median': median(2), 'azimuth_time_correlation_median': median(3),
                 'time_reversals': sum(s[4] for s in samples)}
        stats['rings'].append(entry)
        def display(i, precision):
            value = median(i)
            return f'{value:.{precision}f}' if value is not None else 'N/A'
        rows.append(f"| {r} | {entry['total_points']} | {display(0,0)} | {display(1,6)} | {display(2,3)} | {display(3,5)} | {entry['time_reversals']} |")
    resolution = np.median([r['azimuth_step_deg_median'] for r in stats['rings'] if r['azimuth_step_deg_median'] is not None])
    empty_rings = [r['ring'] for r in stats['rings'] if r['total_points'] == 0]
    correlations = [r['azimuth_time_correlation_median'] for r in stats['rings'] if r['azimuth_time_correlation_median'] is not None]
    report = f'''# TASK-02 데이터 리포트

원본은 rosbag2_py SequentialReader로 읽고 한 메시지씩 변환했다. IMU/GNSS는 CDR 바이트를 유지하고 토픽명과 bag 기록시각만 변경했다. 모든 출력 기록시각은 header stamp의 정수 나노초다.

- 메시지 수: {dict(counts)}
- 포인트 집계: {dict(removed)}
- 제거 비율: {100*(removed['input']-removed['kept'])/removed['input']:.4f}% (nonfinite → origin → near 순으로 상호 배타적 집계)
- offset = header stamp − 전체 입력의 유한한 timestamp 최솟값
- offset 평균 {o.mean():.9f} s, 표준편차 {o.std():.9f} s, 평균 대비 최대편차 {np.max(np.abs(o-o.mean())):.9f} s, 전체 범위 {np.ptp(o):.9f} s
- 안전 기준: 최초 offset 대비 편차 및 전체 범위가 1 ms 이상이면 변환 중단. 이 데이터는 통과했다.
- 상대 time은 무효점을 제거하기 전의 최솟값 기준이며 float32로 저장한다. 포인트 순서와 ring을 유지한다.
- 출력 필드: x@0,y@4,z@8,intensity@12 float32; ring@16 uint16; time@18 float32, point_step=22, little endian, height=1, is_dense=True.

## 스캔 패턴과 Horizon_SCAN 참고값

전체 스캔/ring별 유효점 방위각을 정렬하여 양의 인접 차이(중복 방위각 제외)의 중앙값을 구했다. 아래 표의 해상도·방위각 폭·시간 상관계수·포인트 수는 스캔별 값의 중앙값이며 총점 수와 시간 역전 수는 합계다. 유효점이 없는 ring의 해상도·상관계수는 N/A이며 대표 해상도 집계에서 제외한다. 환경에 따른 결측으로 간격이 커질 수 있으므로 표는 센서 스펙의 단정이 아니라 실측값이다.

전체 구간에서 유효점이 없는 ring: {empty_rings}. 유효 ring의 시간/방위각 상관계수 중앙값은 {np.median(correlations):.6f}, 전체 시간 역전 수는 {sum(r['time_reversals'] for r in stats['rings'])}이다.

대표 방위각 간격은 {resolution:.6f}°이며, 360° 격자를 쓰는 LIO-SAM의 Horizon_SCAN 참고값은 360/해상도 = {360/resolution:.2f} (반올림 {round(360/resolution)})이다. 실측 오차를 고려하면 명목상 0.2°/약 1800칸에 해당하며, TASK-03에서 설정을 결정할 때 참고한다. 링당 실제 유효점 개수와 Horizon_SCAN은 서로 다른 값이다. 전방 FOV의 일부만 채워진다. 방위각과 시간의 Pearson 상관계수 부호는 스캔 진행 방향을 나타내며 절댓값이 1에 가까우면 거의 선형이다. 시간 역전은 저장된 원래 링 내 순서에서 집계했다.

| ring | 전체 유효점 | 스캔당 점 중앙값 | 방위각 간격 ° | 방위각 폭 ° | azimuth/time r | 시간 역전 수 |
|---|---:|---:|---:|---:|---:|---:|
''' + '\n'.join(rows) + '\n\n## 토픽별 header stamp 간격\n\n'
    fig, ax = plt.subplots(figsize=(9, 4))
    for topic, values in stamps.items():
        ns = np.array(values, dtype=np.int64)
        dt = np.diff(ns)*1e-9
        report += f'- {topic}: {len(ns)}개, 간격 median={np.median(dt):.6f} s, min={dt.min():.6f}, max={dt.max():.6f}, 역전={int((dt<0).sum())}\n'
        ax.plot((ns[1:]-ns[0])*1e-9, dt*1000, '.', markersize=1, label=topic)
    ax.set(xlabel='Time since first header stamp (s)', ylabel='Header stamp interval (ms)')
    ax.legend(); fig.tight_layout(); fig.savefig(directory/'timing.png', dpi=160); plt.close(fig)
    # WGS84 geodetic -> ECEF -> ENU, first GNSS fix as origin.
    fixes = np.array(gnss)
    lat, lon = np.radians(fixes[:, 0]), np.radians(fixes[:, 1])
    altitude = fixes[:, 2]
    semi_major, eccentricity2 = WGS84_SEMI_MAJOR_M, WGS84_ECCENTRICITY_SQUARED
    n = semi_major / np.sqrt(1-eccentricity2*np.sin(lat)**2)
    xyz = np.column_stack(((n+altitude)*np.cos(lat)*np.cos(lon), (n+altitude)*np.cos(lat)*np.sin(lon), (n*(1-eccentricity2)+altitude)*np.sin(lat)))
    la, lo = lat[0], lon[0]
    rot = np.array([[-np.sin(lo),np.cos(lo),0],[-np.sin(la)*np.cos(lo),-np.sin(la)*np.sin(lo),np.cos(la)],[np.cos(la)*np.cos(lo),np.cos(la)*np.sin(lo),np.sin(la)]])
    enu = (xyz-xyz[0]) @ rot.T
    fig, ax = plt.subplots(figsize=(6, 6)); ax.plot(enu[:,0],enu[:,1],linewidth=1)
    ax.scatter(*enu[0,:2],label='Start'); ax.scatter(*enu[-1,:2],label='End')
    ax.set(xlabel='East (m)', ylabel='North (m)', title='GNSS trajectory (WGS84 ENU)'); ax.axis('equal'); ax.grid(); ax.legend()
    fig.tight_layout(); fig.savefig(directory/'gnss_traj.png',dpi=160); plt.close(fig)
    report += f"\nGNSS WGS84 ENU 원점: lat={fixes[0,0]:.9f}, lon={fixes[0,1]:.9f}, altitude={fixes[0,2]:.3f} m. 3D 누적거리={np.linalg.norm(np.diff(enu,axis=0),axis=1).sum():.3f} m, 시작–끝={np.linalg.norm(enu[-1]):.3f} m.\n"
    (directory/'data_report.md').write_text(report)
    (directory/'statistics.json').write_text(json.dumps(stats, indent=2, allow_nan=False)+'\n')


def keep_scan(index, keep_every, phase):
    if keep_every < 1 or not 0 <= phase < keep_every:
        raise ValueError('Require keep-every >= 1 and 0 <= phase < keep-every')
    return index % keep_every == phase


def preprocess(source, destination, overwrite=False, report_dir=None, keep_every=1, phase=0):
    keep_scan(0, keep_every, phase)
    source, destination = Path(source).resolve(), Path(destination).resolve()
    report_dir = Path(report_dir).resolve() if report_dir else Path(__file__).resolve().parents[1]/'results/data_report'
    # Protect input, including ancestor/descendant output paths, from overwrite.
    for target in (destination, report_dir):
        if target == source or target in source.parents or source in target.parents:
            raise ValueError('Output/report path must be disjoint from input bag')
    if destination == report_dir or destination in report_dir.parents or report_dir in destination.parents:
        raise ValueError('Output bag and report directory must be disjoint')
    if destination.exists() and not overwrite:
        raise FileExistsError(f'{destination} exists; pass --overwrite')
    r = reader(source)
    metadata = {t.name: t for t in r.get_all_topics_and_types()}
    for name, (_, cls) in TOPICS.items():
        if name not in metadata:
            raise ValueError(f'Missing topic: {name}')
        expected = 'sensor_msgs/msg/'+cls.__name__
        if metadata[name].type != expected:
            raise ValueError(f'Unexpected type for {name}: {metadata[name].type}')
    destination.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=destination.name+'.', dir=destination.parent))
    stage.rmdir()  # rosbag writer requires a nonexistent URI
    writer = None
    try:
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(stage), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
        for name, (out, _) in TOPICS.items():
            m = metadata[name]
            writer.create_topic(rosbag2_py.TopicMetadata(name=out, type=m.type, serialization_format='cdr', offered_qos_profiles=m.offered_qos_profiles))
        counts, removed = Counter(), Counter()
        stamps = {out: [] for out, _ in TOPICS.values()}
        offsets, rings, gnss = [], [], []
        scan_index = 0
        while r.has_next():
            name, data, _ = r.read_next()
            if name not in TOPICS:
                continue
            out, cls = TOPICS[name]
            msg = deserialize_message(data, cls)
            stamp = stamp_ns(msg)
            if cls is PointCloud2:
                selected = keep_scan(scan_index, keep_every, phase)
                scan_index += 1
                if not selected:
                    continue
                converted, offset, stats = convert_cloud(msg)
                offsets.append(offset)
                if abs(offset-offsets[0]) >= OFFSET_TOLERANCE_S or max(offsets)-min(offsets) >= OFFSET_TOLERANCE_S:
                    raise ValueError(f'Point timestamp offset drift >= 1 ms at scan {len(offsets)}: first={offsets[0]}, current={offset}, min={min(offsets)}, max={max(offsets)}')
                removed.update(stats); rings.append(ring_stats(converted))
                data = serialize_message(converted)
                if len(offsets) % 250 == 0:
                    print(f'Converted {len(offsets)} scans', flush=True)
            elif cls is NavSatFix:
                fix = (msg.latitude, msg.longitude, msg.altitude)
                if not np.all(np.isfinite(fix)):
                    raise ValueError('Nonfinite GNSS fix')
                gnss.append(fix)
            counts[out] += 1; stamps[out].append(stamp)
            writer.write(out, data, stamp)
        if any(not counts[out] for out, _ in TOPICS.values()):
            raise ValueError('Required topic has no messages')
        del writer
        writer = None
        write_report(report_dir, counts, removed, stamps, offsets, rings, gnss)
        if destination.exists():
            shutil.rmtree(destination)
        stage.rename(destination)
        (report_dir/'preprocess_error.md').unlink(missing_ok=True)
        print(json.dumps(dict(counts)), flush=True)
    except Exception as exc:
        report_dir.mkdir(parents=True, exist_ok=True)
        (report_dir/'preprocess_error.md').write_text(f'# Preprocessing stopped\n\n{type(exc).__name__}: {exc}\n')
        raise
    finally:
        if writer is not None:
            del writer
        if stage.exists():
            shutil.rmtree(stage)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--in', dest='source', required=True)
    parser.add_argument('--out', dest='destination', required=True)
    parser.add_argument('--overwrite', action='store_true')
    parser.add_argument('--keep-every', type=int, default=1)
    parser.add_argument('--phase', type=int, default=0, help='Zero-based scan index modulo keep-every')
    parser.add_argument('--report-dir')
    args = parser.parse_args()
    if args.keep_every != 1 and args.report_dir is None:
        args.report_dir = str(Path(args.destination).resolve().parent.parent/'results'/('data_report_'+Path(args.destination).name))
    preprocess(args.source, args.destination, args.overwrite, args.report_dir, args.keep_every, args.phase)


if __name__ == '__main__':
    main()

"""Extract quality-filtered GNSS positions; all products live under results/eval."""
import argparse
import json
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import NavSatFix

A = 6378137.0
E2 = 6.6943799901413165e-3
MAX_SIGMA_M = 0.05


def bag_reader(path, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    present = {t.name for t in reader.get_all_topics_and_types()}
    if not set(topics) <= present:
        raise ValueError(f'Missing bag topics: {set(topics)-present}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader


def ecef(lla):
    lat, lon = np.radians(lla[:, :2]).T
    h = lla[:, 2]
    n = A / np.sqrt(1-E2*np.sin(lat)**2)
    return np.column_stack(((n+h)*np.cos(lat)*np.cos(lon),
                            (n+h)*np.cos(lat)*np.sin(lon), (n*(1-E2)+h)*np.sin(lat)))


def to_enu(lla, origin):
    lat, lon = np.radians(origin[:2])
    matrix = np.array([[-np.sin(lon), np.cos(lon), 0],
                       [-np.sin(lat)*np.cos(lon), -np.sin(lat)*np.sin(lon), np.cos(lat)],
                       [np.cos(lat)*np.cos(lon), np.cos(lat)*np.sin(lon), np.sin(lat)]])
    return (ecef(np.asarray(lla))-ecef(np.asarray(origin)[None])) @ matrix.T


def extract(bag, out, sigma_axes="xy"):
    if sigma_axes not in ("xy", "xyz"): raise ValueError("sigma_axes must be xy or xyz")
    axis_count = 2 if sigma_axes == "xy" else 3
    reader = bag_reader(bag, ['/gps/fix'])
    rows = []; stats = dict(total=0, rejected_status=0, rejected_covariance=0, rejected_invalid=0)
    origin = None
    vertical_sigmas=[]
    while reader.has_next():
        _, raw, _ = reader.read_next()
        m = deserialize_message(raw, NavSatFix)
        stats['total'] += 1
        lla = [m.latitude, m.longitude, m.altitude]
        if origin is None:
            if not np.isfinite(lla).all():
                raise ValueError('First fix cannot define a finite ENU origin')
            origin = lla
        if m.status.status < 2:
            stats['rejected_status'] += 1; continue
        covariance = np.array(m.position_covariance).reshape(3, 3)
        if np.isfinite(covariance[2,2]) and covariance[2,2]>=0:
            vertical_sigmas.append(float(np.sqrt(covariance[2,2])))
        if (m.position_covariance_type == 0 or not np.isfinite(covariance).all()
                or (np.diag(covariance) < 0).any()
                or np.sqrt(np.diag(covariance)[:axis_count]).max() > MAX_SIGMA_M):
            stats['rejected_covariance'] += 1; continue
        if not np.isfinite(lla).all():
            stats['rejected_invalid'] += 1; continue
        rows.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9, *lla])
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    stats.update(accepted=len(rows), origin_lla=origin, max_sigma_m=MAX_SIGMA_M, sigma_axes=sigma_axes,
                 vertical_sigma_m=dict(count=len(vertical_sigmas), min=min(vertical_sigmas,default=None),
                 max=max(vertical_sigmas,default=None), mean=float(np.mean(vertical_sigmas)) if vertical_sigmas else None))
    (out/'gt_gnss_metadata.json').write_text(json.dumps(stats, indent=2))
    rows = np.array(rows)
    if len(rows) < 3 or (np.diff(rows[:, 0]) <= 0).any():
        raise ValueError('Insufficient or nonmonotonic GNSS fixes')
    gt = np.column_stack((rows[:, 0], to_enu(rows[:, 1:], origin), np.tile([0,0,0,1], (len(rows),1))))
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    np.savetxt(out/'gt_gnss_enu_tum.txt', gt, fmt='%.9f', header='stamp E N U qx qy qz qw (orientation placeholder)')
    stats.update(accepted=len(gt), origin_lla=origin, max_sigma_m=MAX_SIGMA_M,
                 orientation='placeholder identity; RPE derives yaw from GNSS tangent')
    (out/'gt_gnss_metadata.json').write_text(json.dumps(stats, indent=2))
    return gt, stats


if __name__ == '__main__':
    p=argparse.ArgumentParser(); p.add_argument('--bag', required=True); p.add_argument('--out', required=True)
    p.add_argument('--sigma-axes', choices=['xy','xyz'], default='xy')
    a=p.parse_args(); extract(a.bag, a.out, a.sigma_axes)

"""Independent acceptance audit. Only writes results/loc/verification.json."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from tools.eval.gnss_gt import bag_reader
from tools.eval.traj_eval import load_tum
from .localize import Localizer, cloud, output_dir, query_scans, poses_at


def scan_stamps(path):
    r=bag_reader(path,['/points_raw']); times=[]
    while r.has_next():
        _,raw,_=r.read_next(); msg=deserialize_message(raw,PointCloud2)
        times.append(msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec)
    return np.array(times,dtype=np.int64)


def main():
    p=argparse.ArgumentParser(); p.add_argument('--out',default='results/loc')
    a=p.parse_args(); out=output_dir(a.out)
    before=json.loads((out/'input_manifest_before.json').read_text())
    current_paths=list(Path('data').rglob('*'))+[p for p in Path('results').rglob('*') if not p.is_relative_to('results/loc')]
    after={str(p):[p.stat().st_size,p.stat().st_mtime_ns] for p in current_paths if p.is_file()}
    assert before==after,'Read-only input file set, size or mtime changed'
    original=scan_stamps('data/lio_bag'); odd=scan_stamps('data/lio_bag_odd'); even=scan_stamps('data/lio_bag_even')
    np.testing.assert_array_equal(original[::2],even)
    np.testing.assert_array_equal(original[1::2],odd)
    assert not len(np.intersect1d(odd,even))
    traj=load_tum(out/'loc_traj_tum.txt')
    assert len(traj)==len(odd)==1089
    np.testing.assert_allclose(traj[:,0],odd/1e9,atol=1e-6,rtol=0)
    with (out/'per_scan.csv').open() as f: rows=list(csv.DictReader(f))
    assert len(rows)==len(odd) and all(r['lost']=='False' for r in rows)
    for r in rows:
        success=(float(r['fitness'])>=.3 and float(r['inlier_rmse_m'])<=.3)
        assert (r['accepted']=='True')==success
        assert 0<int(r['iterations'])<=60
    with (out/'robustness.csv').open() as f: robust=list(csv.DictReader(f))
    keys={(int(r['level']),int(r['start']),int(r['seed'])) for r in robust}
    assert keys=={(i,j,k) for i in range(4) for j in range(5) for k in range(10)}
    for r in robust:
        success=(int(r['completed_scans'])==30 and r['lost']=='False' and
                 float(r['position_error_m'])<.3 and float(r['yaw_error_deg'])<1)
        assert (r['success']=='True')==success
    stats=json.loads((out/'run_stats.json').read_text()); assert stats['peak_rss_mib']<4096
    for name in ('localization','mapping_baseline','localization_common_timestamps','mapping_common_timestamps'):
        m=json.loads((out/name/'metrics.json').read_text())
        assert m['matched_count']>3 and m['se3']['ate_2d_m']['rmse']>=0
        assert m['evo_crosscheck']['absolute_difference_m']<.001
    from PIL import Image
    for name in ('loc_traj_overlay','loc_error_vs_time','robustness_heatmap','fitness_vs_time'):
        with Image.open(out/'figs'/f'{name}.png') as im:
            assert im.width>500 and im.height>300 and np.asarray(im).std()>5
    # Public API versus cached implementation on three actual held-out scans.
    target=cloud(np.load(out/'prior_map.npz')['xyz']); loc=Localizer(target)
    mapping=load_tum('results/even_loop_on/keyframes_final_tum.txt'); checks=[]
    for i,(t,source,_) in enumerate(query_scans('data/lio_bag_odd')):
        if i not in (20,500,1000): continue
        initial=poses_at(mapping,[t])[0]
        cached,detail=loc.register(source,initial); public=initial
        for distance in loc.cfg.correspondence_m:
            result=o3d.pipelines.registration.registration_generalized_icp(
                source,target,distance,public,loc.estimator,
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=loc.cfg.max_iterations))
            public=result.transformation
        pos=float(np.linalg.norm(cached[:3,3]-public[:3,3]))
        angle=float(Rotation.from_matrix(cached[:3,:3]@public[:3,:3].T).magnitude())
        assert pos<1e-5 and angle<1e-5,(pos,angle)
        checks.append(dict(scan=i,position_difference_m=pos,rotation_difference_rad=angle,
                           fitness_difference=abs(detail['fitness']-result.fitness),
                           rmse_difference_m=abs(detail['inlier_rmse_m']-result.inlier_rmse)))
        if i==1000: break
    audit=dict(passed=True,input_files_unchanged=len(before),input_protection_check='file set + size + mtime_ns',
               even_scans=len(even),odd_scans=len(odd),split_disjoint=True,
               baseline_scans=len(rows),lost_count=0,robustness_trials=len(keys),
               peak_rss_mib=stats['peak_rss_mib'],real_scan_public_api_crosschecks=checks)
    (out/'verification.json').write_text(json.dumps(audit,indent=2))
    print(json.dumps(audit,indent=2))

if __name__=='__main__': main()

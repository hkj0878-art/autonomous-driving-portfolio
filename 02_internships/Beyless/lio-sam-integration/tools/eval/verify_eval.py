"""Independent artifact/criterion audit; including rev2 yaw-body lever, RPE noise and loop consistency."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image

RUNS=[f'loop_{mode}_r{i}' for mode in ('on','off') for i in (1,2,3)]+['even_loop_on']


def verify(root):
    out=root/'results/eval'; checks={}
    for run in RUNS:
        for kind in ('final','online'):
            directory=out/run/kind
            m=json.loads((directory/'metrics.json').read_text())
            assert m['matched_count']>=3
            for key in ('se3','se3_lever'):
                for dimension in ('ate_2d_m','ate_3d_m'):
                    assert all(np.isfinite(v) and v>=0 for v in m[key][dimension].values())
                assert all(np.isfinite(v) and v>=0 for v in m[key]['ate_m'].values())
                r=np.array(m[key]['R']);np.testing.assert_allclose(r.T@r,np.eye(3),atol=1e-8)
                assert np.linalg.det(r)>0.999999
            assert m['se3_lever']['diagnostic']['dof']==2
            assert m['se3_lever']['lever_arm_body_m'][2]==0
            assert m['se3_lever']['lever_norm_m']<3
            assert m['rpe']['10']['yaw_deg_per_100m'] is None
            assert m['rpe']['100']['translation_percent']['mean']<2
            assert 'se3_lever_3d_diagnostic' in m and 'rpe_se3_diagnostic' in m
            assert m['evo_crosscheck']['absolute_difference_m']<.001
            error=np.loadtxt(directory/'error_vs_time.csv',delimiter=',',skiprows=1)
            assert len(error)==m['matched_count'] and (np.diff(error[:,0])>0).all()
            for key,column3,column2 in [('se3',1,3),('se3_lever',2,4)]:
                assert abs(np.sqrt(np.mean(error[:,column3]**2))-m[key]['ate_3d_m']['rmse'])<1e-8
                assert abs(np.sqrt(np.mean(error[:,column2]**2))-m[key]['ate_2d_m']['rmse'])<1e-8
            for length in ('10','25','50','100'):assert m['rpe'][length]['count']>0
        q=json.loads((out/run/'map_quality.json').read_text())
        assert np.isfinite(q['mme_nats']) and q['mpv_m2']>0
        assert q['overlap']['median_m']<=q['overlap']['p95_m']<=1
        assert 0<=q['overlap']['double_wall_ratio']<=1
        clouds=np.load(out/run/'overlap_clouds.npz')
        for name in ('first','last'):
            a=clouds[name];assert a.shape[1]==6 and np.isfinite(a).all()
            assert (a[:,3]>=0).all() and (a[:,3]<.05).all()
    checks['fourteen_metrics_and_seven_maps']=True
    for filename,count in [('summary.csv',14),('map_quality.csv',7)]:
        with (out/filename).open() as f:assert len(list(csv.DictReader(f)))==count
    assert (out/'summary.md').stat().st_size>1000
    figures={}
    for path in (out/'figs').glob('*.png'):
        a=np.array(Image.open(path).convert('RGB'))
        assert min(a.shape[:2])>=600 and a.std()>5
        figures[path.name]=list(a.shape[:2])
    assert len(figures)==12
    checks['twelve_nonempty_figures']=figures
    manifest=json.loads((out/'source_manifest_before.json').read_text())
    for relative,checksum in manifest.items():
        assert hashlib.sha256((root/relative).read_bytes()).hexdigest()==checksum,relative
    checks['protected_sources_unchanged']=True
    with (out/'loop_validation.csv').open() as f:loops=list(csv.DictReader(f))
    gt=np.loadtxt(out/'gt_gnss_enu_tum.txt')
    gnss=json.loads((out/'gt_gnss_metadata.json').read_text())
    assert gnss['sigma_axes']=='xy' and gnss['accepted']==len(gt)
    assert gnss['accepted']+gnss['rejected_status']+gnss['rejected_covariance']+gnss['rejected_invalid']==gnss['total']
    checks['horizontal_gnss_filter_verified']=True
    pair_count=0
    for run in RUNS:
        pairs=json.loads((root/'results'/run/'loop_pairs.json').read_text())['pairs']
        poses=np.loadtxt(root/'results'/run/'keyframes_final_tum.txt')
        rows=[r for r in loops if r['run']==run]
        if run.startswith('loop_off'):
            assert not pairs and len(rows)==1 and rows[0]['classification']=='NOT_APPLICABLE'
            continue
        assert len(pairs)>=1 and len(rows)==len(pairs)
        pair_count+=len(pairs)
        for pair,row in zip(pairs,rows):
            nearest=np.linalg.norm(np.array(pair['endpoints'])[:,None,:]-poses[None,:,1:4],axis=2)
            indices=np.argmin(nearest,axis=1)
            np.testing.assert_array_equal(indices,pair['keyframe_indices'])
            np.testing.assert_allclose(nearest[np.arange(2),indices],pair['matching_distances_m'])
            assert max(pair['matching_distances_m'])<=.2 and indices[0]!=indices[1]
            stamps=poses[indices,0]
            assert np.all((stamps>=gt[0,0])&(stamps<=gt[-1,0]))
            positions=np.column_stack([np.interp(stamps,gt[:,0],gt[:,axis]) for axis in (1,2)])
            distance=float(np.linalg.norm(positions[0]-positions[1]))
            assert abs(distance-float(row['gt_distance_m']))<1e-8
            gt3=np.column_stack([np.interp(stamps,gt[:,0],gt[:,axis]) for axis in (1,2,3)])
            alignment=json.loads((out/run/'align_se3.json').read_text())
            predicted=poses[indices,1:4]@np.asarray(alignment['R']).T+np.asarray(alignment['t'])
            delta=float(np.linalg.norm((predicted[0]-predicted[1])-(gt3[0]-gt3[1])))
            assert abs(delta-float(row['relative_position_error_m']))<1e-8
            assert row['classification']==('CONSISTENT(TP)' if delta<.5 else 'INCONSISTENT')
    ranked=sorted((json.loads((out/run/'final/metrics.json').read_text())['se3_lever']['ate_2d_m']['rmse'],run)
                  for run in RUNS if run.startswith('loop_on_r'))
    representative=ranked[1][1]
    assert f'**{representative}**' in (out/'summary.md').read_text()
    alignment=json.loads((out/representative/'align_se3.json').read_text())
    matrix=np.array(alignment['matrix_4x4']);assert matrix.shape==(4,4)
    np.testing.assert_allclose(matrix[3],[0,0,0,1])
    metrics=json.loads((out/representative/'final/metrics.json').read_text())
    np.testing.assert_allclose(matrix[:3,:3],metrics['se3']['R'])
    np.testing.assert_allclose(matrix[:3,3],metrics['se3']['t'])
    checks['representative_run']=representative
    checks['actual_loop_pairs_verified']=True
    checks['loop_pair_count']=pair_count
    with (out/'rpe_noise_floor.csv').open() as f: noise=list(csv.DictReader(f))
    assert len(noise)==49
    for row in noise:
        assert int(row['trials'])==10 and float(row['sigma_m'])==.014
        assert float(row['clean_gt_mean'])<1e-10 and float(row['noise_mean'])>0
        assert int(row['segment_m'])>=25 or row['metric']=='translation_percent'
    # Independent recomputation of the primary antenna trajectory and RPE,
    # not merely checking that summary repeats the metrics file.
    from .traj_eval import load_tum, associate, rpe, EvalConfig
    from scipy.spatial.transform import Rotation
    maximum=0.;lever_norms=[]
    for run in RUNS:
        for kind, filename in [('final','keyframes_final_tum.txt'),('online','odom_online_tum.txt')]:
            m=json.loads((out/run/kind/'metrics.json').read_text())
            est=load_tum(root/'results'/run/filename); matched, y, _=associate(est,gt)
            r=np.asarray(m['se3_lever']['R']);t=np.asarray(m['se3_lever']['t'])
            rot=r@Rotation.from_quat(matched[:,4:]).as_matrix()
            yaw=np.arctan2(rot[:,1,0],rot[:,0,0]);l=np.asarray(m['se3_lever']['lever_arm_body_m'])
            offset=np.column_stack((np.cos(yaw)*l[0]-np.sin(yaw)*l[1],
                                   np.sin(yaw)*l[0]+np.cos(yaw)*l[1],np.zeros(len(yaw))))
            antenna=matched[:,1:4]@r.T+t+offset
            error=antenna-y
            assert abs(np.sqrt(np.mean(np.sum(error[:,:2]**2,axis=1)))-m['se3_lever']['ate_2d_m']['rmse'])<1e-9
            calculated=rpe(antenna,rot,y,EvalConfig())
            for length in ('10','25','50','100'):
                for field in ('translation_percent','yaw_deg_per_100m'):
                    if length=='10' and field.startswith('yaw'):continue
                    # Saved GT is rounded to 9 decimals; allow its propagated tangent rounding.
                    assert abs(calculated[length][field]['mean']-m['rpe'][length][field]['mean'])<1e-7
            maximum=max(maximum,m['rpe']['100']['translation_percent']['mean'])
            lever_norms.append(m['se3_lever']['lever_norm_m'])
    with (out/'group_summary.csv').open() as f: groups=list(csv.DictReader(f))
    for group in ('loop_on','loop_off'):
        for kind in ('final','online'):
            levers=np.array([json.loads((out/f'{group}_r{i}'/kind/'metrics.json').read_text())['se3_lever']['lever_arm_body_m'] for i in (1,2,3)])
            for axis,field in [(0,'lever_x_m'),(1,'lever_y_m')]:
                row=next(r for r in groups if r['group']==group and r['trajectory']==kind and r['metric']==field)
                assert abs(float(row['std_sample'])-np.std(levers[:,axis],ddof=1))<1e-9
    checks['rev2_noise_floor_trials_verified']=True
    checks['rev2_primary_rpe_recomputed']=True
    checks['max_rpe_100m_percent_all_fourteen']=maximum
    checks['max_lever2d_norm_m_all_fourteen']=max(lever_norms)
    checks['lever2d_group_std_verified']=True
    checks['loop_consistency_threshold_m']=.5
    checks['overall_completion']='PASS' 
    (out/'independent_verification.json').write_text(json.dumps(checks,indent=2))
    return checks


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2]);a=p.parse_args()
    print(json.dumps(verify(a.root),indent=2))

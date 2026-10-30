"""Reproducible full-sequence and 200-trial held-out localization benchmark.

Run from the repository root after source env.sh, with .venv/bin/python -m.
All generated products, caches, evaluation and resumable trial data stay in loc.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
from time import perf_counter
import resource
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import open3d as o3d
from scipy.spatial.transform import Rotation
from tools.eval.build_map_from_poses import build_map
from tools.eval.traj_eval import evaluate, load_tum
from .localize import (Config, Localizer, cloud, output_dir, poses_at, query_scans,
                       track, tum_row, write_csv)

LEVELS=((.5,2.),(1.,5.),(2.,10.),(3.,15.))
SEEDS=10
STARTS=5
FAIRNESS='동일 주행의 held-out(홀수) 스캔 → 진짜 재주행보다 낙관적. 다른 날 주행 bag으로 같은 스크립트를 재사용 가능'


def save_json(path,value):
    Path(path).write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False))


def fingerprint(paths):
    return {str(p):hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in paths}


def make_plots(out, data, baseline_data, scans, robust):
    figs=out/'figs'; figs.mkdir(exist_ok=True)
    def save(name):
        plt.tight_layout(); plt.savefig(figs/name,dpi=200); plt.close()
    plt.figure(figsize=(7,6)); plt.plot(*data['gt'][:,:2].T,color='black',label='RTK GNSS')
    plt.plot(*data['aligned'][:,:2].T,color='#0072B2',lw=1,label='Localization SE3')
    plt.plot(*data['antenna'][:,:2].T,color='#D55E00',lw=1,label='Localization SE3+lever2D')
    plt.plot(*baseline_data['antenna'][:,:2].T,color='#009E73',lw=1,label='Even mapping SE3+lever2D')
    plt.axis('equal'); plt.xlabel('East (m)'); plt.ylabel('North (m)'); plt.title('Held-out localization vs GNSS'); plt.legend(fontsize=8)
    save('loc_traj_overlay.png')
    plt.figure(figsize=(9,4)); plt.plot(data['t']-data['t'][0],data['error'],color='#0072B2',label='Localization')
    plt.plot(baseline_data['t']-data['t'][0],baseline_data['error'],color='#D55E00',label='Even mapping')
    plt.xlabel('Elapsed time (s)'); plt.ylabel('2D error (m)'); plt.legend(); plt.title('SE3+lever2D horizontal error')
    save('loc_error_vs_time.png')
    plt.figure(figsize=(9,4)); t=np.array([r['timestamp'] for r in scans]); t-=t[0]
    plt.plot(t,[r['fitness'] for r in scans],color='#0072B2',label='Fitness')
    plt.plot(t,[r['inlier_rmse_m'] for r in scans],color='#D55E00',label='Inlier RMSE (m)')
    plt.axhline(.3,color='black',ls='--',lw=.8,label='Acceptance thresholds (0.3)')
    plt.xlabel('Elapsed time (s)'); plt.ylabel('Fitness / RMSE (m)'); plt.title('Fine-stage GICP quality'); plt.legend()
    save('fitness_vs_time.png')
    heat=np.array([[np.mean([r['success'] for r in robust if r['level']==i and r['start']==j])*100
                    for j in range(STARTS)] for i in range(len(LEVELS))])
    plt.figure(figsize=(8,4)); plt.imshow(heat,vmin=0,vmax=100,cmap='cividis',aspect='auto')
    plt.xticks(range(STARTS),[f'Start {j+1}' for j in range(STARTS)])
    plt.yticks(range(len(LEVELS)),[f'{m:g} m / {deg:g} deg' for m,deg in LEVELS])
    for i in range(len(LEVELS)):
        for j in range(STARTS): plt.text(j,i,f'{heat[i,j]:.0f}%',ha='center',va='center',color='white' if heat[i,j]<50 else 'black')
    plt.colorbar(label='Success rate (%)'); plt.title('30-scan convergence: 10 seeds per cell')
    save('robustness_heatmap.png')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map-run',default='results/even_loop_on')
    p.add_argument('--map-bag',default='data/lio_bag')
    p.add_argument('--map-scan-filter',choices=['all','even'],default='even',help='Use all for an already-even split bag')
    p.add_argument('--query-bag',default='data/lio_bag_odd')
    p.add_argument('--gt',default='results/eval/gt_gnss_enu_tum.txt')
    p.add_argument('--initial-pose',nargs=7,type=float,metavar=('X','Y','Z','QX','QY','QZ','QW'))
    p.add_argument('--reference-traj',help='Query-time map-frame reference TUM for revisit robustness scoring')
    p.add_argument('--out',default='results/loc'); p.add_argument('--resume',action='store_true')
    a=p.parse_args(); out=output_dir(a.out); cfg=Config(); began=perf_counter()
    mapping=load_tum(Path(a.map_run)/'keyframes_final_tum.txt'); gt=load_tum(a.gt)
    reference=load_tum(a.reference_traj) if a.reference_traj else mapping
    input_paths=[Path(a.map_run)/'keyframes_final_tum.txt',Path(a.gt)]
    if a.reference_traj: input_paths.append(Path(a.reference_traj))
    input_hashes=fingerprint(input_paths)
    signature=dict(args={k:v for k,v in vars(a).items() if k!='resume'},config=asdict(cfg),input_sha256=input_hashes,
                   code_sha256=fingerprint([Path(__file__),Path(__file__).with_name('localize.py')]))
    signature=json.loads(json.dumps(signature))
    sigpath=out/'configuration.json'
    if a.resume and sigpath.exists() and json.loads(sigpath.read_text())!=signature:
        raise ValueError('Resume configuration/input/code changed; use a separate --out')
    save_json(sigpath,signature)
    prior=out/'prior_map.npz'
    if not a.resume or not prior.exists():
        print('Building even-only prior map',flush=True)
        maps,meta=build_map(a.map_bag,mapping,a.map_scan_filter,cfg.map_voxel_m)
        np.savez_compressed(prior,xyz=maps[0][:,:3],metadata=json.dumps(meta)); del maps
    with np.load(prior) as archive:
        xyz=archive['xyz']; prior_meta=json.loads(str(archive['metadata']))
    if prior_meta['scan_filter']!=a.map_scan_filter or prior_meta['voxel_m']!=cfg.map_voxel_m:
        raise ValueError('Prior map cache configuration mismatch')
    localizer=Localizer(cloud(xyz,cfg),cfg); del xyz
    print(f'Prior target: {len(localizer.target.points)} points',flush=True)
    # Read once, retain only the 5 x 30 robustness windows; baseline streams.
    times=[]
    for t,_,_ in query_scans(a.query_bag,cfg): times.append(t)
    times=np.array(times)
    supported=np.flatnonzero((times>=reference[0,0])&(times<=reference[-1,0]))
    if len(supported)<cfg.robustness_scans: raise ValueError('Insufficient supported query scans')
    # Equally spaced on map trajectory arc length, including late-loop window.
    refs=poses_at(reference,times[supported]); arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(refs[:,:3,3],axis=0),axis=1))]
    last=len(supported)-cfg.robustness_scans
    start_idx=supported[np.searchsorted(arc,np.linspace(0,arc[last],STARTS))]
    if len(set(start_idx))!=STARTS: raise ValueError('Trajectory too short for five distinct starts')
    needed=set(int(i+k) for i in start_idx for k in range(cfg.robustness_scans)); cached={}
    trajectory=out/'loc_traj_tum.txt'; table=out/'per_scan.csv'
    if a.resume and trajectory.exists() and table.exists():
        import csv
        with table.open() as f:
            scan_rows=list(csv.DictReader(f))
        for row in scan_rows:
            for k in row:
                row[k]=(row[k]=='True') if k in ('accepted','lost','coarse_converged','fine_converged') else float(row[k])
    else:
        first_stamp=times[0]
        # Mapping keyframes begin ~0.95 s after the bag. Explicit zero-order
        # endpoint initialization covers this unobserved startup; no GNSS used.
        initial=poses_at(reference,[np.clip(first_stamp,reference[0,0],reference[-1,0])])[0]
        if a.initial_pose is not None:
            values=np.asarray(a.initial_pose)
            if not np.isfinite(values).all() or not np.isclose(np.linalg.norm(values[3:]),1,atol=1e-4):
                raise ValueError('Initial pose must be finite with unit quaternion')
            initial=np.eye(4); initial[:3,3]=values[:3]; initial[:3,:3]=Rotation.from_quat(values[3:]).as_matrix()
        rows=[]; scan_rows=[]
        def stream():
            for i,scan in enumerate(query_scans(a.query_bag,cfg)):
                if i in needed: cached[i]=scan
                yield scan
        for t,pose,row in track(localizer,stream(),initial):
            rows.append(tum_row(t,pose)); scan_rows.append(row)
            if len(rows)%100==0: print(f'Baseline {len(rows)}/{len(times)} fitness={row["fitness"]:.3f} ms={row["total_ms"]:.1f}',flush=True)
        np.savetxt(trajectory,np.array(rows),fmt='%.9f'); write_csv(table,scan_rows)
    if len(scan_rows)!=len(times) or any(r['lost'] for r in scan_rows):
        raise RuntimeError('Baseline LOST or incomplete; per_scan.csv retained for diagnosis')
    if len(cached)!=len(needed):
        for i,scan in enumerate(query_scans(a.query_bag,cfg)):
            if i in needed: cached[i]=scan
            if i>=max(needed): break
    metrics,data=evaluate(load_tum(trajectory),gt,out/'localization')
    baseline,baseline_data=evaluate(reference,gt,out/'mapping_baseline')
    # Common timestamps remove keyframe vs dense-query sampling as a confound.
    common_ref=poses_at(reference,times[supported])
    common_tum=np.array([tum_row(t,pose) for t,pose in zip(times[supported],common_ref)])
    common_base,_=evaluate(common_tum,gt,out/'mapping_common_timestamps')
    common_loc,_=evaluate(load_tum(trajectory)[supported],gt,out/'localization_common_timestamps')
    robust_path=out/'robustness_trials'; robust_path.mkdir(exist_ok=True)
    robust=[]
    for level,(distance,angle) in enumerate(LEVELS):
        for start,index in enumerate(start_idx):
            scans=[cached[int(index+k)] for k in range(cfg.robustness_scans)]
            initial_reference=poses_at(reference,[scans[0][0]])[0]
            final_reference=poses_at(reference,[scans[-1][0]])[0]
            for seed in range(SEEDS):
                path=robust_path/f'l{level}_s{start}_seed{seed}.json'
                if a.resume and path.exists():
                    robust.append(json.loads(path.read_text())); continue
                rng=np.random.default_rng(np.random.SeedSequence([start,seed]))
                direction=rng.uniform(-np.pi,np.pi); sign=rng.choice([-1,1])
                initial=initial_reference.copy()
                initial[:2,3]+=distance*np.array([np.cos(direction),np.sin(direction)])
                initial[:3,:3]=Rotation.from_euler('z',sign*angle,degrees=True).as_matrix()@initial[:3,:3]
                result=list(track(localizer,scans,initial)); _,final,last_row=result[-1]
                error=float(np.linalg.norm(final[:3,3]-final_reference[:3,3]))
                yaw=lambda r: np.arctan2(r[1,0],r[0,0])
                delta=yaw(final[:3,:3])-yaw(final_reference[:3,:3])
                yaw_error=float(abs(np.degrees(np.arctan2(np.sin(delta),np.cos(delta)))))
                success=(len(result)==cfg.robustness_scans and not last_row['lost'] and
                         error<cfg.success_position_m and yaw_error<cfg.success_yaw_deg)
                row=dict(level=level,start=start,seed=seed,start_index=int(index),start_timestamp=scans[0][0],
                         perturbation_m=distance,perturbation_deg=angle,direction_rad=direction,yaw_sign=int(sign),
                         completed_scans=len(result),lost=last_row['lost'],position_error_m=error,
                         yaw_error_deg=yaw_error,success=bool(success),
                         rejected_scans=sum(not r['accepted'] for _,_,r in result),
                         total_ms=sum(r['total_ms'] for _,_,r in result))
                save_json(path,row); robust.append(row)
            write_csv(out/'robustness.csv',robust)
            print(f'Robustness {len(robust)}/200; level {distance}m/{angle}deg, start {start+1}: '+
                  f'{sum(r["success"] for r in robust if r["level"]==level and r["start"]==start)}/10',flush=True)
    write_csv(out/'robustness.csv',robust)
    make_plots(out,data,baseline_data,scan_rows,robust)
    speed=np.array([r['total_ms'] for r in scan_rows]); failures=sum(not r['accepted'] for r in scan_rows)
    result=dict(scans=len(scan_rows),lost=False,failures=failures,failure_rate=failures/len(scan_rows),
                mean_fitness=float(np.mean([r['fitness'] for r in scan_rows])),
                mean_ms=float(speed.mean()),max_ms=float(speed.max()),p95_ms=float(np.percentile(speed,95)),
                realtime_factor_10hz=float(100/speed.mean()),compute_to_sensor_time_ratio=float(speed.mean()/100),
                robustness_trials=len(robust),robustness_successes=sum(r['success'] for r in robust),
                startup_endpoint_hold_s=float(max(mapping[0,0]-times[0],0)),
                scans_outside_map_pose_interval=int(len(times)-len(supported)),
                prior_metadata=prior_meta,target_points=len(localizer.target.points),
                peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
                wall_seconds=perf_counter()-began,open3d_version=o3d.__version__)
    save_json(out/'run_stats.json',result)
    lines=[FAIRNESS,'', '# TASK-05b — fixed-prior localization','',
           '| Trajectory | 2D ATE SE3 RMSE (m) | 2D ATE SE3+lever2D RMSE (m) |','|---|---:|---:|']
    for name,m in [('Held-out localization (all scans)',metrics),('Even mapping (keyframes)',baseline),
                   ('Localization (common timestamps)',common_loc),('Even mapping (common timestamps)',common_base)]:
        lines.append(f'| {name} | {m["se3"]["ate_2d_m"]["rmse"]:.4f} | {m["se3_lever"]["ate_2d_m"]["rmse"]:.4f} |')
    lines+=['',f'완주 {len(scan_rows)}/{len(times)} scans, LOST=0; 실패 {failures} ({failures/len(times):.2%}).',
        f'Fitness mean {result["mean_fitness"]:.4f}. 처리 평균/최대 {speed.mean():.1f}/{speed.max():.1f} ms, p95 {result["p95_ms"]:.1f} ms.',
        f'10 Hz 기준 실시간 계수 = sensor budget / compute time = **{100/speed.mean():.2f}×** (1 이상이면 실시간).',
        f'최대 RSS {result["peak_rss_mib"]:.0f} MiB. Scan 시간은 전처리+예측+정합이며 bag I/O·초기 맵 구축·평가/그림은 제외한다.',
        '', '## Initial-pose robustness','', '| Perturbation | Success / trials | Rate |','|---|---:|---:|']
    for i,(m,deg) in enumerate(LEVELS):
        successes=sum(r['success'] for r in robust if r['level']==i)
        lines.append(f'| {m:g} m / {deg:g}° | {successes}/50 | {successes/50:.0%} |')
    improvement=common_base['se3_lever']['ate_2d_m']['rmse']-common_loc['se3_lever']['ate_2d_m']['rmse']
    lines+=['','## Interpretation', '',
        f'공통 시각의 lever2D ATE 기준 위치추정과 매핑 차이는 {improvement:+.7f} m (양수는 위치추정 개선)이다.',
        f'홀수 스캔 {len(times)}개를 고정 prior에 정합했고 GNSS는 평가에만 사용했다. IMU 입력과 매핑 포즈의 매 스캔 재초기화는 없다.',
        f'강건성 {len(robust)}회 중 {result["robustness_successes"]}회가 30스캔 후 3D 위치 <0.3 m 및 yaw <1°를 만족했다. 실패 정합은 예측값 유지, 5회 연속 실패는 LOST 처리했다.',
        '', '## Method and limitations', '',
        f'- Prior: {a.map_bag}, {a.map_scan_filter} scans only, 0.2 m voxel, fixed even mapping final poses. Metadata: `{prior_meta}`.',
        '- Query: 1–80 m, 0.3 m voxel, no deskew; constant world velocity/body angular velocity prediction from the last two output poses, actual timestamp intervals.',
        '- GICP: 1.0 → 0.5 m correspondence gates, max 30 iterations/stage, absolute fitness/RMSE change <1e-6. Map 20-NN planar covariances and KD-tree built once; scan covariance built once per scan.',
        '- Open3D public registration_generalized_icp rebuilds the tree and hides iterations. The cached loop uses its TransformationEstimationForGeneralizedICP solver; synthetic and real-scan cross-checks compare against the public API. Source: https://github.com/isl-org/Open3D/blob/main/cpp/open3d/pipelines/registration/GeneralizedICP.cpp',
        f'- 첫 map keyframe은 첫 홀수 스캔보다 {result["startup_endpoint_hold_s"]:.3f} s 늦다. 최초 포즈는 가장 가까운 첫 keyframe으로 고정 초기화(시작 정지 가정); 범위 밖 {result["scans_outside_map_pose_interval"]}개도 전부 위치추정했다. 강건성 기준 포즈에는 외삽을 쓰지 않았다.',
        '- 5 starts are equally spaced by map-path arc length over feasible 30-scan windows. Per seed: exact horizontal perturbation magnitude, random uniform direction, random yaw sign; paired directions/signs across levels. No translation z perturbation. End errors use even-pose SLERP/linear interpolation, not GNSS.',
        '- Robustness is relative to the even mapping trajectory, which also has map/trajectory approximation errors. Success is not a GNSS accuracy claim. Early LOST trials count as failures; no extra recovery or tuning by seed.',
        '- Same-run held-out scans share the environment, IMU-derived map geometry and mapping drift. Split reduces direct scan reuse but cannot establish independent revisit performance. Map poses are used only for initial poses and robustness scoring.',
        '- SE3/lever2D alignments are globally fitted to GNSS for evaluation; lever estimates are nuisance fits. GNSS tangent and flat-ground approximations are inherited from TASK-05. No GNSS/IMU enters localization.',
        '- 별도 날짜의 bag에는 --map-run/--map-bag/--query-bag/--gt 및 별도 --out을 사용한다. 맵 좌표계의 초기 포즈를 --initial-pose으로 제공할 수 있다. 강건성 평가에는 쿼리 시각에 대응하는 별도의 --reference-traj가 필요하다.',
        '', 'Reproduce:', '', '```bash', 'source env.sh',
        'PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 .venv/bin/python -m tools.loc.run_loc_eval --resume', '```']
    (out/'summary.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(result,indent=2),flush=True)

if __name__=='__main__': main()

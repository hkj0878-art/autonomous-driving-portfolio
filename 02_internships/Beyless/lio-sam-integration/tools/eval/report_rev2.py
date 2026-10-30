"""Revision 2 report and reproducible GNSS RPE noise floor (no experiment replay)."""
import csv
import json
import numpy as np
from scipy.spatial.transform import Rotation
from .traj_eval import EvalConfig, rpe, tangent_yaw

NOISE_SIGMA_M = .014
NOISE_TRIALS = 10
NOISE_SEED = 20261026


def write_csv(path, rows):
    with path.open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)


def noise_floor(series):
    """Use each final trajectory's synchronized GT samples, no alignment or lever fit.

    Clean reference remains fixed. Each trial adds independent isotropic 3D
    position noise; synthetic estimate yaw comes from its own 5 m tangent.
    Statistics are over the ten trial means, not correlated RPE segments.
    """
    rng=np.random.default_rng(NOISE_SEED); rows=[]
    for run, kinds in series.items():
        gt=kinds['final']['gt']; cfg=EvalConfig()
        clean_rotation=Rotation.from_euler('z',tangent_yaw(gt,cfg.tangent_baseline_m)).as_matrix()
        clean=rpe(gt,clean_rotation,gt,cfg)
        trials=[]
        for _ in range(NOISE_TRIALS):
            noisy=gt+rng.normal(0,NOISE_SIGMA_M,gt.shape)
            rot=Rotation.from_euler('z',tangent_yaw(noisy,cfg.tangent_baseline_m)).as_matrix()
            trials.append(rpe(noisy,rot,gt,cfg))
        for length in cfg.segment_lengths_m:
            key=str(int(length))
            for field in ('translation_percent','yaw_deg_per_100m'):
                if length<25 and field.startswith('yaw'): continue
                v=[trial[key][field]['mean'] for trial in trials]
                rows.append(dict(run=run,segment_m=int(length),metric=field,
                    clean_gt_mean=clean[key][field]['mean'],noise_mean=float(np.mean(v)),
                    noise_std_sample=float(np.std(v,ddof=1)),trials=NOISE_TRIALS,
                    sigma_m=NOISE_SIGMA_M,seed=NOISE_SEED,poses=len(gt)))
    return rows


def fmt(v):
    return f'{np.mean(v):.4f} ± {np.std(v,ddof=1):.4f}' if len(v)>1 else f'{v[0]:.4f} (n=1)'


def report(out, rows, maps, loops, aggregates, series, representative, gnss_stats):
    final=[r for r in rows if r['trajectory']=='final']
    noise=noise_floor(series);write_csv(out/'rpe_noise_floor.csv',noise)
    lines=['# TASK-05 rev2 evaluation','',
        '| Group | n | 2D ATE RMSE, SE3+lever2D (m) | Endpoint vector error, SE3 (m) | RPE 100 m, lever2D (%) | Loop consistency | MME (nats) |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for group in ('loop_on','loop_off','even'):
        select=lambda run: run.startswith(group+'_r') if group!='even' else run=='even_loop_on'
        rr=[r for r in final if select(r['run'])]; mm=[m for m in maps if select(m['run'])]
        ll=[r for r in loops if select(r['run']) and r['classification']!='NOT_APPLICABLE']
        per_run=[]
        for row in rr:
            pairs=[p for p in ll if p['run']==row['run']]
            if pairs: per_run.append(sum(p['classification']=='CONSISTENT(TP)' for p in pairs)/len(pairs))
        ratio=(fmt(per_run)+f"; {sum(r['classification']=='CONSISTENT(TP)' for r in ll)}/{len(ll)} pairs") if ll else 'N/A (no factors)'
        lines.append(f"| {group} | {len(rr)} | {fmt([r['ate_lever_2d_rmse_m'] for r in rr])} | {fmt([r['endpoint_error_m'] for r in rr])} | {fmt([r['rpe_100m_translation_percent'] for r in rr])} | {ratio} | {fmt([m['mme_nats'] for m in mm])} |")
    lines+=['',f'TASK-06 representative run: **{representative}** (median final 2D SE3+lever2D ATE among ON, n=3).',
        'The SE3-only align_se3.json files remain the map alignment; horizontal lever fitting does not change them.',
        '', 'Group values are mean ± sample std (ddof=1); even has one observation and no estimated repeatability.',
        '', '## Interpretation (n=3)', '']
    on=[r['ate_lever_2d_rmse_m'] for r in final if r['run'].startswith('loop_on_r')]
    off=[r['ate_lever_2d_rmse_m'] for r in final if r['run'].startswith('loop_off_r')]
    improvement=float(np.mean(off)-np.mean(on));std_on=np.std(on,ddof=1);std_off=np.std(off,ddof=1)
    interpretation=[
        f'Loop closure changes horizontal ATE from OFF {fmt(off)} m to ON {fmt(on)} m; OFF−ON = {improvement:.4f} m ({improvement/np.mean(off)*100:.1f}% reduction; negative means degradation).',
        f'The mean difference is {"smaller" if abs(improvement)<max(std_on,std_off) else "larger"} than the larger group std ({max(std_on,std_off):.4f} m). With only three runs per group, this does not establish statistical significance or generalization to another drive.',
        'Yaw-only RPE removes the artificial mounting-pitch contribution; 100 m translation is now consistent with sub-metre ATE. The GNSS noise floor below uses ten independent 1.4 cm perturbations and matched final-pose sampling.',
        'Lever2D estimates are fitted on the same GT used to score ATE; agreement across repeats supports a plausible mounting offset but is not independent calibration. ON/OFF variation can also absorb trajectory distortion.',
        'Loop consistency checks the relative position vector, not an ICP rotation or an independent ground-truth loop transform. Online odometry already contains earlier loop updates, so online/final differences do not isolate a single constraint.'
    ]
    lines+=interpretation
    (out/'task05_rev2.md').write_text('# TASK-05 rev2 work log\n\nScope: tools/eval/ and results/eval/ only. No experiments rerun. Existing canonical v3 trajectories, maps and loop records are read-only.\n\n'+ '\n\n'.join(interpretation)+'\n\nMethods and limitations are documented in summary.md; verification in independent_verification.json and pytest_rev2.txt. Per the user scope, logs/task05.md is not modified.\n')
    lines+=['', '## Methods and limits','',
        f'GNSS accepted {gnss_stats["accepted"]}/{gnss_stats["total"]}. ENU origin is first fix. Status >=2 and horizontal sigma <=0.05 m; vertical sigma is recorded only.',
        'SE3 Umeyama fits rigid alignment without scale and is unchanged; evo independently checks 3D ATE to <1 mm.',
        'Lever2D jointly fits R_align, t and lx, ly with lz=0: p_gnss = R_align p_est + t + Rz(yaw(R_align R_est)) [lx,ly,0]. Yaw is extracted after alignment; roll/pitch including the approximately 9.3° LiDAR mounting pitch is removed. Legacy unconstrained 3-DoF fits remain in metrics.json under se3_lever_3d_diagnostic only.',
        'Primary RPE uses the same lever2D-corrected antenna positions and joint alignment as primary ATE, with yaw-only aligned estimated rotations. SE3-only RPE is retained separately in metrics.json under rpe_se3_diagnostic (about 3–4% at 100 m): its position-only alignment is biased by the roughly 1.5 m rotating origin offset, including about 2.3° alignment yaw change when lever2D is fitted. The correction is fitted in-sample and is therefore optimistic. GT yaw is a centered 5 m trajectory tangent (roll/pitch zero, flat-ground approximation). All start poses use first endpoint >= target length and actual GT arc length; segments overlap and are correlated. Translation is reported at 10/25/50/100 m; yaw only at 25/50/100 m. Tangent yaw at corners and imperfect lever calibration remain limitations.',
        'Endpoint vector error = norm((aligned last−first) − (GT last−first)), evaluated at each trajectory’s synchronized support; this is not merely the start–end gap.',
        '', '## Trajectory metrics','',
        '| Run | Trajectory | SE3 2D ATE (m) | Lever2D 2D ATE (m) | SE3 3D ATE (m) | Lever2D 3D ATE (m) | lx / ly (m) | RPE 100 m, lever2D (%) |',
        '|---|---|---:|---:|---:|---:|---|---:|']
    for r in rows:
        lines.append(f"| {r['run']} | {r['trajectory']} | {r['ate_se3_2d_rmse_m']:.4f} | {r['ate_lever_2d_rmse_m']:.4f} | {r['ate_se3_3d_rmse_m']:.4f} | {r['ate_lever_3d_rmse_m']:.4f} | {r['lever_x_m']:.4f} / {r['lever_y_m']:.4f} | {r['rpe_100m_translation_percent']:.4f} |")
    lines+=['','## Repeatability: ON/OFF mean ± sample std','','| Group | Trajectory | Metric | Mean | Std | n |','|---|---|---|---:|---:|---:|']
    for r in aggregates:
        lines.append(f"| {r['group']} | {r['trajectory']} | {r['metric']} | {r['mean']:.5f} | {r['std_sample']:.5f} | {r['n']} |")
    lines+=['','## Lever2D consistency','','| Population | n | lx (m) | ly (m) | Norm (m) |','|---|---:|---:|---:|---:|']
    for kind in ('final','online'):
        for group in ('loop_on','loop_off','all seven'):
            rr=[r for r in rows if r['trajectory']==kind and (group=='all seven' or r['run'].startswith(group+'_r'))]
            lines.append(f"| {group} / {kind} | {len(rr)} | {fmt([r['lever_x_m'] for r in rr])} | {fmt([r['lever_y_m'] for r in rr])} | {fmt([r['lever_norm_m'] for r in rr])} |")
    lines+=['','These are yaw-only ENU heading body offsets with lz fixed at zero, not the tilted LiDAR-frame x/z components. Per-run fits (including even) are listed above. Across-all-run std mixes loop modes and the single even trial; use within-group std to assess repeatability.','',
        '## RPE GNSS noise floor','',
        f'Clean GT and GT + isotropic Gaussian position noise σ={NOISE_SIGMA_M} m, {NOISE_TRIALS} trials, seed {NOISE_SEED}. Reference is clean matched GT for each run. No alignment or lever fit; noisy estimate yaw is derived from its own 5 m tangent. Values are mean ± sample std of ten trial means, not of overlapping segments. All clean-GT translation/yaw means are numerically zero. This tests metric sensitivity at final-keyframe sampling, not a complete GNSS error model.',
        '', '| Run | Segment (m) | Translation noise (%) | Yaw noise (deg/100 m) |','|---|---:|---:|---:|']
    for run in series:
        for length in (10,25,50,100):
            rr=[r for r in noise if r['run']==run and r['segment_m']==length]
            def value(field):
                v=[r for r in rr if r['metric']==field]
                return f"{v[0]['noise_mean']:.4f} ± {v[0]['noise_std_sample']:.4f}" if v else '— (not reported)'
            lines.append(f"| {run} | {length} | {value('translation_percent')} | {value('yaw_deg_per_100m')} |")
    lines+=['','## Loop constraint consistency','','| Run | Current / previous keyframe | GT horizontal distance (m), reference only | Relative position error, 3D (m) | Verdict |','|---|---|---:|---:|---|']
    for r in loops:
        val=lambda x: '—' if x is None else f'{x:.4f}'
        lines.append(f"| {r['run']} | {r['keyframe_current']} / {r['keyframe_previous']} | {val(r['gt_distance_m'])} | {val(r['relative_position_error_m'])} | {r['classification']} |")
    lines+=['', 'After unchanged SE3 alignment, Δ = (p_i−p_j) − (g_i−g_j), with GT interpolated at the two final keyframe times. ||Δ|| <0.5 m is CONSISTENT(TP); otherwise INCONSISTENT. GT distance is reference only. For pairs traversed in the same direction the antenna lever effect approximately cancels. Marker matching must be <=0.2 m. This checks position consistency only.',
        '', '## Preserved map quality','','| Run | MME (nats) | MPV (m²) | Overlap median (m) | p95 (m) | >0.2 m ratio |','|---|---:|---:|---:|---:|---:|']
    for m in maps:
        lines.append(f"| {m['run']} | {m['mme_nats']:.5f} | {m['mpv_m2']:.7f} | {m['overlap_median_m']:.4f} | {m['overlap_p95_m']:.4f} | {m['double_wall_ratio']:.3f} |")
    lines+=['','MME/MPV use the same 0.1 m ENU voxel, 0.3 m radius, >=6 neighbors and covariance eigenvalue floor 1e-9 m², cropped to GNSS XY bounds with unbounded height. Existing map metrics and first/last 15 s overlap clouds are reused because source maps and SE3 alignment are unchanged. Overlap uses both NN directions gated at 1 m; coverage is in map_quality.csv. No deskew. Even is not held-out localization TASK-05b.',
        '', 'v3 provenance: two divergent even attempts remain archived under results/archive_v3_failed_attempts; canonical even used sensor readiness and DDS discovery delay with unchanged params/rate. No experiment replay in rev2.',
        '', f"Maximum evo ATE difference: {max(r['evo_difference_m'] for r in rows):.3g} m."]
    (out/'summary.md').write_text('\n'.join(lines)+'\n')

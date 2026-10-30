"""Evaluate seven existing experiments without mutating source artifacts."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from .gnss_gt import extract
from .traj_eval import load_tum, evaluate
from .build_map_from_poses import build_map
from .map_quality import read_map, local_quality, overlap

ROOT=Path(__file__).resolve().parents[2]
RUNS=[f'loop_{mode}_r{i}' for mode in ('on','off') for i in (1,2,3)]+['even_loop_on']
COLORS=['#0072B2','#D55E00','#009E73','#CC79A7','#E69F00','#56B4E9','#882255']
DPI=200
OVERLAP_SECONDS=15.
LOOP_MATCH_MAX_M=0.2
LOOP_CONSISTENCY_MAX_M=0.5


def save_fig(fig,path):
    fig.tight_layout(); fig.savefig(path,dpi=DPI); plt.close(fig)


def source_manifest():
    paths=[]
    for name in RUNS:
        paths.extend(p for p in (ROOT/'results'/name).rglob('*') if p.is_file())
    return {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


def write_table(path,rows):
    with Path(path).open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)


def main():
    p=argparse.ArgumentParser(); p.add_argument('--out',default=str(ROOT/'results/eval'))
    p.add_argument('--bag',default=str(ROOT/'data/lio_bag')); p.add_argument('--reuse-overlap',action='store_true')
    p.add_argument('--reuse-map-quality',action='store_true',help='Reuse unchanged map metrics; trajectory evaluation only')
    p.add_argument('--sigma-axes',choices=['xy','xyz'],default='xy')
    args=p.parse_args(); out=Path(args.out).resolve()
    if not out.is_relative_to(ROOT/'results/eval'): raise ValueError('Output must be under results/eval')
    out.mkdir(parents=True,exist_ok=True); figs=out/'figs';figs.mkdir(exist_ok=True)
    started=time.time(); before=source_manifest()
    (out/'source_manifest_before.json').write_text(json.dumps(before,indent=2))
    for run in RUNS:
        meta=json.loads((ROOT/'results'/run/'run_meta.json').read_text())
        if not meta.get('success'): raise ValueError(f'Failed source experiment: {run}')
        for filename in ('keyframes_final_tum.txt','odom_online_tum.txt','map/GlobalMap.pcd'):
            if not (ROOT/'results'/run/filename).is_file(): raise FileNotFoundError(f'{run}/{filename}')
    gt,gnss_stats=extract(args.bag,out,args.sigma_axes)
    print(f'GNSS: {gnss_stats}',flush=True)
    rows=[]; map_rows=[]; loop_rows=[]; all_metrics={}; all_series={}
    bounds=(gt[:,1:3].min(axis=0),gt[:,1:3].max(axis=0))
    for k,run in enumerate(RUNS):
        directory=ROOT/'results'/run; all_metrics[run]={};all_series[run]={}
        for kind,filename in [('final','keyframes_final_tum.txt'),('online','odom_online_tum.txt')]:
            traj=load_tum(directory/filename)
            metrics,series=evaluate(traj,gt,out/run/kind)
            all_metrics[run][kind]=metrics;all_series[run][kind]=series
            row=dict(run=run,trajectory=kind,matched=metrics['matched_count'],
                     ate_se3_3d_rmse_m=metrics['se3']['ate_m']['rmse'],ate_lever_3d_rmse_m=metrics['se3_lever']['ate_m']['rmse'],
                     ate_se3_2d_rmse_m=metrics['se3']['ate_2d_m']['rmse'],ate_lever_2d_rmse_m=metrics['se3_lever']['ate_2d_m']['rmse'],
                     lever_norm_m=metrics['se3_lever']['lever_norm_m'],lever_warning=metrics['se3_lever']['warning'],
                     endpoint_error_m=metrics['endpoint']['displacement_error_m'],
                     evo_difference_m=metrics['evo_crosscheck']['absolute_difference_m'])
            for length,r in metrics['rpe'].items():
                row[f'rpe_{length}m_translation_percent']=r['translation_percent']['mean'] if r['count'] else None
                row[f'rpe_{length}m_yaw_deg_per_100m']=r['yaw_deg_per_100m']['mean'] if r['yaw_deg_per_100m'] is not None else None
            row['lever_x_m'],row['lever_y_m']=metrics['se3_lever']['lever_arm_body_m'][:2]
            rows.append(row)
        metrics=all_metrics[run]['final'];series=all_series[run]['final']
        fig,ax=plt.subplots(figsize=(7,6)); ax.plot(gt[:,1],gt[:,2],color='black',label='RTK GNSS')
        ax.plot(series['aligned'][:,0],series['aligned'][:,1],color=COLORS[k],label='Final keyframes (SE3)')
        ax.plot(series['antenna'][:,0],series['antenna'][:,1],color='#009E73',linestyle='--',label='Yaw-body lever 2D fit (antenna)')
        ax.set(xlabel='East (m)',ylabel='North (m)',title=run);ax.axis('equal');ax.legend();ax.grid(alpha=.2)
        save_fig(fig,figs/f'traj_overlay_{run}.png')
        print(f'{run}: ATE {metrics["se3"]["ate_m"]["rmse"]:.4f} m; evaluating map',flush=True)
        align=metrics['se3']
        if args.reuse_map_quality:
            quality=json.loads((out/run/'map_quality.json').read_text())
        else:
            xyz=read_map(directory/'map/GlobalMap.pcd',align['R'],align['t'],bounds)
            quality=local_quality(xyz);del xyz
        trajectory=load_tum(directory/'keyframes_final_tum.txt')
        # Windows refer to the full sensor sequence, not differing final keyframe ends.
        windows=[(gt[0,0],gt[0,0]+OVERLAP_SECONDS),(gt[-1,0]-OVERLAP_SECONDS,gt[-1,0])]
        cache=out/run/'overlap_clouds.npz'
        if (args.reuse_overlap or args.reuse_map_quality) and cache.exists():
            loaded=np.load(cache); a,b=loaded['first'],loaded['last']
            metadata=json.loads(str(loaded['metadata']))
        else:
            maps,metadata=build_map(args.bag,trajectory,'even' if run=='even_loop_on' else 'all',
                                    windows=windows,alignment=(align['R'],align['t']))
            a,b=maps
            np.savez_compressed(cache,first=a,last=b,metadata=json.dumps(metadata))
        quality['overlap']=overlap(a[:,:3],b[:,:3]);quality['build']=metadata
        quality['crop']=dict(frame='ENU',xy_min=bounds[0].tolist(),xy_max=bounds[1].tolist(),vertical='unbounded',
                             note='Same GNSS XY bounding box for all runs; saved LIO-SAM GlobalMap, revoxelized in ENU at 0.1 m')
        (out/run/'map_quality.json').write_text(json.dumps(quality,indent=2))
        map_rows.append(dict(run=run,mme_nats=quality['mme_nats'],mpv_m2=quality['mpv_m2'],
                             eligible_fraction=quality['eligible_fraction'],overlap_median_m=quality['overlap']['median_m'],
                             overlap_p95_m=quality['overlap']['p95_m'],double_wall_ratio=quality['overlap']['double_wall_ratio'],
                             A_overlap_fraction=quality['overlap']['A_to_B']['overlap_fraction'],
                             B_overlap_fraction=quality['overlap']['B_to_A']['overlap_fraction']))
        if run in ('loop_on_r2','loop_off_r2'):
            fig,ax=plt.subplots(figsize=(8,7))
            for cloud,color,label in [(a,'#0072B2','First 15 s'),(b,'#D55E00','Last 15 s')]:
                sample=cloud[::max(1,len(cloud)//60000)]
                ax.scatter(sample[:,0],sample[:,1],s=.3,c=color,label=label,alpha=.6,rasterized=True)
            ax.set(xlabel='East (m)',ylabel='North (m)',title=f'Loop overlap: {run} (no deskew)');ax.axis('equal');ax.legend(markerscale=8)
            save_fig(fig,figs/f'loop_overlap_{"on" if "loop_on" in run else "off"}.png')
        meta=json.loads((directory/'run_meta.json').read_text())
        pairs=json.loads((directory/'loop_pairs.json').read_text())['pairs']
        if meta['loop_enabled'] and not pairs: raise ValueError(f'Missing actual loop pairs: {run}')
        for pair in pairs:
            indices=pair['keyframe_indices']; stamps=np.array(pair['keyframe_stamps'])
            if max(pair['matching_distances_m'])>LOOP_MATCH_MAX_M: raise ValueError(f'Uncertain marker matching: {run}: {pair}')
            if indices[0]==indices[1]: raise ValueError(f'Degenerate loop pair: {run}')
            from .traj_eval import associate
            # associate requires >=3 poses; duplicate one stamp solely for interpolation.
            probe=np.zeros((3,8));probe[:,0]=[stamps[0],stamps[1],stamps[1]]
            matched,positions,_=associate(probe,gt)
            if len(matched)!=3: raise ValueError('Loop GT time unavailable')
            distance=float(np.linalg.norm(positions[0,:2]-positions[1,:2]))
            estimated=trajectory[indices,1:4]@np.asarray(align['R']).T+np.asarray(align['t'])
            relative_error=float(np.linalg.norm((estimated[0]-estimated[1])-(positions[0]-positions[1])))
            loop_rows.append(dict(run=run,recorded_events=meta['loop_events'],keyframe_current=indices[0],
                                  keyframe_previous=indices[1],gt_distance_m=distance,
                                  relative_position_error_m=relative_error,
                                  classification='CONSISTENT(TP)' if relative_error<LOOP_CONSISTENCY_MAX_M else 'INCONSISTENT',
                                  matching_max_m=max(pair['matching_distances_m'])))
        if not pairs:
            loop_rows.append(dict(run=run,recorded_events=0,keyframe_current=None,keyframe_previous=None,
                                  gt_distance_m=None,relative_position_error_m=None,classification='NOT_APPLICABLE',matching_max_m=None))
        print(f'{run}: map complete, {quality["total_points"]} points',flush=True)
    write_table(out/'summary.csv',rows);write_table(out/'map_quality.csv',map_rows);write_table(out/'loop_validation.csv',loop_rows)
    aggregates=[]
    for kind in ('final','online'):
        for group in ('loop_on','loop_off'):
            selected=[r for r in rows if r['trajectory']==kind and r['run'].startswith(group+'_r')]
            for field in ['ate_se3_2d_rmse_m','ate_lever_2d_rmse_m','ate_se3_3d_rmse_m','ate_lever_3d_rmse_m','endpoint_error_m','lever_norm_m','lever_x_m','lever_y_m'] + [key for key in selected[0] if key.startswith('rpe_')]:
                v=[r[field] for r in selected if r[field] is not None]
                if not v: continue
                aggregates.append(dict(group=group,trajectory=kind,metric=field,n=len(v),mean=float(np.mean(v)),std_sample=float(np.std(v,ddof=1))))
    for group in ('loop_on','loop_off'):
        for field in list(map_rows[0])[1:]:
            v=[r[field] for r in map_rows if r['run'].startswith(group+'_r')]
            aggregates.append(dict(group=group,trajectory='map',metric=field,n=len(v),mean=float(np.mean(v)),std_sample=float(np.std(v,ddof=1))))
    write_table(out/'group_summary.csv',aggregates)
    fig,ax=plt.subplots(figsize=(7,6));ax.plot(gt[:,1],gt[:,2],color='black',label='RTK GNSS')
    for k,run in enumerate(RUNS[:6]):
        s=all_series[run]['final']['antenna'];ax.plot(s[:,0],s[:,1],c=COLORS[k],label=run,alpha=.8)
    ax.axis('equal');ax.set(xlabel='East (m)',ylabel='North (m)',title='Final trajectories, SE3 + horizontal lever 2D');ax.legend(fontsize=8);save_fig(fig,figs/'traj_on_vs_off.png')
    fig,ax=plt.subplots(figsize=(9,4))
    for k,run in enumerate(RUNS[:6]):
        s=all_series[run]['final'];ax.plot(s['t']-gt[0,0],s['error'],c=COLORS[k],label=run)
    ax.set(xlabel='Time since first GNSS fix (s)',ylabel='Horizontal SE3 + lever 2D ATE (m)',title='Final keyframe position errors');ax.legend(ncol=3,fontsize=8);save_fig(fig,figs/'ate_vs_time.png')
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for mode,color,shift in [('on','#0072B2',-.18),('off','#D55E00',.18)]:
        for ax,field,title,lengths in [(axes[0],'translation_percent','Translation (%)',(10,25,50,100)),
                                      (axes[1],'yaw_deg_per_100m','Yaw (deg/100 m)',(25,50,100))]:
            x=np.arange(len(lengths))
            values=np.array([[all_metrics[f'loop_{mode}_r{i}']['final']['rpe'][str(length)][field]['mean'] for length in lengths] for i in (1,2,3)])
            ax.bar(x+shift,values.mean(axis=0),.36,yerr=values.std(axis=0,ddof=1),color=color,label=f'Loop {mode}')
            ax.set_xticks(x,[str(v) for v in lengths]);ax.set(xlabel='GT segment length (m)',ylabel=title,title=title+'; 5 m GNSS tangent');ax.legend()
    save_fig(fig,figs/'rpe_bar.png')
    on_final=sorted([r for r in rows if r['trajectory']=='final' and r['run'].startswith('loop_on_r')],key=lambda r:r['ate_lever_2d_rmse_m'])
    representative=on_final[1]['run']
    for run in RUNS:
        align=all_metrics[run]['final']['se3'];matrix=np.eye(4);matrix[:3,:3]=align['R'];matrix[:3,3]=align['t']
        (out/run/'align_se3.json').write_text(json.dumps(dict(matrix_4x4=matrix.tolist(),R=align['R'],t=align['t'],
            source_frame='LIO-SAM odom/front_lidar pose coordinates (metres)',target_frame='GNSS local ENU (metres), first fix origin',
            convention='p_ENU = matrix_4x4 @ [p_odom,1]; rigid SE3, no scale, no lever arm'),indent=2))
    from .report_rev2 import report
    report(out, rows, map_rows, loop_rows, aggregates, all_series, representative, gnss_stats)
    after=source_manifest()
    if before!=after: raise AssertionError('Protected source files changed during evaluation')
    import resource
    verification=dict(metrics_count=len(rows),map_count=len(map_rows),figure_count=len(list(figs.glob('*.png'))),
                      max_evo_difference_m=max(r['evo_difference_m'] for r in rows),protected_sources_unchanged=True,
                      wall_seconds=time.time()-started,peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
                      loop_pair_validation='PASS: all recorded pairs evaluated',representative_run=representative)
    (out/'verification.json').write_text(json.dumps(verification,indent=2))
    print(json.dumps(verification,indent=2),flush=True)


if __name__=='__main__':main()

#!/usr/bin/env python3
"""Independently check seven experiment artifacts, settings, and final trajectories."""
import hashlib
import json
from pathlib import Path
import numpy as np
import yaml
from pcd_to_tum import read_pcd,pose_to_tum
ROOT=Path(__file__).resolve().parents[1]
RUNS=[f'loop_{mode}_r{i}' for mode in ('on','off') for i in (1,2,3)]+['even_loop_on']

def verify():
    base=yaml.safe_load((ROOT/'config/params_base.yaml').read_text()); off=yaml.safe_load((ROOT/'config/params_loop_off.yaml').read_text())
    b=base['/**']['ros__parameters']; o=off['/**']['ros__parameters']
    assert {k for k in b if b[k]!=o[k]}=={'loopClosureEnableFlag'}
    diagnostic=json.loads((ROOT/'results/imu_frame_diagnostic.json').read_text())
    assert diagnostic['configured_gravity_tilt_deg']<1
    np.testing.assert_allclose(np.array(b['extrinsicRPY']).reshape(3,3),np.array(b['extrinsicRot']).reshape(3,3).T,rtol=0,atol=1e-12)
    rows=[]; checks=[]; verified_endpoints={}; rates=set(); details=[]
    for name in RUNS:
        d=ROOT/'results'/name
        if not (d/'run_meta.json').exists(): rows.append(f'| {name} | — | — | — | — | — | — | — | 미완료 |'); checks.append(False); continue
        m=json.loads((d/'run_meta.json').read_text())
        sanity_good=False
        error=None
        try:
            for f in ('run.log','odom_online_tum.txt','keyframes_final_tum.txt','map/GlobalMap.pcd','map/CornerMap.pcd','map/SurfMap.pcd','map/trajectory.pcd','map/transformations.pcd'):
                assert (d/f).stat().st_size>0,f
            a=pose_to_tum(read_pcd(d/'map/transformations.pcd')); tum=np.loadtxt(d/'keyframes_final_tum.txt')
            np.testing.assert_allclose(a,tum,rtol=0,atol=1e-8)
            dist=float(np.linalg.norm(np.diff(a[:,1:4],axis=0),axis=1).sum()); end=float(np.linalg.norm(a[-1,1:4]-a[0,1:4])); speed=float(np.max(np.linalg.norm(np.diff(a[:,1:4],axis=0),axis=1)/np.diff(a[:,0])))
            assert 238.7*.9<=dist<=238.7*1.1 and speed<=30, 'Distance/speed sanity failed'
            sanity_good=True
            verified_endpoints[name]=end
            assert (m['loop_events']>=1 if m['loop_enabled'] else m['loop_events']==0), 'Loop event criterion failed'
            assert m['gps_publishers']==0 and not m['crash_detected']
            actual=yaml.safe_load((d/'params.yaml').read_text())['/**']['ros__parameters']
            assert actual['savePCD'] is False
            assert actual== (o if name.startswith('loop_off_') else b)
            assert m['success'] and m['sanity_pass']
            verified_endpoints[name]=end
            rates.add(m['rate'])
            assert hashlib.sha256((d/'params.yaml').read_bytes()).hexdigest()==m['params_sha256']
            good=True
        except Exception as e:
            good=False; dist=m.get('distance_m',float('nan')); end=m.get('endpoint_m',float('nan')); error=repr(e)
        checks.append(good)
        details.append(dict(run=name,sanity_pass=sanity_good,all_run_checks_pass=good,error=error,distance_m=dist,endpoint_m=end))
        rows.append(f"| {name} | {m['rate']} | {m['processing_wall_seconds']:.1f} | {m.get('keyframes','—')} | {m.get('loop_events','—')} | {dist:.3f} | {end:.3f} | {'PASS' if sanity_good else 'FAIL'} | {'PASS' if good else 'FAIL'} |")
    comparisons=[]
    for i in (1,2,3):
        on=ROOT/f'results/loop_on_r{i}/run_meta.json'; off=ROOT/f'results/loop_off_r{i}/run_meta.json'
        if on.exists() and off.exists(): comparisons.append(verified_endpoints.get(f'loop_on_r{i}',float('inf'))<verified_endpoints.get(f'loop_off_r{i}',0))
    all_ok=all(checks) and len(rates)==1 and len(comparisons)==3 and all(comparisons)
    text='# TASK-03+04 실험 검증\n\n최종 최적화 keyframe 궤적 기준. 처리시간은 기동·재생·drain·저장·종료를 포함한 wall time(초).\n\n| run | rate | 처리시간 s | keyframes | loop events | 누적거리 m | 시작–끝 m | sanity | 전체 run 검증 |\n|---|---:|---:|---:|---:|---:|---:|---|---|\n'+'\n'.join(rows)+'\n\nON/OFF 동일 반복 번호별 종점거리 감소: '+str(comparisons)+f'\n\n전체 완료 기준: {"PASS" if all_ok else "미충족"}\n'
    text+='\n### 반복 분산 (표본 표준편차, n=3)\n\n'
    for mode in ('on','off'):
        paths=[ROOT/f'results/loop_{mode}_r{i}/run_meta.json' for i in (1,2,3)]
        if all(p.exists() for p in paths):
            ms=[json.loads(p.read_text()) for p in paths]
            for key in ('distance_m','endpoint_m','keyframes','loop_events'):
                values=np.array([m.get(key,float('nan')) for m in ms],dtype=float)
                text+=f"- loop {mode} {key}: mean={values.mean():.6f}, std={values.std(ddof=1):.6f}, variance={values.var(ddof=1):.6f}\n"
    if not all_ok:
        text+='\n미충족 run은 위 표와 각 run_meta.json 및 logs/task03_04.md의 v2 분석을 참조한다.\n'
    text+=f"\n최종 배치 설정: historyKeyframeFitnessScore={b['historyKeyframeFitnessScore']}, radius={b['historyKeyframeSearchRadius']} m, timeDiff={b['historyKeyframeSearchTimeDiff']} s.\n"
    text+='\nRPY 정정 후 fitness=0.3 기본 배치 7회는 [archive_v2_fitness03/experiments_summary.md](archive_v2_fitness03/experiments_summary.md)에 전체 보존했다. 해당 배치는 sanity 7/7이나 ON r2 loop=0, ON/OFF 종점 비교 1/3 통과였다. 이후 fitness만 0.5로 조정하고 7회 전체를 다시 수행했다. 파일럿은 fitness=0.3이며 1배속 통과로 rate를 결정했다. 별도 fitness 0.5 trial을 최종 반복에 포함하지 않았다.\n'
    text+='\n## 진단 파일럿\n\n| run | rate | keyframes | loops | 누적거리 m | 종점거리 m | 최대속도 m/s | 평균/최대 지연 s | 결과 |\n|---|---:|---:|---:|---:|---:|---:|---|---|---|\n'
    for pilot in ('pilot_rate1','pilot_rate05'):
        path=ROOT/'results'/pilot/'run_meta.json'
        if not path.exists(): continue
        m=json.loads(path.read_text())
        text+=f"| {pilot} | {m['rate']} | {m.get('keyframes','—')} | {m.get('loop_events','—')} | {m.get('distance_m',float('nan')):.3f} | {m.get('endpoint_m',float('nan')):.3f} | {m.get('max_speed_mps',float('nan')):.3f} | {m.get('mean_sim_lag_s','미계측')} / {m.get('max_sim_lag_s','미계측')} | {'PASS' if m['success'] else 'FAIL'} |\n"
    text+='\n승인된 v2: extrinsicRot=R.T, extrinsicRPY=R. 중력축 진단: [imu_frame_diagnostic.json](imu_frame_diagnostic.json). 상세: [task03_04.md](../logs/task03_04.md). 파일럿 지연은 /clock − odometry header stamp(시뮬레이션 초)이다.\n'
    text+='\n## v1 / v2 비교\n\nv1은 잘못된 RPY 설정이며 archive_v1_wrong_rpy에 보존했다. 발산은 거리/최대속도 sanity 실패를 뜻한다.\n\n| run | v1 종점 m | v2 종점 m | v1 발산 | v2 발산 |\n|---|---:|---:|---|---|\n'
    for name in RUNS:
        old=ROOT/'results/archive_v1_wrong_rpy'/name/'run_meta.json'
        new=ROOT/'results'/name/'run_meta.json'
        if old.exists() and new.exists():
            a=json.loads(old.read_text()); b=json.loads(new.read_text())
            text+=f"| {name} | {a.get('endpoint_m',float('nan')):.3f} | {b.get('endpoint_m',float('nan')):.3f} | {'아니오' if a.get('sanity_pass') else '예'} | {'아니오' if b.get('sanity_pass') else '예'} |\n"

    (ROOT/'results/verification_v2.json').write_text(json.dumps(dict(all_pass=all_ok,runs=details,endpoint_comparisons=comparisons,gravity_tilt_deg=diagnostic['configured_gravity_tilt_deg']),indent=2)+'\n')
    (ROOT/'results/experiments_summary.md').write_text(text)
    print(text)
    return all_ok
if __name__=='__main__': raise SystemExit(0 if verify() else 1)

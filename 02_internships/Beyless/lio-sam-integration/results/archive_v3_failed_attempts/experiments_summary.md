# TASK-03+04 실험 검증

최종 최적화 keyframe 궤적 기준. 처리시간은 기동·재생·drain·저장·종료를 포함한 wall time(초).

| run | rate | 처리시간 s | keyframes | loop events | 누적거리 m | 시작–끝 m | sanity | 전체 run 검증 |
|---|---:|---:|---:|---:|---:|---:|---|---|
| loop_on_r1 | 1.0 | 124.9 | 158 | 2 | 235.077 | 0.291 | PASS | PASS |
| loop_on_r2 | 1.0 | 124.9 | 151 | 3 | 234.685 | 0.120 | PASS | PASS |
| loop_on_r3 | 1.0 | 125.1 | 167 | 2 | 235.578 | 0.221 | PASS | PASS |
| loop_off_r1 | 1.0 | 124.9 | 163 | 0 | 235.403 | 0.701 | PASS | PASS |
| loop_off_r2 | 1.0 | 124.9 | 155 | 0 | 235.648 | 0.564 | PASS | PASS |
| loop_off_r3 | 1.0 | 124.9 | 165 | 0 | 236.909 | 0.285 | PASS | PASS |
| even_loop_on | 1.0 | 124.9 | 141 | 0 | 1380.694 | 544.697 | FAIL | FAIL |

ON/OFF 동일 반복 번호별 종점거리 감소: [True, True, True]

전체 완료 기준: 미충족

### 반복 분산 (표본 표준편차, n=3)

- loop on distance_m: mean=235.113017, std=0.447651, variance=0.200392
- loop on endpoint_m: mean=0.210827, std=0.086250, variance=0.007439
- loop on keyframes: mean=158.666667, std=8.020806, variance=64.333333
- loop on loop_events: mean=2.333333, std=0.577350, variance=0.333333
- loop off distance_m: mean=235.986930, std=0.807940, variance=0.652767
- loop off endpoint_m: mean=0.516619, std=0.211625, variance=0.044785
- loop off keyframes: mean=161.000000, std=5.291503, variance=28.000000
- loop off loop_events: mean=0.000000, std=0.000000, variance=0.000000

미충족 run은 위 표와 각 run_meta.json 및 logs/task03_04.md의 v2 분석을 참조한다.

최종 배치 설정: historyKeyframeFitnessScore=0.5, radius=15.0 m, timeDiff=30.0 s.

RPY 정정 후 fitness=0.3 기본 배치 7회는 [archive_v2_fitness03/experiments_summary.md](archive_v2_fitness03/experiments_summary.md)에 전체 보존했다. 해당 배치는 sanity 7/7이나 ON r2 loop=0, ON/OFF 종점 비교 1/3 통과였다. 이후 fitness만 0.5로 조정하고 7회 전체를 다시 수행했다. 파일럿은 fitness=0.3이며 1배속 통과로 rate를 결정했다. 별도 fitness 0.5 trial을 최종 반복에 포함하지 않았다.

## 진단 파일럿

| run | rate | keyframes | loops | 누적거리 m | 종점거리 m | 최대속도 m/s | 평균/최대 지연 s | 결과 |
|---|---:|---:|---:|---:|---:|---:|---|---|---|

승인된 v2: extrinsicRot=R.T, extrinsicRPY=R. 중력축 진단: [imu_frame_diagnostic.json](imu_frame_diagnostic.json). 상세: [task03_04.md](../logs/task03_04.md). 파일럿 지연은 /clock − odometry header stamp(시뮬레이션 초)이다.

## v1 / v2 비교

v1은 잘못된 RPY 설정이며 archive_v1_wrong_rpy에 보존했다. 발산은 거리/최대속도 sanity 실패를 뜻한다.

| run | v1 종점 m | v2 종점 m | v1 발산 | v2 발산 |
|---|---:|---:|---|---|
| loop_on_r1 | 0.623 | 0.291 | 아니오 | 아니오 |
| loop_on_r2 | 1.036 | 0.120 | 아니오 | 아니오 |
| loop_on_r3 | 0.808 | 0.221 | 아니오 | 아니오 |
| loop_off_r1 | 5.501 | 0.701 | 아니오 | 아니오 |
| loop_off_r2 | 21.376 | 0.564 | 아니오 | 아니오 |
| loop_off_r3 | 5.511 | 0.285 | 아니오 | 아니오 |
| even_loop_on | 986.064 | 544.697 | 예 | 예 |

# TASK-03+04 실험 검증

최종 최적화 keyframe 궤적 기준. 처리시간은 기동·재생·drain·저장·종료를 포함한 wall time(초).

| run | rate | 처리시간 s | keyframes | loop events | 누적거리 m | 시작–끝 m | sanity | 전체 run 검증 |
|---|---:|---:|---:|---:|---:|---:|---|---|
| loop_on_r1 | 1.0 | 125.0 | 173 | 2 | 234.507 | 0.299 | PASS | PASS |
| loop_on_r2 | 1.0 | 125.0 | 175 | 0 | 236.074 | 8.784 | PASS | FAIL |
| loop_on_r3 | 1.0 | 125.0 | 175 | 2 | 238.789 | 0.921 | PASS | PASS |
| loop_off_r1 | 1.0 | 125.2 | 189 | 0 | 235.285 | 0.306 | PASS | PASS |
| loop_off_r2 | 1.0 | 125.0 | 169 | 0 | 235.488 | 0.456 | PASS | PASS |
| loop_off_r3 | 1.0 | 125.0 | 190 | 0 | 235.385 | 0.370 | PASS | PASS |
| even_loop_on | 1.0 | 125.2 | 166 | 2 | 234.817 | 0.136 | PASS | PASS |

ON/OFF 동일 반복 번호별 종점거리 감소: [True, False, False]

전체 완료 기준: 미충족

### 반복 분산 (표본 표준편차, n=3)

- loop on distance_m: mean=236.456725, std=2.166565, variance=4.694005
- loop on endpoint_m: mean=3.334332, std=4.729436, variance=22.367562
- loop on keyframes: mean=174.333333, std=1.154701, variance=1.333333
- loop on loop_events: mean=1.333333, std=1.154701, variance=1.333333
- loop off distance_m: mean=235.386141, std=0.101441, variance=0.010290
- loop off endpoint_m: mean=0.377549, std=0.075231, variance=0.005660
- loop off keyframes: mean=182.666667, std=11.846237, variance=140.333333
- loop off loop_events: mean=0.000000, std=0.000000, variance=0.000000

미충족 run은 위 표와 각 run_meta.json 및 logs/task03_04.md의 v2 분석을 참조한다.

## 진단 파일럿

| run | rate | keyframes | loops | 누적거리 m | 종점거리 m | 최대속도 m/s | 평균/최대 지연 s | 결과 |
|---|---:|---:|---:|---:|---:|---:|---|---|---|
| pilot_rate1 | 1.0 | 176 | 2 | 236.368 | 0.293 | 7.701 | 0.13676980599729371 / 0.23416566848754883 | PASS |
| pilot_rate05 | 0.5 | 164 | 3 | 235.926 | 1.233 | 4.746 | 0.12036244744994208 / 0.22022581100463867 | PASS |

승인된 v2: extrinsicRot=R.T, extrinsicRPY=R. 중력축 진단: [imu_frame_diagnostic.json](imu_frame_diagnostic.json). 상세: [task03_04.md](../logs/task03_04.md). 파일럿 지연은 /clock − odometry header stamp(시뮬레이션 초)이다.

## v1 / v2 비교

v1은 잘못된 RPY 설정이며 archive_v1_wrong_rpy에 보존했다. 발산은 거리/최대속도 sanity 실패를 뜻한다.

| run | v1 종점 m | v2 종점 m | v1 발산 | v2 발산 |
|---|---:|---:|---|---|
| loop_on_r1 | 0.623 | 0.299 | 아니오 | 아니오 |
| loop_on_r2 | 1.036 | 8.784 | 아니오 | 아니오 |
| loop_on_r3 | 0.808 | 0.921 | 아니오 | 아니오 |
| loop_off_r1 | 5.501 | 0.306 | 아니오 | 아니오 |
| loop_off_r2 | 21.376 | 0.456 | 아니오 | 아니오 |
| loop_off_r3 | 5.511 | 0.370 | 아니오 | 아니오 |
| even_loop_on | 986.064 | 0.136 | 예 | 아니오 |

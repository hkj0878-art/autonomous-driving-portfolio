동일 주행의 held-out(홀수) 스캔 → 진짜 재주행보다 낙관적. 다른 날 주행 bag으로 같은 스크립트를 재사용 가능

# TASK-05b — fixed-prior localization

| Trajectory | 2D ATE SE3 RMSE (m) | 2D ATE SE3+lever2D RMSE (m) |
|---|---:|---:|
| Held-out localization (all scans) | 0.6893 | 0.2589 |
| Even mapping (keyframes) | 0.6637 | 0.2396 |
| Localization (common timestamps) | 0.6971 | 0.2586 |
| Even mapping (common timestamps) | 0.7066 | 0.2586 |

완주 1089/1089 scans, LOST=0; 실패 0 (0.00%).
Fitness mean 0.9998. 처리 평균/최대 82.6/379.5 ms, p95 170.0 ms.
10 Hz 기준 실시간 계수 = sensor budget / compute time = **1.21×** (1 이상이면 실시간).
최대 RSS 523 MiB. Scan 시간은 전처리+예측+정합이며 bag I/O·초기 맵 구축·평가/그림은 제외한다.

## Initial-pose robustness

| Perturbation | Success / trials | Rate |
|---|---:|---:|
| 0.5 m / 2° | 50/50 | 100% |
| 1 m / 5° | 50/50 | 100% |
| 2 m / 10° | 48/50 | 96% |
| 3 m / 15° | 32/50 | 64% |

## Interpretation

공통 시각의 lever2D ATE 기준 위치추정과 매핑 차이는 -0.0000028 m (양수는 위치추정 개선)이다.
홀수 스캔 1089개를 고정 prior에 정합했고 GNSS는 평가에만 사용했다. IMU 입력과 매핑 포즈의 매 스캔 재초기화는 없다.
강건성 200회 중 180회가 30스캔 후 3D 위치 <0.3 m 및 yaw <1°를 만족했다. 실패 정합은 예측값 유지, 5회 연속 실패는 LOST 처리했다.

## Method and limitations

- Prior: data/lio_bag, even scans only, 0.2 m voxel, fixed even mapping final poses. Metadata: `{'scans_read': 2178, 'scans_used': [1052], 'voxel_m': 0.2, 'scan_filter': 'even', 'deskew': False, 'relative_time': 'voxel weighted mean of original scan-relative seconds'}`.
- Query: 1–80 m, 0.3 m voxel, no deskew; constant world velocity/body angular velocity prediction from the last two output poses, actual timestamp intervals.
- GICP: 1.0 → 0.5 m correspondence gates, max 30 iterations/stage, absolute fitness/RMSE change <1e-6. Map 20-NN planar covariances and KD-tree built once; scan covariance built once per scan.
- Open3D public registration_generalized_icp rebuilds the tree and hides iterations. The cached loop uses its TransformationEstimationForGeneralizedICP solver; synthetic and real-scan cross-checks compare against the public API. Source: https://github.com/isl-org/Open3D/blob/main/cpp/open3d/pipelines/registration/GeneralizedICP.cpp
- 첫 map keyframe은 첫 홀수 스캔보다 0.950 s 늦다. 최초 포즈는 가장 가까운 첫 keyframe으로 고정 초기화(시작 정지 가정); 범위 밖 38개도 전부 위치추정했다. 강건성 기준 포즈에는 외삽을 쓰지 않았다.
- 5 starts are equally spaced by map-path arc length over feasible 30-scan windows. Per seed: exact horizontal perturbation magnitude, random uniform direction, random yaw sign; paired directions/signs across levels. No translation z perturbation. End errors use even-pose SLERP/linear interpolation, not GNSS.
- Robustness is relative to the even mapping trajectory, which also has map/trajectory approximation errors. Success is not a GNSS accuracy claim. Early LOST trials count as failures; no extra recovery or tuning by seed.
- Same-run held-out scans share the environment, IMU-derived map geometry and mapping drift. Split reduces direct scan reuse but cannot establish independent revisit performance. Map poses are used only for initial poses and robustness scoring.
- SE3/lever2D alignments are globally fitted to GNSS for evaluation; lever estimates are nuisance fits. GNSS tangent and flat-ground approximations are inherited from TASK-05. No GNSS/IMU enters localization.
- 별도 날짜의 bag에는 --map-run/--map-bag/--query-bag/--gt 및 별도 --out을 사용한다. 맵 좌표계의 초기 포즈를 --initial-pose으로 제공할 수 있다. 강건성 평가에는 쿼리 시각에 대응하는 별도의 --reference-traj가 필요하다.

Reproduce:

```bash
source env.sh
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 .venv/bin/python -m tools.loc.run_loc_eval --resume
```

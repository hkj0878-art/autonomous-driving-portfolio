# TASK-05b — held-out fixed-map localization

작업 환경: `/home/hong/SLAM`, `source env.sh`, `.venv/bin/python`.
sudo 및 시스템 설치는 사용하지 않았다. 데이터와 기존 결과는 읽기 전용으로
접근하며, 모든 실행 산출물은 `results/loc/` 아래에 기록한다.

## 구현

- `tools/loc/localize.py`: 고정 prior 대상의 Open3D GICP, 1.0→0.5 m 2단계,
  스캔 범위 1–80 m 및 voxel 0.3 m, 등속/등각속 예측(실제 timestamp 간격),
  실패 시 예측 유지, 연속 5회 실패 LOST. IMU/GNSS 입력은 없다.
- `tools/loc/run_loc_eval.py`: 짝수 prior 생성(0.2 m), 전체 홀수 주행,
  GNSS ATE(SE3/lever2D), 기본 even keyframe 기준선 및 공통 시각 기준선,
  5 시작점×4 섭동×10 시드, CSV/JSON 체크포인트와 4개 그림.
- `tools/loc/verify_loc.py`: 기본 실험 독립 검증. parity 및 시각 분리,
  전 스캔 완주, 실패/성공 판정, 200개 조합, 메모리, 입력 보존,
  실제 스캔 3개에 대한 공개 Open3D GICP API 교차검증.
- `tools/tests/test_loc.py`: 알려진 변환 복원과 공개 API 일치, 실패 경계값,
  LOST 예측 유지, 등속 예측, 보간 및 출력 경로 보호 테스트.
- 추가 정리: `tools/tests/test_maps.py`의 보조 `map2d_nav2.yaml` 요구를 제거.
  단일 `map2d.yaml`, `free_thresh=0.196`, unknown 205 보존을 검증한다.
- `pytest.ini`: 프로젝트 전체인 `tools/` 아래 테스트를 수집한다.
  기본 무제한 검색으로는 빌드되지 않은 vendored GTSAM Python binding의
  자체 테스트 수집이 실패했다. third_party 테스트는 별도 빌드 환경의 대상이다.

## 환경 정리 및 실행

`.venv`는 system-site-packages가 켜져 있고 사용자 site의 numpy 2.x와
matplotlib이 섞여 있었다. 사용자 site를 `PYTHONNOUSERSITE=1`로 제외하고
필요한 의존성을 `.venv` 안에 설치했다. numpy 1.26.4, scipy 1.15.3,
matplotlib 3.10.8, Open3D 0.20.0. 정확한 패키지 목록과 설치 로그는
`results/loc/venv_packages.txt`, `dependencies*.log`에 보관했다.
2D Agg 그림은 정상 생성되며, 시스템 mpl_toolkits와의 3D import 경고는
이 작업의 2D 그림에 영향을 주지 않는다.

```bash
source env.sh
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv/bin/python -m tools.loc.run_loc_eval --resume
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv/bin/python -m pytest -q
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv/bin/python -m tools.loc.verify_loc
```

초기 별도 prior 생성에는 `tools.eval.build_map_from_poses` CLI를 썼고,
`run_loc_eval --resume`가 동일한 prior를 재사용했다. 독립 재생성 결과
676,030개 xyz가 비트 단위로 동일했다. `/usr/bin/time -v` 측정 prior
생성 최대 RSS 346,944 KiB(339 MiB), 51.37초. 메타데이터와 검증 로그는
`prior_reproducibility.json`, `map_resource_check.log`에 기록했다.

## API 적응 및 근사/한계

Open3D `registration_generalized_icp`는 호출마다 target tree를 다시 만들고
반복 수를 반환하지 않는다. 이를 그대로 매 스캔 호출하면 고정 tree/실제
반복 수 기록 조건을 만족시킬 수 없다. 따라서 동일한 Open3D
`TransformationEstimationForGeneralizedICP.compute_transformation`을 호출하는
ICP 외부 루프를 구현했다. target cKDTree와 20-NN normal 기반
(planar epsilon/1/1) 공분산은 한 번 생성하고 재사용한다. 반복 중에는
변환된 source 공분산만 회전한다. fitness와 RMSE는 유클리드 최근접 대응점
기준이며, 두 지표 변화가 각각 1e-6 미만이면 수렴한다(단계당 최대 30회).
공개 API와 동일한 합성 테스트 및 실제 스캔 교차검증으로 확인한다.

근거: [Open3D GICP 소스](https://github.com/isl-org/Open3D/blob/main/cpp/open3d/pipelines/registration/GeneralizedICP.cpp),
[공개 API](https://www.open3d.org/docs/release/python_api/open3d.pipelines.registration.registration_generalized_icp.html).

- 문서의 GT 경로 `results/gt_gnss_enu_tum.txt`는 실제로 존재하지 않는다.
  기존 결과를 이동하지 않고 `results/eval/gt_gnss_enu_tum.txt`를 읽었다.
- 첫 최적화 keyframe이 첫 홀수 스캔보다 약 0.95 s 늦다. 초기 포즈는 첫
  keyframe을 유지하는 시작 정지 가정으로 제공하여 1,089개 모두 처리한다.
  강건성의 기준 포즈는 SLERP/선형보간 범위 안에서만 사용한다.
- 동일 주행의 held-out(홀수) 스캔 → 진짜 재주행보다 낙관적.
  맵은 IMU 기반 even 매핑 포즈를 사용하므로 간접적인 공통 오차는 남는다.
  IMU를 위치추정 예측에 사용하지 않으며 GNSS는 평가 함수에서만 사용한다.
- 디스큐 없음. 맵과 쿼리 스캔 모두 scan header pose 근사이다.
- 5개 시작점은 가능한 30스캔 구간의 경로 누적거리 기준으로 균등 배치한다.
  섭동은 수평 크기를 정확히 유지하고 방향을 uniform 샘플링, yaw 부호는
  무작위로 선택한다. 시작점/시드가 같으면 수준 간 방향과 부호를 공유한다.
- 강건성 성공은 맵 포즈 대비 마지막 3D 위치 <0.3 m 및 yaw <1°이며,
  조기 LOST는 실패로 센다. GNSS 정확도나 참된 재주행 성공률을 뜻하지 않는다.
- keyframe 기반 ATE와 dense scan ATE의 표본 차이가 있으므로 공통 시각
  비교 표를 추가했다. GT 정렬/lever2D 피팅은 평가 단계의 in-sample 추정이다.

## 직접 검증 결과

- 기본 실험: 홀수 **1,089/1,089** scans 완주, 실패 0, LOST 0.
- ATE 2D RMSE: SE3 **0.689273 m**,
  SE3+lever2D **0.258905 m**.
- 강건성 **200회 전부 실행**, 180회 성공. 수준별 50/50, 50/50,
  48/50, 32/50 = 100%, 100%, 96%, 64%. LOST 조기 종료 5회는 실패로 포함.
- 스캔당 평균/최대 **82.60/379.51 ms**, p95 169.98 ms, 10 Hz 기준 평균
  실시간 계수 **1.21×**. 최대 지연은 100 ms를 넘으므로 매 스캔 deadline
  보장을 의미하지 않는다. 원래 전체 평가 wall time 566.97초.
- 최대 RSS: 최초 실행 525.1 MiB, 최종 resume 포함
  525.1 MiB. prior 생성 포함 모두 4 GiB 이하.
- **전체 pytest 71 passed, 0 failed** (`.venv/bin/python -m pytest -q`).
- 그림 4개를 직접 열어 확인: 궤적 좌표/축/범례, 오류 시계열, fitness/RMSE,
  4×5 히트맵의 값과 색이 정상이며 빈 그림/축 뒤집힘 없음.
- 실제 20/500/1000번째 홀수 스캔에서 공개 GICP와 cached GICP 교차검증:
  최대 위치 차이 약 1.3e-14 m, 회전 차이 약 4.1e-16 rad.
- 짝수/홀수 각각 1,089 scans, 원래 bag의 [::2]/[1::2]와 정확히 일치,
  timestamp 교집합 없음. 기존 data/results **894개 파일**의 집합/크기/mtime_ns 동일.
- `results/loc/verification.json`의 passed=true 확인.

## 결과 해석 (3줄)

1. 고정 even prior에 대해 홀수 스캔은 완주했지만 공통 시각 lever2D ATE는
   매핑과 둘 다 약 0.25857 m로, 정확도 개선의 근거는 없다.
2. 1 m/5°까지 100%, 2 m/10°에서 96%, 3 m/15°에서 64%로 초기 오차가
   커질수록 시작 위치에 따라 수렴 능력이 달라진다.
3. 동일 주행 split, 디스큐 생략, in-sample 정렬/lever 피팅을 포함한
   낙관적 평가이며 별도 날짜의 실제 재주행 성능을 보장하지 않는다.

## 최종 재현성 정리

별도 날짜의 --reference-traj 사용 시 원래 map-run 시각 대신 쿼리 시각의
reference로 기준선을 평가하도록 오케스트레이터를 수정했다. 기본 실험에서는
reference=mapping이므로 모든 정합/시드 결과가 동일하다. 원래 실행 소스와
설정, 통계는 results/loc/execution_source/에 보관했고 cache_migration.json에
변경 전후 hash 및 재사용 근거를 남겼다. localize.py와 설정은 변경하지 않았다.
기존 200회 체크포인트를 재사용해 평가/그림을 재생성하고 전체 pytest와
독립 검증을 다시 실행했다. 최종 --resume도 정상 종료했다.

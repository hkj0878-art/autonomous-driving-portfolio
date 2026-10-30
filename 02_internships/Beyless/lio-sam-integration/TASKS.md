# SLAM 프로젝트 — LIO-SAM (ROS 2 Humble) 매핑 + GNSS 정확도 평가

## 전체 목표
보유 rosbag(LiDAR+IMU+RTK GNSS)으로 LIO-SAM 매핑 파이프라인을 직접 구성하고, 루프 클로저 유무에 따른 궤적 정확도를 RTK GNSS 기준으로 정량 평가해 포트폴리오용 결과물(2D 지도, 3D PCD, 평가 리포트)을 만든다.

## 데이터 분석 결과 (data/synced_bag, 이미 ROS 2 sqlite3 bag — 변환 불필요, 전처리만 필요)
| 토픽 | 타입 | 주기 | frame | 비고 |
|---|---|---|---|---|
| /front/lidar_point_synced | PointCloud2 | 20 Hz | front_lidar | 128 ring, 수평 FOV 약 -65~+61°(전방 전용, Hesai AT128 계열로 추정), 스캔당 78336점 고정(무효점은 0,0,0), 필드: x y z intensity(f32) ring(u16 @16) timestamp(f64 @18, **절대시각이 2020년 epoch** — 헤더 stamp와 고정 오프셋 약 169765152.46 s), 스캔 내 시간 폭 약 25 ms |
| /imu/data_synced | Imu | 100 Hz | imu_link | Xsens MTi(추정), orientation 있음(9축, yaw 안정적), z 가속도 +9.8 → z-up |
| /fix_synced | NavSatFix | 32 Hz | gps | status=2(GBAS/RTK), σ≈1.3–1.5 cm → **정답(GT)으로 사용** |
- 길이 108.7 s, 주행거리 약 237 m, 약 70×60 m 사각형 **폐루프 1바퀴**(시계방향, 시작-끝 0.47 m), 평지(고도 ±0.1 m)
- 헤더 stamp(2025-11-11)와 bag 기록시각(2025-11-13)이 다름 → **헤더 stamp 기준**으로 처리
- /tf, /tf_static 없음 → 외부 파라미터는 사용자 제공 캘리브레이션 사용(아래)

## IMU–LiDAR 외부 파라미터 (사용자 제공, 2026-10-26) — 검증 완료
제공값(사용자 표기 "T_lb lidar->imu"):
```
t = [0.01027, -0.297547, 0.042205]
R = [ 0.986903,  0.007064,  0.161160,
     -0.005760,  0.999947, -0.008561,
     -0.161212,  0.007520,  0.986891 ]   # pitch≈9.28°, roll≈0.44°, yaw≈-0.33°
```
**검증**: 첫 5스캔의 RANSAC 지면 법선(lidar frame)과 IMU 중력방향 비교 → `R.T @ up_imu` 오차 0.55°, `R @ up_imu` 18.8°, 단위행렬 9.6°.
⇒ 제공된 R은 **p_imu = R · p_lidar** (lidar→imu)이다. LIO-SAM의 `extrinsicRot`/`extrinsicRPY`는 IMU 측정을 lidar frame으로 돌리는 행렬(`acc_lidar = extRot · acc_imu`)이므로 **R의 전치(R.T)를 넣어야 한다**.
LIO-SAM 설정값:
```yaml
extrinsicTrans: [0.01027, -0.297547, 0.042205]
extrinsicRot: [0.986903, -0.005760, -0.161212,
               0.007064,  0.999947,  0.007520,
               0.161160, -0.008561,  0.986891]
extrinsicRPY: [0.986903,  0.007064,  0.161160,     # ⚠️ 정정(2026-10-26): R 원본(전치 아님)
               -0.005760,  0.999947, -0.008561,
               -0.161212,  0.007520,  0.986891]
# 근거: utility.hpp:319 acc=extRot·acc, :331 q_final=q_from·Quat(extRPY) → 일관성 조건 extRPY·extRot=I → extRPY=R.
# 최초 설계(extRPY=extRot=R.T)는 중력축 18.57° 불일치를 일으킴 (results/imu_frame_diagnostic.json, Codex 발견)
```
(translation은 원본 값을 사용한다. LIO-SAM은 회전 없는 lever arm으로만 쓰며, 9° 회전에 따른 차이는 약 5 cm로 무시 가능하다. 리포트 한계 항목에 기재한다.)

## 아키텍처 결정사항
- LIO-SAM `ros2` 브랜치(TixiaoShan/LIO-SAM)를 그대로 사용하고, 알고리즘 바깥(전처리·설정·평가·지도 내보내기)을 직접 구현한다.
- GTSAM은 sudo 없이 소스 빌드하여 `~/SLAM/third_party/install`에 설치한다(시스템 경로 수정 금지).
- 포인트클라우드는 오프라인 전처리로 LIO-SAM velodyne 형식(x,y,z,intensity f32 / ring u16 / time f32 = 스캔 시작 기준 상대초)으로 변환한 새 bag을 만든다. 실시간 변환 노드보다 재현성과 디버깅이 쉽다.
- GNSS는 평가용 GT로만 쓴다. 기본 실험에서는 LIO-SAM GPS factor를 끈다(GT 누출 방지). GPS 융합은 선택 실험으로 분리한다.
- 루프 테스트 = 같은 bag을 **루프 클로저 ON/OFF**로 매핑해 ATE/RPE/종점 드리프트를 비교한다.
- 평가 도구는 venv(`~/SLAM/.venv`)에 둔다. 시스템 numpy 2.x와 ROS Humble의 충돌을 피하기 위해서다.
- 피할 것: ~/ros2_ws, ~/catkin_ws 수정, 시스템 패키지 설치, 원본 bag 수정.

## 디렉터리 구조
```
SLAM/
  data/synced_bag/          원본(읽기 전용)
  data/lio_bag/             전처리된 bag
  third_party/              gtsam 소스/설치
  ws/src/LIO-SAM            colcon 워크스페이스
  tools/                    전처리·기록·평가·지도변환 스크립트(+tests)
  config/                   LIO-SAM params yaml
  results/<run_name>/       traj_tum.txt, map pcd, metrics.json, plots
  report/REPORT.md          포트폴리오 리포트
```

## 태스크 목록
- [x] TASK-01: 환경 구축 (GTSAM 소스 빌드 + LIO-SAM ros2 빌드)
- [x] TASK-02: bag 전처리 (point time 변환, 무효점 제거) + 데이터 리포트
- [x] TASK-03: 외부 파라미터 확인 + LIO-SAM 설정 + 실행/궤적 기록 파이프라인
- [x] TASK-04: 실험 실행 (loop ON / loop OFF / (선택) GPS 융합)
- [x] TASK-05: GNSS 대비 정확도 평가 (ATE, RPE, 종점 드리프트, 그래프)
- [x] TASK-06: 지도 산출물 (3D PCD, 2D occupancy grid PGM/YAML, 렌더 이미지)
- [x] TASK-07: 포트폴리오 리포트 (REPORT.md + README 재현 절차)

## 실행 순서 및 의존성
TASK-01 ─┐
TASK-02 ─┴→ TASK-03 → TASK-04 → (TASK-05 ∥ TASK-06) → TASK-07
(TASK-01과 TASK-02는 병렬 가능)

---

### [CODEX-TASK] TASK-01: 환경 구축 (GTSAM + LIO-SAM ros2)

**목적**
LIO-SAM을 ROS 2 Humble에서 빌드하고 실행할 수 있게 한다.

**입력**
- 시스템: Ubuntu 22.04, ROS 2 Humble(/opt/ros/humble), 16코어, RAM 14 GB(가용 약 4 GB)
- 설치되어 있음: libboost-all-dev 1.74, libeigen3-dev 3.4, libpcl-dev 1.12, libtbb-dev
- 의존성: 없음

**출력**
- /home/hong/SLAM/third_party/gtsam (소스, 4.1.1 태그 권장 — LIO-SAM 검증 버전), /home/hong/SLAM/third_party/install (설치 prefix)
- /home/hong/SLAM/ws/src/LIO-SAM (ros2 브랜치), /home/hong/SLAM/ws/install
- /home/hong/SLAM/env.sh: ROS + GTSAM LD_LIBRARY_PATH/CMAKE_PREFIX_PATH + ws overlay를 source하는 스크립트
- /home/hong/SLAM/logs/task01_build.md: 사용한 버전, 커맨드, 이슈와 해결 방법

**구현 조건**
- sudo 사용 불가. 시스템 경로나 /opt에 쓰지 않는다. apt 패키지가 더 필요하면 직접 설치하지 말고 로그에 "사용자 sudo 필요: <명령>"으로 남기고 중단한다.
- GTSAM cmake: -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_WITH_TBB=ON(문제가 생기면 OFF), CMAKE_INSTALL_PREFIX=third_party/install
- 메모리가 적으므로 make는 -j6 이하, colcon은 --parallel-workers 2 및 MAKEFLAGS=-j4로 빌드한다.
- LIO-SAM 소스 수정은 빌드 호환성 문제일 때만 최소한으로 하고, 수정 내용을 diff로 로그에 남긴다.
- ~/ros2_ws, ~/catkin_ws는 건드리지 않는다.

**완료 기준**
- [x] `source ~/SLAM/env.sh && ros2 pkg executables lio_sam`에 imageProjection, imuPreintegration, featureExtraction, mapOptimization이 나온다
- [x] 각 노드가 기본 params로 5초 동안 크래시 없이 기동된다(공유 라이브러리 에러 없음)
- [x] task01_build.md 작성

**컨텍스트**
작업 루트는 /home/hong/SLAM이다. 전체 설계는 TASKS.md를 참고한다. 이 태스크에서는 데이터 처리를 하지 않는다.

---

### [CODEX-REVISION] TASK-01 재작업 요청

**유지할 것**
- GTSAM 4.1.1 / LIO-SAM ros2 소스 클론, 버전 기록, 의존성 조사 결과

**수정할 것**
1. xacro 미설치로 중단 → **xacro는 블로커가 아니다.** 업스트림 run.launch.py의 robot_state_publisher용일 뿐이며 C++ 빌드/노드 실행에는 필요 없다. package.xml exec_depend는 colcon 빌드를 막지 않는다. 설치 요청 없이 그대로 진행한다.
2. 업스트림 launch 대신 `ws/src/LIO-SAM`을 수정하지 말고 `/home/hong/SLAM/launch/lio_sam_nox.launch.py`를 새로 작성한다. 4개 노드 + static_transform_publisher(map→odom identity 등 업스트림 launch가 내보내는 TF 중 xacro 외 항목)를 포함하고, params 파일 경로와 use_sim_time을 인자로 받는다. rviz는 인자로 on/off한다(기본 off).

**추가할 것**
- 메모리 주의: swap이 거의 가득 차 있다. GTSAM make -j4, colcon MAKEFLAGS=-j3 --parallel-workers 1로 빌드한다. OOM(killed)이 나면 -j2로 낮춰 재시도한다.
- 기동 시험은 새 launch로 5초 실행해서 확인한다.

**완료 기준 (재확인)**
- [x] `source ~/SLAM/env.sh && ros2 pkg executables lio_sam`에 4개 노드가 나온다
- [x] `ros2 launch /home/hong/SLAM/launch/lio_sam_nox.launch.py`가 5초 동안 크래시와 공유 라이브러리 에러 없이 기동된다
- [x] logs/task01_build.md 갱신

---

### [CODEX-TASK] TASK-02: bag 전처리 + 데이터 리포트

**목적**
원본 LiDAR 포인트를 LIO-SAM이 요구하는 형식으로 변환한 bag을 만들어, 디스큐(deskew)와 IMU 동기화가 올바르게 동작하게 한다.

**입력**
- 파일: /home/hong/SLAM/data/synced_bag (읽기 전용, sqlite3, 4.4 GB, 16360 msgs)
- 데이터: 상단 "데이터 분석 결과" 표 참고
- 의존성: 없음 (TASK-01과 병렬 실행, LIO-SAM 빌드 결과 불필요)

**출력**
- /home/hong/SLAM/tools/preprocess_bag.py (CLI: --in, --out, --overwrite)
- /home/hong/SLAM/tools/tests/test_preprocess.py (pytest)
- /home/hong/SLAM/data/lio_bag/ (변환된 rosbag2, sqlite3)
  - /points_raw  PointCloud2, frame_id `front_lidar`, 필드 순서 x,y,z,intensity(float32), ring(uint16), time(float32), point_step 일관, is_dense=True
  - /imu_raw     원본 /imu/data_synced 그대로(frame_id `imu_link`)
  - /gps/fix     원본 /fix_synced 그대로
  - 메시지 기록 시각(bag timestamp)은 **헤더 stamp**로 다시 기록한다. 원본 bag 기록시각은 2일 어긋나 있어 재생 시 /clock과 헤더가 일치하지 않기 때문이다.
- /home/hong/SLAM/results/data_report/: data_report.md, gnss_traj.png(ENU 궤적), timing.png(토픽별 stamp 간격)

**구현 조건**
- rosbag2_py(Reader/Writer)로 스트리밍 처리하고 전체를 메모리에 올리지 않는다. 다른 빌드가 동시에 돌고 있으므로 RAM은 1 GB 이하로 쓴다.
- 포인트 시간 변환:
  1. 스캔마다 `offset = header_stamp - min(pt.timestamp)`를 계산한다. 오프셋이 전 스캔에서 일정한지(표준편차, 최대편차) 리포트에 기록한다(예상값 약 169765152.46 s, 편차 1 ms 미만).
  2. `time = pt.timestamp - min(pt.timestamp)` (float32, 0~약 0.025 s)
  3. 헤더 stamp는 원본을 유지한다(= 스캔 시작). 오프셋 편차가 1 ms 이상이면 중단하고 리포트에 원인을 쓴다.
- 무효점 제거: NaN/inf, (0,0,0), range < 0.5 m. 제거 비율을 통계로 남긴다.
- 포인트 순서는 유지한다. ring 값은 그대로 둔다(0–127).
- 리포트에 기록할 것: ring별 포인트 수, 링당 수평 포인트 수와 방위각 해상도(°) — TASK-03에서 Horizon_SCAN 결정에 쓴다(LIO-SAM은 360°를 Horizon_SCAN칸으로 나누므로 360/해상도가 필요). 포인트 시간이 링 내 방위각과 어떤 관계인지(스캔 패턴)도 기록한다.
- 매직 넘버는 모듈 상단 상수로 둔다. 에러는 명시적으로 처리한다(토픽 누락, 필드 누락 → 명확한 예외).
- 원본 bag 수정 금지.

**완료 기준**
- [ ] data/lio_bag 생성, `ros2 bag info`에서 3토픽의 메시지 수가 원본과 같다(2178/10884/3298)
- [ ] 변환된 첫/중간/마지막 스캔에서 time ∈ [0, 0.05), is_dense=True, NaN과 원점 점 없음을 스크립트로 검증한다
- [ ] bag 기록시각과 헤더 stamp가 일치한다(차이 < 1 ms)
- [ ] pytest 통과(합성 PointCloud2로 변환 함수 단위 테스트: 시간 변환, 무효점 제거, 필드 레이아웃)
- [ ] data_report.md와 그림 2장 생성

**컨텍스트**
ROS 2 Humble: `source /opt/ros/humble/setup.bash`. 시스템 numpy는 2.2.6이며 sensor_msgs_py.point_cloud2.read_points는 동작이 확인됐다. 원본 필드 오프셋: x0 y4 z8 intensity12(f32), ring16(u16), timestamp18(f64, 정렬되지 않음), point_step은 메시지에서 읽는다. LIO-SAM(velodyne 모드)은 is_dense=false면 에러를 내고, ring/time 필드가 없어도 에러를 낸다.

---

## 설계 추가 (2026-10-26): 맵 품질 평가 지표 — TASK-05 확장 + TASK-05b 신설
TASK-05 평가 항목을 다음 세 그룹으로 확장한다.
1. **궤적 정확도 (GT 대비)**: ATE RMSE/mean/max (SE3 정렬, 스케일 없음), RPE(10 m/50 m 구간, 위치 % + 회전 deg/m), 종점 드리프트, 루프 ON/OFF 비교
2. **맵 내부 품질 (GT 불필요)**: MME(Mean Map Entropy), MPV(Mean Plane Variance) — 반경 0.3 m 이웃, 같은 복셀 다운샘플 기준으로 run 간 비교. **루프 겹침 구간 일관성**: 주행 처음 15 s와 마지막 15 s에 쌓인 점군끼리 cloud-to-cloud 거리(median, p95)를 구한다(이중 벽이나 고스팅의 정량 지표)
3. **맵 기반 재주행(위치추정) 테스트 — TASK-05b**: 루프 ON 맵을 prior map으로 고정하고, 스캔별 scan-to-map 정합(small_gicp 또는 Open3D GICP/NDT, IMU 예측 초기값 + 초기 위치에 1 m/5° 섭동)으로 궤적을 추정해 GNSS와 ATE를 비교한다. 정합 실패율과 fitness도 기록한다.
   - 공정성: 같은 bag을 쓰므로 **짝수 스캔으로 맵을 만들고 홀수 스캔으로 위치추정**하는 split으로 누출을 줄인다. 리포트에는 "동일 주행 기반 → 낙관적 추정치"라고 명시한다. 같은 경로를 다시 주행한 bag을 받으면 같은 스크립트로 진짜 재주행 평가를 한다.

---

## 리뷰 결과: TASK-01 (재작업), TASK-02 — 둘 다 APPROVE (2026-10-26)
- TASK-01: GTSAM 4.1.1 + LIO-SAM ros2 빌드, env.sh, launch/lio_sam_nox.launch.py 완료. 위험 요소: OpenCV의 libtbb.so.2와 GTSAM의 libtbb.so.12가 함께 링크됨 → 매핑 부하에서 크래시가 나면 GTSAM_WITH_TBB=OFF로 다시 빌드한다(TASK-03에서 감시).
- TASK-02: data/lio_bag 생성, 검증 통과(pytest 11). 핵심 수치: 방위각 해상도 0.2° → **Horizon_SCAN=1800**, 유효점 약 3.9만/스캔, 상위 ring 119–127 일부는 항상 비어 있음(정상), 스캔 시간 폭 25 ms, 시간과 방위각이 선형(디스큐 OK).

---

### [CODEX-TASK] TASK-03+04: LIO-SAM 설정 + 실행 파이프라인 + 실험 실행

**목적**
변환된 bag으로 LIO-SAM을 재현 가능하게 돌리고, 루프 클로저 ON/OFF 실험의 궤적과 지도를 저장한다.

**입력**
- 파일: data/lio_bag (/points_raw, /imu_raw, /gps/fix), env.sh, launch/lio_sam_nox.launch.py, ws/src/LIO-SAM/config/params.yaml(템플릿)
- 데이터: 위 "IMU–LiDAR 외부 파라미터" 절의 **LIO-SAM 설정값(전치 적용된 값)** 그대로 사용
- 의존성: TASK-01, TASK-02 완료

**출력**
- config/params_base.yaml, config/params_loop_off.yaml (loopClosureEnableFlag만 다름)
- tools/run_lio_sam.sh `<run_name> <params_yaml> [bag_dir] [rate]`: launch(use_sim_time) → `ros2 bag play --clock` → 재생 종료 후 drain 대기 → save_map 서비스 호출 → 종료. 로그는 results/<run>/run.log에 남긴다.
- tools/record_traj.py: 실행 중 /lio_sam/mapping/odometry를 TUM으로 저장(`odom_online_tum.txt`, timestamp는 header stamp 초)하고, 루프 클로저 이벤트 수(/lio_sam/mapping/loop_closure_constraints 마커 또는 로그)를 기록
- tools/pcd_to_tum.py: save_map이 만든 transformations.pcd(PointTypePose: x y z roll pitch yaw time)를 **최종 최적화 keyframe 궤적** TUM(`keyframes_final_tum.txt`)으로 변환한다. 루프 보정이 반영된 이 궤적이 메인 평가 대상이다.
- results/<run>/: run.log, odom_online_tum.txt, keyframes_final_tum.txt, map/(GlobalMap.pcd, CornerMap.pcd, SurfMap.pcd, trajectory.pcd, transformations.pcd), run_meta.json(params 해시, git commit, 재생 속도, 처리 시간, 키프레임 수, 루프 이벤트 수)
- 테스트: tools/tests/test_pcd_to_tum.py (합성 PointTypePose → TUM 쿼터니언 변환 정확성)

**구현 조건 (params_base.yaml)**
- pointCloudTopic /points_raw, imuTopic /imu_raw, odomTopic "odometry/imu", gpsTopic "odometry/gps"(이 토픽은 bag에 없음 → GPS factor가 사실상 비활성임을 로그로 확인. **GNSS는 절대 LIO-SAM 입력으로 넣지 않는다** — 평가 GT 누출 방지)
- lidarFrame과 baselinkFrame을 **둘 다 "front_lidar"**로 둔다(imuPreintegration.cpp:128의 TF lookup 회피, base_link TF 없음). odometryFrame "odom", mapFrame "map"
- sensor velodyne, N_SCAN 128, Horizon_SCAN 1800, downsampleRate 1, lidarMinRange 1.0, lidarMaxRange 100.0
- extrinsicTrans/extrinsicRot/extrinsicRPY: TASKS.md "LIO-SAM 설정값" 그대로
- IMU 노이즈와 gravity는 업스트림 기본값을 유지한다(값을 바꾸면 이유를 기록한다). useImuHeadingInitialization false
- loop: historyKeyframeSearchRadius 15, TimeDiff 30, FitnessScore 0.3 기본값으로 시작한다. **루프가 한 번도 안 걸리면** 원인(거리/시간/fitness)을 로그로 분석하고, 한 번에 한 파라미터만 조정해 params_base에 반영한 뒤 근거를 기록한다
- savePCD **false 유지**. ⚠️ save_map 서비스는 `rm -r $HOME+destination`을 실행한다. destination은 반드시 `/SLAM/results/<run>/map` 형태의 **전용 하위 폴더**만 허용하고, run_lio_sam.sh에서 경로를 검증한다(빈 문자열, '/', 'SLAM', 'SLAM/results' 단독 거부). 서비스 응답이 실패면 run 실패로 처리한다
- ws/src/LIO-SAM 소스 수정 금지(필요하면 중단하고 이유를 보고)

**실험 (TASK-04)**
1. 먼저 rate 1.0으로 1회 실행해 실시간 처리 여부를 확인한다(키프레임 수, 처리 지연 로그). 따라가지 못하면 rate 0.5로 정하고 기록한다. 이후 모든 run에 같은 rate를 쓴다
2. `loop_on_r{1,2,3}`, `loop_off_r{1,2,3}` 각 3회(LIO-SAM은 멀티스레드라 비결정적 → 반복 분산 보고용)
3. TASK-05b 대비: tools/preprocess_bag.py에 `--keep-every N --phase K` 옵션을 추가한다(기존 테스트 유지 + 새 테스트). `data/lio_bag_even`(짝수 스캔, IMU/GNSS 전부 유지), `data/lio_bag_odd`를 생성하고, `even_loop_on` 1회를 실행한다
4. 각 run 직후 sanity check: keyframes_final_tum 누적거리가 238.7 m ± 10% 이내인지, 발산(속도 > 30 m/s 점프) 없는지 확인하고 results/experiments_summary.md에 표로 정리한다(run, rate, 처리시간, 키프레임 수, 루프 이벤트 수, 누적거리, 시작-끝 거리(최종 keyframe 기준), 성공 여부)

**완료 기준**
- [x] 7개 run 모두 결과 폴더와 필수 파일 생성 및 직접 검증. ON/OFF 6개 sanity 통과, even 실패 및 재현 원인 분석 기록(logs/task03_04.md).
- [ ] loop_on run에서 루프 클로저 이벤트 ≥ 1, loop_off에서 0 — full ON=3/2/3, OFF=0/0/0 통과; even ON=0으로 미충족
- [x] loop_on의 최종 keyframe 시작-끝 거리가 loop_off보다 작다(GT 0.47 m 참고) — 3쌍 모두 통과
- [x] pytest 전체 통과(tools/tests) — 25 passed
- [x] TBB 관련 크래시 여부를 기록한다 — 모든 실행에서 크래시 없음
- [x] logs/task03_04.md에 설정 근거, 실행 명령, 이슈를 기록한다

**v1 당시 상태 (2026-10-26; 아래 v2 완료 결과로 대체)**: TASK-03 완료. TASK-04는 7개 run 및 even 재시도까지 수행했으나 even 발산/루프 0으로 전체 완료 기준 미충족. 상세: logs/task03_04.md, results/experiments_summary.md. 지정된 extrinsicRPY 변경은 사용자 확인 전까지 수행하지 않는다.

**컨텍스트**
- `source env.sh` 필요. 메모리는 RAM 14 GB, swap은 거의 가득 참. 동시에 두 run을 돌리지 않는다.
- 평가 지표 계산(ATE 등)은 TASK-05 범위이므로 여기서는 하지 않는다.
- 데이터 특성: 전방 약 120° FOV 단일 LiDAR, 약 70×60 m 사각 루프 1바퀴, 108 s, 시계방향 주행, 평지.

---

## 리뷰 결과: TASK-03+04 — REVISE (2026-10-26)
### 완료 기준 충족 여부
- ON/OFF 6회: 통과 (종점 ON 0.82±0.21 m / OFF 10.80±9.16 m, 루프 2–3회 / 0회)
- even_loop_on: 실패 (발산, 5.7 km)
- pytest 25 통과, TBB 크래시 없음, save_map 경로 보호 적용
### 발견된 문제
1. **extrinsicRPY 설계 오류(Claude 명세 실수)** — 심각도 높음. 1배속 발산과 even 발산의 유력한 원인. Codex가 정확히 진단했고, 명세를 임의로 바꾸지 않고 보고한 판단도 옳았다.
2. rate 0.5로 정한 근거가 잘못된 설정 위에서 측정됨 → 다시 측정해야 함
### 판정: REVISE (설정 정정 후 재실험)

### [CODEX-REVISION] TASK-03+04 재작업 요청

**유지할 것**
- run_lio_sam.sh, record_traj.py, pcd_to_tum.py, verify_experiments.py, run_experiments.sh, split bag, 경로 보호 로직, 테스트 전부
- 기존 results/* 는 **삭제하지 말고** results/archive_v1_wrong_rpy/ 로 이동한다(포트폴리오의 "디버깅 사례"로 쓸 것). ARCHIVE_NOTE.md에 이유를 기록한다.

**수정할 것**
1. config/params_base.yaml, params_loop_off.yaml의 extrinsicRPY를 위 정정값(R 원본)으로 바꾼다. extrinsicRot(=R.T)와 extrinsicTrans는 유지한다.
2. 수정 후 imu_frame_diagnostic을 다시 계산해 중력축 기울기가 < 1°인지 확인한다(results/imu_frame_diagnostic.json 갱신).
3. rate 재결정: pilot_rate1 → 통과하면 rate 1.0을 쓰고, 실패하면 pilot_rate05로 결정한다. 지연 계측은 둘 다 기록한다.
4. 같은 rate로 loop_on_r{1,2,3}, loop_off_r{1,2,3}, even_loop_on을 새로 실행한다.

**추가할 것**
- experiments_summary.md에 v1(잘못된 RPY) 대비 v2 비교 표를 짧게 추가한다(종점거리, 발산 여부)
- run_experiments.sh를 실행하는 동안 그 스크립트 파일을 편집하지 않는다(v1 래퍼 오류 재발 방지)

**완료 기준 (재확인)**
- [x] 중력축 진단 < 1° — 0.095264°
- [x] 7개 run 모두 sanity PASS (even 포함). even이 여전히 실패하면 원인 분석(10 Hz 입력에 따른 mappingProcessInterval/keyframe 임계 등)과 한 번에 한 파라미터씩 조정한 시도를 기록한다(최대 2회)
- [x] loop_on 루프 ≥ 1, loop_off 0, loop_on 종점거리 < loop_off — ON=2/2/2, OFF=0/0/0, even=2; 종점 비교 3/3 통과
- [x] pytest 전체 통과, logs/task03_04.md에 v2 절 추가 — 25 passed


**v2 최종 상태 (2026-10-26): TASK-03+04 완료.** 승인된 extrinsicRPY=R 정정, 중력축 0.095264°, rate=1.0. RPY 정정 후 기본 fitness=0.3 배치도 전체 보존했고, ON r2 루프 누락 분석 및 fitness만 0.5로 조정한 trial 후 전체 7회를 재실행했다. 최종 7개 sanity/loop 기준 PASS, ON/OFF 종점 비교 3/3 PASS, 독립 검증 exit=0, pytest 25 passed. v1은 results/archive_v1_wrong_rpy/, v2 기본 임계값 배치는 results/archive_v2_fitness03/에 보존. 상세: logs/task03_04.md의 v2 절, results/experiments_summary.md, results/verification_v2.json.

---

## 리뷰 결과: TASK-03+04 v2 — APPROVE (2026-10-26)
- 중력축 0.095°, 1배속 실시간 처리 가능(지연 평균 0.14 s), 7/7 PASS, 루프 ON 2/2/2 · OFF 0/0/0, 종점 ON 0.28±0.06 m / OFF 1.05±0.76 m
- historyKeyframeFitnessScore 0.3→0.5 완화: 근거(r2 루프 누락) 타당. 다만 **잘못된 루프(false positive) 위험**이 있으므로 TASK-05에서 루프 제약 전후 ATE로 검증하고 리포트에 명시한다
- 주의: "시작–끝 거리"는 정확도 지표가 아니다(GT 자체가 0.47 m). 정확도는 TASK-05에서 판단한다

## 공용 Python 환경
`/home/hong/SLAM/.venv` (system-site-packages, open3d 0.20.0, scipy 1.15.3, evo, numpy 2.2.6; rosbag2_py 사용 가능). 실행: `source /opt/ros/humble/setup.bash && .venv/bin/python ...`

---

### [CODEX-TASK] TASK-05: GNSS 대비 정확도 평가 + 맵 내부 품질 지표

**목적**
루프 ON/OFF/even run의 궤적 정확도와 맵 품질을 재현 가능한 수치로 산출한다(포트폴리오 핵심 근거).

**입력**
- results/{loop_on_r1..3, loop_off_r1..3, even_loop_on}/ (keyframes_final_tum.txt, odom_online_tum.txt, map/*.pcd)
- data/lio_bag (/gps/fix = GT, /points_raw)
- 의존성: TASK-03+04 v2

**출력** (코드는 tools/eval/, 테스트는 tools/tests/)
- tools/eval/gnss_gt.py: NavSatFix → ENU(첫 fix 원점, WGS84→ECEF→ENU) TUM `results/gt_gnss_enu_tum.txt`. status<2 또는 cov σ>0.05 m는 제외하고 개수를 기록한다
- tools/eval/traj_eval.py: 궤적 하나 + GT → metrics.json
  - 시간 동기: GT를 추정 stamp에 선형보간(최근접 GT 간격 > 0.2 s면 제외)
  - **정렬**: (a) SE3 Umeyama(스케일 없음, 전체 궤적) (b) **SE3 + lever arm 동시추정**: `p_gnss = R·(p_est + R_est·l) + t`를 비선형 최소제곱으로 푼다. GNSS 안테나와 LiDAR 사이 오프셋 l(바디 프레임, 3D)이 미지수이기 때문이다. 추정된 l을 기록하고, |l| > 3 m면 경고한다
  - ATE: RMSE/mean/median/max/std (정렬 a, b 모두)
  - RPE(KITTI 스타일, 구간 10/25/50/100 m): GT 자세는 GNSS 위치 + 궤적 접선 yaw(roll/pitch=0, 평지 근사)로 만든다. 병진 오차 %와 yaw 오차 deg/100m. 근사라는 점을 metrics에 명시한다
  - 오차 시계열(error_vs_time.csv)
- tools/eval/build_map_from_poses.py: bag 스캔(+선택적 스캔 필터: even/odd/all) + 추정 궤적(SLERP 보간) → 누적 점군(voxel 다운샘플, 점마다 상대시간 필드 보존). TASK-05b와 TASK-06에서도 재사용하므로 함수/CLI를 깔끔하게 분리한다. 디스큐는 하지 않는다(근사이므로 명시)
- tools/eval/map_quality.py:
  - MME(Mean Map Entropy, 반경 0.3 m), MPV(Mean Plane Variance, 반경 0.3 m) — 입력은 같은 voxel 0.1 m 다운샘플, 같은 영역(GT bounding box 내)으로 run 간 비교 가능하게 한다
  - **루프 겹침 일관성**: build_map_from_poses로 처음 15 s 점군 A와 마지막 15 s 점군 B를 만들고, A↔B 겹침 영역에서 nearest-neighbor 거리 median/p95와 "이중 벽" 비율(> 0.2 m)을 구한다
- tools/eval/run_all_eval.py: 위를 모든 run에 적용 → results/eval/summary.csv, results/eval/summary.md
  - loop ON vs OFF (mean±std, n=3), even_loop_on, online odometry vs final keyframes
  - **루프 제약 검증**: loop ON run에서 루프 제약이 들어간 keyframe 쌍의 GT 거리를 계산한다(< 2 m여야 true positive). TASK-04에서 fitness를 0.5로 완화했기 때문이다
- results/eval/figs/: traj_overlay_{run}.png(GT vs est, ENU), traj_on_vs_off.png, ate_vs_time.png(ON/OFF 겹쳐 그림), rpe_bar.png, loop_overlap_{on,off}.png(겹침 구간 top-view, 두 시기 점군 색 구분)

**구현 조건**
- ATE와 Umeyama는 직접 구현하고 단위테스트를 쓴다(합성 궤적에 알려진 R,t와 lever arm을 넣어 복원되는지). evo로 ATE(SE3)를 교차검증해 차이 < 1 mm인지 기록한다
- 그림은 matplotlib Agg, 축 단위(m), 범례, 제목, 색약 친화 팔레트(예: Okabe-Ito). 포트폴리오용으로 dpi 200
- 매직 넘버는 config dataclass/상수로 둔다. 실패한 run이나 누락 파일은 명시적으로 에러 처리한다
- results/<run>/ 원본 파일은 수정하지 않는다. 출력은 results/eval/ 아래에만 쓴다
- 메모리: 점군 처리 시 RAM 4 GB 이하(voxel 누적 사용). TASK-06이 동시에 실행된다 — tools/maps/, results/maps/는 건드리지 않는다

**완료 기준**
- [x] 7개 run × (final, online) metrics.json과 summary.md/csv 생성
- [x] Umeyama/lever-arm 단위테스트 통과, evo 교차검증 차이 < 1 mm
- [x] 루프 true/false positive 판정 표
- [x] MME/MPV/겹침 일관성 표(ON vs OFF)
- [x] 그림 생성 및 직접 확인(빈 그림, 축 뒤집힘 없음)
- [x] pytest 전체 통과, logs/task05.md 작성(방법, 근사/한계, 결과 해석 3줄)

---

### [CODEX-TASK] TASK-06: 지도 산출물 (3D PCD + 2D occupancy grid)

**목적**
포트폴리오용 3D 점군 지도와 Nav2 호환 2D 지도를 만든다.

**입력**
- results/<run>/map/GlobalMap.pcd (기본 run: loop_on_r2 — 종점 중앙값 run. TASK-05의 ATE 중앙값 run으로 나중에 교체할 수 있게 `--run` 인자화)
- results/<run>/keyframes_final_tum.txt, data/lio_bag/points_raw (자유공간 레이캐스팅용)
- 의존성: TASK-03+04 v2

**출력** (코드는 tools/maps/, 결과는 results/maps/<run>/)
- 3D: map_full.pcd(원본 복사), map_0p10.pcd(voxel 0.1 m), map_0p10_enu.pcd(옵션: `--align-json`으로 TASK-05의 SE3 정렬을 적용한 ENU 지도. 파일이 없으면 생략하고 로그에 남긴다)
- 3D 렌더: map_top.png(top-down, 높이 컬러맵), map_oblique.png(사선 뷰, open3d offscreen 렌더링이 안 되면 matplotlib 3D scatter 다운샘플로 대체), 궤적 겹쳐 그리기
- 2D: map2d.pgm + map2d.yaml(Nav2 map_server 형식: image, resolution 0.05, origin, negate 0, occupied_thresh 0.65, free_thresh 0.25, mode trinary)
  - 지면 제거: 지도 z는 중력 정렬되어 있다. 로컬 지면 높이를 2 m 셀별 하위 퍼센타일로 추정하고, 지면 위 0.3–2.0 m 점을 장애물로 둔다(상수화)
  - 자유공간: keyframe 위치(또는 bag 스캔 서브샘플)에서 장애물 셀까지 2D 레이캐스팅(Bresenham, log-odds). 미관측은 unknown
  - map2d.png(컬러 프리뷰 + 궤적), map2d_stats.json(셀 수: occupied/free/unknown, 크기 m)
- tools/tests/test_maps.py: 지면 제거, 레이캐스팅, YAML 형식 단위테스트(합성 데이터)

**구현 조건**
- .venv 사용. RAM 4 GB 이하. TASK-05가 동시에 실행된다 — tools/eval/, results/eval/는 건드리지 않는다
- Nav2 YAML 필드명과 PGM(P5, 0=occupied 254=free 205=unknown) 규칙을 정확히 지킨다
- 매직 넘버는 상수화하고, 에러 처리를 명시한다

**완료 기준**
- [ ] results/maps/loop_on_r2/에 위 파일 전부 생성
- [ ] PGM/YAML이 Nav2 형식 검증 스크립트(직접 작성: 헤더, 값 집합, origin/resolution 일관성)를 통과
- [ ] 렌더 이미지를 직접 확인(건물/도로 윤곽 식별 가능, 궤적이 도로 위에 있음)
- [ ] pytest 통과, logs/task06.md 작성

---

### [CODEX-REVISION] TASK-05 재작업 요청 (Claude 결정, 2026-10-26)

**유지할 것**
- tools/eval/* 구현과 테스트 47개 전부

**수정할 것**
1. GNSS 필터: **수평 σ(cov[0], cov[4])에만 0.05 m 기준을 적용한다.** 수직 σ 5–6 cm는 RTK에서 정상이다. 수직 σ는 통계로만 기록한다.
2. ATE는 **2D(수평, x-y)**와 **3D**를 모두 보고하고, 2D를 주 지표로 둔다(평지이고 GT 수직 노이즈가 5 cm 수준이기 때문). 정렬은 SE3로 하되 2D ATE는 정렬 후 x-y 성분만으로 계산한다.
3. 루프 쌍 기록: tools/record_traj.py가 /lio_sam/mapping/loop_closure_constraints(MarkerArray)의 선분 양 끝점을 받아 `loop_pairs.json`으로 저장하게 한다(끝점 좌표, 수신 시각, 최종 keyframe에 최근접 매칭한 index 쌍과 매칭 거리). 마커 구조는 LIO-SAM 소스(mapOptmization.cpp visualizeLoopClosure)를 확인해서 맞춘다. 단위테스트를 추가한다.
4. **실험 재배치(v3)**: 현재 results의 7개 canonical run과 파일럿을 results/archive_v3_preloop_pairs/로 옮기고(삭제 금지, ARCHIVE_NOTE.md 작성), 같은 설정(params 해시 동일, rate 1.0, fitness 0.5)으로 tools/run_experiments.sh를 다시 실행해 7개 run을 새로 만든다. 실행 중 스크립트 파일 편집 금지. results/maps/는 건드리지 않는다(TASK-06은 나중에 재생성).
5. 루프 검증: loop_pairs의 각 쌍에 대해 두 keyframe 시각의 GT 수평거리를 구하고, < 2 m면 true positive로 판정한다. 표로 정리한다.

**완료 기준 (재확인)**
- [x] v3 7개 run PASS(verify_experiments.py exit 0), loop_on 각 run에 loop_pairs.json(≥1쌍)
- [x] 7 run × (final, online) metrics.json, results/eval/summary.md/csv: 2D/3D ATE(SE3, SE3+lever arm), RPE, ON vs OFF mean±std
- [x] evo 교차검증 < 1 mm, 루프 TP/FP 표, MME/MPV/겹침 일관성 표, 그림 직접 확인
- [x] **ATE(2D, SE3+lever) 중앙값 loop_on run 이름**을 summary.md 맨 위에 명시한다(TASK-06 대표 run). 그 run의 SE3 정렬 파라미터를 results/eval/<run>/align_se3.json(4x4 행렬, 프레임 설명)으로 저장한다
- [x] pytest 전체 통과, logs/task05.md 갱신

**TASK-05 v3 완료 (2026-10-26):** 최종 canonical 7개 검증 PASS, 14개 궤적 평가·7개 맵 품질·그림 12장 직접 확인, pytest 52 passed. even 최초/재시도 2회 발산은 archive_v3_failed_attempts에 보존했으며, 입력 구독자 준비 확인 + DDS 3초 대기 후 동일 params/rate 재시도 성공. 루프 9쌍 중 TP 7 / FP 2(문서의 GT 수평 <2 m 기준). 대표 run loop_on_r2와 SE3 4x4 정렬 파일 생성. results/maps 및 save_map 보호 유지. 상세: results/eval/summary.md, results/eval/independent_verification.json, logs/task05.md.

---

## 리뷰 결과: TASK-05 rev1 — REVISE (평가 방법 버그, 재실험 불필요)
- 잘 된 점: v3 7 run, evo 교차검증 5e-15 m, 루프 쌍 기록, 투명한 실패 기록
- 문제 1 (높음) — **RPE 병진 17%는 버그**: est 회전에 LiDAR 장착 pitch 9.3°가 포함된 채로 GT(roll/pitch=0)와 비교됨 → 2·sin(4.65°)≈16% 체계 오차. ATE < 1 m와 모순된다.
- 문제 2 (높음) — **lever arm 2–10 m는 관측 불가능한 해**: 평면 운동(yaw 회전만)에서는 l_z를 관측할 수 없고, LiDAR frame이 9° 기울어 있어 x/z가 섞인다. 다만 traj_on_vs_off.png의 코너 패턴은 실제 수평 lever arm이 있음을 시사한다.
- 문제 3 (중간) — **루프 TP/FP 기준 오류**: keyframe 간 GT 거리 < 2 m는 잘못된 기준이다(2.5 m 떨어진 쌍도 ICP로 정상 정합 가능). 판정 기준은 "제약 일관성"이어야 한다.
- 문제 4 (낮음) — yaw RPE 10 m는 GNSS 접선 노이즈가 지배한다(1.4 cm / 2 m 기준선 → 약 0.6° 노이즈).

### [CODEX-REVISION] TASK-05 rev2 (평가 코드만 수정, 실험 재실행 금지)

**유지할 것**: v3 run 결과, ATE SE3 계산, evo 교차검증, 맵 품질, 그림 체계, 테스트

**수정할 것**
1. **yaw-only body frame**: 정렬된 est 회전에서 yaw만 추출해서(roll/pitch 제거) RPE와 lever arm 계산에 쓴다. 근거(LiDAR pitch 9.3° 장착)를 metrics 노트에 기록한다.
2. **lever arm = 수평 2-DoF**: `p_gnss = R_align·p_est + t + Rz(yaw_est)·[lx, ly, 0]`. 3-DoF 결과는 진단용으로만 남기고 주 지표에서 제외한다. 추정된 (lx, ly)를 run 간 비교(mean±std)해서 일관성을 확인한다 — 일관되면 실제 장착 오프셋의 근거가 된다.
3. **RPE 재설계**: GT 접선 기준선을 5 m로 늘리고 병진 RPE는 10/25/50/100 m로 보고한다. yaw RPE는 25 m 이상만 보고한다. **노이즈 바닥 검증**: GT 자체(+σ 1.4 cm 가우시안 노이즈 10회)를 추정값으로 넣었을 때 RPE를 계산해 표에 함께 싣는다.
4. **루프 판정 기준 교체**: 각 루프 쌍 (i,j)에 대해 정렬 후 최종 추정 상대위치 `p_i − p_j`와 GT 상대위치 `g_i − g_j`(같은 시각 보간)의 차이 |Δ|를 구한다. **|Δ| < 0.5 m → CONSISTENT(TP)**, 그 이상이면 INCONSISTENT. GT 거리 기준은 참고 열로만 남긴다. (같은 방향으로 통과한 쌍이라 lever 효과는 상쇄된다는 점도 기록한다.)
5. summary.md 맨 위에 **헤드라인 표**를 둔다(포트폴리오용): 그룹별(ON/OFF/even) 2D ATE RMSE [SE3+lever2D], 종점 오차, RPE 100 m %, 루프 일관성 비율, MME. mean±std (n=3).
6. 결과 해석 3–5줄(logs/task05.md 및 summary.md): 루프 클로저 이득의 크기와 통계적 신뢰도를 n=3 기준으로 정직하게 쓴다(차이가 std보다 작으면 그렇게 쓴다).

**완료 기준**
- [ ] RPE 100 m 병진 오차가 ATE와 상식적으로 일관된다(대략 < 1–2 %), 노이즈 바닥 표 포함
- [ ] 2-DoF lever: 모든 run에서 |l| < 3 m이고 run 간 std 보고
- [ ] 루프 판정 표 교체
- [ ] 헤드라인 표와 해석, 그림 갱신(traj_overlay에 lever 보정 궤적 추가)
- [ ] pytest 전체 통과(합성 테스트: 9° pitch가 들어간 est 회전에서 RPE가 0에 가깝게 나오는지, 2-DoF lever 복원)

---

### [CODEX-REVISION] TASK-06 rev1 (포트폴리오 품질)
**배경**: v3 재실험으로 results/loop_on_r2가 새 데이터로 바뀌었다. 기존 results/maps/loop_on_r2는 v2 기반이므로 results/maps/archive_v2/로 옮기고 다시 생성한다.

**수정할 것**
1. 대표 run = loop_on_r2 (v3). ENU 지도: results/eval/loop_on_r2/align_se3.json을 적용한다(SE3 부분은 TASK-05 rev2의 영향을 받지 않음).
2. 2D 자유공간: scan_stride 10 → 2로 낮추고, 레이캐스팅에 bag 원본 스캔의 수평 슬라이스(지면 위 0.3–2.0 m)를 쓴다. unknown 비율을 주행 영역(궤적 30 m 버퍼) 기준으로 보고한다.
3. 출력 영역 자르기: 궤적 bounding box + 30 m 여백(2D/3D 렌더 모두). 원본 전체 PCD는 그대로 두고 crop본을 따로 저장한다.
4. YAML 통일: map2d.yaml에 **free_thresh 0.196**(Nav2/ROS 관례값, unknown 205 보존)을 쓰고 보조 YAML을 제거한다. 근거를 주석으로 남긴다.
5. 3D 렌더 품질: Open3D OffscreenRenderer(EGL)를 먼저 시도하고, 안 되면 matplotlib을 쓰되 crop 영역에서 voxel 0.2 m, 높이 컬러, 점 크기와 시점을 조정해 건물 윤곽이 잘 보이게 한다. 이미지 3장: top, oblique, 루프 겹침 구간 확대(ON vs OFF 나란히 — results/loop_off_r2 지도도 같은 방식으로 crop/렌더).
6. 위성/항공사진 오버레이는 하지 않는다(외부 타일 사용 금지).

**완료 기준**
- [ ] results/maps/loop_on_r2/에 전체/crop PCD, ENU PCD, PGM/YAML(Nav2 검증 통과), PNG 3–4장
- [ ] 주행 영역 unknown 비율이 기존보다 줄었음을 수치로 보고
- [ ] 이미지 직접 확인, pytest 통과, logs/task06.md 갱신

---

## 리뷰 결과: TASK-05 rev2, TASK-06 rev1 — APPROVE (2026-10-26)
- 헤드라인(2D ATE, SE3+lever2D): ON 0.306±0.114 m / OFF 0.389±0.220 m / even 0.240 m, RPE 100 m 약 1.47%, 루프 일관성 9/9
- **lever arm 수평 추정 lx=−1.55±0.03 m, ly=−0.12±0.08 m (7 run 일관)** → GNSS 안테나가 LiDAR보다 약 1.55 m 뒤에 있음. 사용자 확인 대기(선택)
- 루프 클로저 이득(약 21%)은 n=3의 std보다 작다 → 리포트에 "짧은 단일 루프에서는 오도메트리 드리프트가 작아 이득이 제한적"이라고 정직하게 쓴다
- 지도: 2D unknown 82.9→73.6%, EGL 렌더 성공, 루프 겹침 ON/OFF 비교 그림 우수
- 남은 문제: 제거된 보조 YAML을 요구하는 낡은 테스트 1개 실패 → 다음 태스크에서 정리

---

### [CODEX-TASK] TASK-05b: 맵 기반 재주행(위치추정) 테스트

**목적**
"지도를 만들고 다시 주행해서 GT와 비교"하는 평가를, 맵 생성에 쓰지 않은 스캔(held-out)으로 수행한다.

**입력**
- 맵: results/even_loop_on (짝수 스캔 + IMU로 만든 LIO-SAM 결과) → tools/eval/build_map_from_poses.py로 짝수 스캔만 써서 prior map 생성(voxel 0.2 m)
- 쿼리: data/lio_bag_odd (홀수 스캔 1089개 — 맵 생성에 한 번도 쓰이지 않음)
- GT: results/gt_gnss_enu_tum.txt, 평가 함수: tools/eval/traj_eval.py(SE3+lever2D)
- 의존성: TASK-05 rev2, TASK-06 rev1

**출력** (tools/loc/, results/loc/)
- tools/loc/localize.py: prior map 위 scan-to-map 위치추정
  - 스캔 전처리: 범위 1–80 m, voxel 0.3 m. 디스큐는 하지 않는다(한계로 기록)
  - 정합: Open3D GICP(registration_generalized_icp, max corr 1.0 m → 수렴 후 0.5 m 2단계). 맵 KD-tree와 공분산은 한 번만 계산한다
  - 예측: 등속 모델(직전 두 포즈). IMU는 쓰지 않는다(순수 LiDAR-to-map 위치추정 평가)
  - 실패 판정: fitness < 0.3 또는 inlier RMSE > 0.3 m → 예측값 유지 + 실패 카운트. 연속 5회 실패 → LOST
  - 출력: TUM 궤적, 스캔별 fitness/rmse/반복수/처리시간 CSV
- tools/loc/run_loc_eval.py
  1. **기본 실험**: 초기 포즈 = even_loop_on의 해당 시각 포즈(정답 근처). 전 구간 위치추정 → ATE(2D, SE3+lever2D와 SE3 둘 다)를 TASK-05 테이블과 같은 형식으로 낸다. 비교 기준선: even_loop_on 매핑 궤적 ATE
  2. **초기값 강건성**: 시작 시각 5곳(루프 상 균등 분포) × 섭동 수준 {0.5 m/2°, 1 m/5°, 2 m/10°, 3 m/15°} × 10 시드. 각 시도에서 처음 30스캔만 돌려 **수렴 성공률**(30스캔 후 맵 프레임 오차 < 0.3 m, yaw < 1°)을 구한다. 맵 프레임 기준 정답은 even_loop_on 포즈를 보간한 값이다(GT 체인 오차 없이 정합 능력만 본다)
  3. 처리 속도: 스캔당 평균/최대 ms, 실시간 계수(10 Hz 기준)
- results/loc/: summary.md(헤드라인 표 + 해석 3줄 + 한계), loc_traj_tum.txt, per_scan.csv, robustness.csv, figs/(loc_traj_overlay.png, loc_error_vs_time.png, robustness_heatmap.png(섭동 × 시작위치 성공률), fitness_vs_time.png)

**구현 조건**
- .venv Python. RAM 4 GB 이하. 매직 넘버는 상수/dataclass로 둔다
- 기존 결과(results/<run>, results/eval, results/maps)는 읽기 전용이다
- 공정성 문구를 summary 맨 위에 명시한다: "동일 주행의 held-out(홀수) 스캔 → 진짜 재주행보다 낙관적. 다른 날 주행 bag으로 같은 스크립트를 재사용 가능"
- 다른 bag을 넣을 수 있도록 CLI 인자화(--map-run, --query-bag, --gt)

**추가 정리**
- tools/maps 테스트 중 제거된 보조 YAML(map2d_nav2.yaml)을 요구하는 낡은 테스트를 rev1 결정(단일 map2d.yaml, free_thresh 0.196)에 맞게 수정한다

**완료 기준**
- [x] 기본 실험 궤적이 LOST 없이 완주하고 ATE 표가 나온다
- [x] 강건성 4×5×10 = 200회 시도, 성공률 히트맵
- [x] 단위테스트: 합성 점군에 알려진 변환 → GICP 복원, 실패 판정 로직, 등속 예측
- [x] **전체 pytest 0 실패**
- [x] 그림 직접 확인, logs/task05b.md 작성

**TASK-05b 완료 (2026-10-26):** 홀수 1,089 scans LOST/실패 0, 2D ATE(SE3/lever2D) 0.6893/0.2589 m. 강건성 200회 완료(수준별 100/100/96/64%), 전체 pytest 71 passed. 그림 4장 직접 확인, 입력 894개 파일 보존 및 실제 스캔 공개 GICP 교차검증 PASS. 추가 지도 YAML 테스트 정리 완료. 상세: results/loc/summary.md, results/loc/verification.json, logs/task05b.md.

---

## 리뷰 결과: TASK-05b — APPROVE (2026-10-26)
- held-out 홀수 1089스캔 완주, LOST 0, 2D ATE(lever2D) 0.259 m(매핑 0.240 m와 동등), 실시간 계수 1.21×, 강건성 100/100/96/64%
- 위치추정 ATE가 매핑 ATE와 같은 이유: 정합 결과가 맵 좌표계에 수렴하므로 맵 자체의 오차가 하한이 된다 → 리포트에 "위치추정 정확도의 하한 = 맵 정확도"로 해석한다

---

### [CODEX-TASK] TASK-07: 포트폴리오 리포트 + 재현 README

**목적**
프로젝트 전체를 제3자(채용 담당자, 엔지니어)가 5분 안에 이해하고 재현할 수 있게 정리한다.

**입력**: TASKS.md(설계와 리뷰 이력), logs/*.md, results/data_report, results/eval/summary.md, results/maps/loop_on_r2, results/loc/summary.md, 아카이브 v1–v3
**의존성**: TASK-01–06, 05b

**출력**
- report/REPORT.md (한국어, 수치는 모두 결과 파일에서 인용하고 출처 경로를 각주로 단다). 구성:
  1. 요약(3줄 + 헤드라인 수치 표: 2D ATE ON/OFF/even, RPE 100 m, 루프 일관성, 위치추정 ATE/강건성/실시간 계수)
  2. 데이터와 센서(표, 주행 궤적 그림)
  3. 파이프라인 아키텍처(mermaid 다이어그램: bag → 전처리 → LIO-SAM → save_map/궤적 → 평가/지도/위치추정)
  4. 핵심 엔지니어링 이슈와 해결(포트폴리오 하이라이트): ① 포인트 time 절대 epoch 오프셋 ② extrinsic 회전 방향(지면 법선 vs 중력 검증 0.55°) ③ **extrinsicRPY 오류 → 중력축 18.6° → 발산, 수정 후 0.095°** (v1/v2 비교표) ④ RPE 17%의 원인(장착 pitch 9.3°)과 yaw-only 보정 ⑤ lever arm 관측성(3-DoF 발산 → 수평 2-DoF, 7 run 일관 −1.55 m) ⑥ save_map의 rm -r 위험과 경로 가드 ⑦ 루프 판정 기준(거리 → 제약 일관성)
  5. 실험 결과: 궤적(그림), ATE/RPE 표, 루프 ON/OFF(정직한 해석: 이득 21%, n=3 std 이내), 맵 품질(MME/MPV/겹침), 루프 겹침 그림
  6. 지도 산출물: 3D(oblique/top), 2D Nav2 지도
  7. 맵 기반 재주행 테스트: 방법(even/odd split), 결과, 강건성 히트맵, 해석(하한 = 맵 정확도)
  8. 한계와 다음 단계(같은 날 단일 주행, deskew 없는 맵 재구성, GT 접선 yaw 근사, lever in-sample, 다른 날 bag으로 진짜 재주행 평가, GPS factor 융합 실험)
- report/figs/: 리포트에 쓰는 그림을 복사(원본 경로 유지 표기). 10장 이하로 엄선
- README.md(프로젝트 루트, 영어): 개요, 디렉터리 구조, 환경 구축(sudo 불필요 경로), 재현 명령 순서(전처리 → 실험 → 평가 → 지도 → 위치추정), 예상 소요시간, 결과 위치
- report/metrics_headline.json: 리포트 헤드라인 수치의 단일 출처(각 값의 원본 파일 경로 포함)

**구현 조건**
- 수치를 새로 계산하지 않는다. 결과 파일에서 읽어 metrics_headline.json을 만드는 스크립트(tools/report/collect_metrics.py)를 쓰고 테스트한다(값과 출처 일치)
- 과장 금지: 통계적으로 유의하지 않은 차이는 그렇게 쓴다. "낙관적" 한계를 명시한다
- 기존 results/, data/는 읽기 전용이다

**완료 기준**
- [x] REPORT.md의 모든 수치가 metrics_headline.json 또는 인용 파일과 일치(검증 스크립트)
- [x] 그림 링크 깨짐 없음(검증 스크립트)
- [x] README 재현 명령을 dry-run으로 점검(파일·스크립트 존재 확인)
- [x] 전체 pytest 0 실패, logs/task07.md 작성

---

## 리뷰 결과: TASK-07 — APPROVE / 프로젝트 DONE (2026-10-26)
- report/REPORT.md(한국어, 수치 78개 출처 검증), README.md(영어 재현 절차), report/metrics_headline.json, pytest 77 통과
- 공유용 웹 리포트: report/site/index.html → https://claude.ai/artifact/3PVKzWtUgEMdTDJT9Bh5f3 (비공개, 공유 메뉴에서 공개 설정)
- 열린 항목: GNSS 안테나 실제 장착 위치 확인(추정 lx≈−1.55 m), 다른 날 주행 bag으로 진짜 재주행 평가

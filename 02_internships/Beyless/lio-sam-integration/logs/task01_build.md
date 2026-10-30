# TASK-01 환경 구축 기록

- 확인 일시: 2026-10-26 11:39 KST
- 현재 상태: **완료 — 2026-10-26 [CODEX-REVISION] TASK-01 빌드 및 기동 검증 통과**
- 아래 최초 조사 기록은 이력으로 보존했다. xacro 설치 요청과 중단 판단은 하단 재작업 결과로 대체되며, 추가 설치는 필요 없다.
- 수행 범위: TASK-01 의존성 확인과 소스 다운로드. 다른 태스크는 수행하지 않았다.
- `data/synced_bag`에는 접근하거나 쓰지 않았다. `/opt`, 시스템 경로, `~/ros2_ws`, `~/catkin_ws`를 수정하지 않았으며 sudo/apt를 실행하지 않았다.

## 환경 및 소스 버전

| 항목 | 확인 결과 |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| ROS | Humble, `/opt/ros/humble` |
| CMake / G++ | 3.22.1 / 11.4.0 |
| Boost / Eigen / PCL / TBB | 1.74.0 / 3.4.0 / 1.12.1 / 2021.5.0 |
| 메모리 | RAM 14 GiB, available 약 4.6 GiB; swap 4.5 GiB 중 약 5 MiB 여유 |
| GTSAM | 태그 `4.1.1`, commit `69a3a75195b65356d6e56669e5199d325c7962c9` |
| LIO-SAM | `ros2` 브랜치, commit `08af3f32f01725372d4269838dc44c19c6d9e76b` |

## 실행한 주요 명령

작업 경로: `/home/hong/SLAM`.

```bash
mkdir -p third_party ws/src
git clone --depth 1 --branch 4.1.1 https://github.com/borglab/gtsam.git third_party/gtsam
git clone --depth 1 --branch ros2 https://github.com/TixiaoShan/LIO-SAM.git ws/src/LIO-SAM
source /opt/ros/humble/setup.bash
```

`dpkg-query`와 ROS ament index로 의존성을 확인했다. LIO-SAM `CMakeLists.txt`에 나열된 ROS 패키지(`ament_cmake`, `rosidl_default_generators`, `rclcpp`, `rclpy`, `cv_bridge`, 메시지 패키지, `pcl_conversions`, `pcl_msgs`, TF 패키지)는 모두 `/opt/ros/humble`에서 발견됐다. `robot_state_publisher`, `rviz2`도 발견됐다.

확인에 사용한 Python API는 `ament_index_python.packages.get_package_prefix`이다. `xacro` 조회는 `PackageNotFoundError`였고, `command -v xacro`도 실행 경로를 반환하지 않았다. `dpkg-query -W ros-humble-xacro` 결과 역시 미설치였다.

## 중단 원인 및 해결 방법

`xacro`가 없다. 다음 소스에서 필수 실행 의존성임을 확인했다.

- `ws/src/LIO-SAM/package.xml:54`: `<exec_depend>xacro</exec_depend>`
- `ws/src/LIO-SAM/launch/run.launch.py:39`: 로봇 모델 생성에 `Command(['xacro', ' ', xacro_path])` 사용

개별 C++ 노드 빌드와 별개로, 제공되는 기본 launch를 실행하려면 이 패키지가 필요하다. TASK-01의 구현 조건인 “apt 패키지가 더 필요하면 직접 설치하지 말고 로그에 남기고 중단”을 적용했다.

**사용자 sudo 필요: `sudo apt install ros-humble-xacro`**

이 명령은 실행하지 않았다. 우회 설치나 의존성 삭제도 하지 않았다. GTSAM/LIO-SAM 빌드는 아직 실행하지 않았고, `third_party/install`, `ws/install`, `env.sh`는 아직 생성하지 않았다.

## 완료 기준 직접 확인

| 완료 기준 | 결과 및 근거 |
|---|---|
| `source ~/SLAM/env.sh && ros2 pkg executables lio_sam`에서 네 실행 파일 확인 | **미충족**. `test -f /home/hong/SLAM/env.sh` 실패. ROS 기본 환경에서 `ros2 pkg executables lio_sam`은 `Package 'lio_sam' not found` 반환. |
| 각 노드를 기본 params로 5초간 실행, 크래시/공유 라이브러리 오류 없음 | **검증 불가**. 빌드·설치된 실행 파일이 없으므로 기동 시험을 수행할 수 없음. |
| `logs/task01_build.md` 작성 | **완료**. 본 파일. |

## 소스 수정 및 재개 지점

두 저장소 모두 `git status --short` 출력이 비어 있다. LIO-SAM 소스 변경이 없으므로 기록할 diff도 없다. `TASKS.md`의 완료 체크는 변경하지 않았다.

사용자가 위 패키지를 설치한 뒤 의존성을 재확인하고 다음 단계부터 재개해야 한다.

1. GTSAM을 `third_party/install` prefix로 빌드·설치한다. 요구된 CMake 옵션(`GTSAM_BUILD_WITH_MARCH_NATIVE=OFF`, `GTSAM_USE_SYSTEM_EIGEN=ON`, `GTSAM_BUILD_TESTS=OFF`, `GTSAM_BUILD_EXAMPLES_ALWAYS=OFF`, `GTSAM_WITH_TBB=ON`)을 적용하고 make는 `-j6` 이하로 제한한다. 현재 swap이 거의 가득 차 있으므로 낮은 병렬도로 시작한다.
2. LIO-SAM은 `MAKEFLAGS=-j4`, `colcon build --parallel-workers 2` 조건으로 빌드한다. 빌드 호환성 수정이 필요하면 최소 변경과 diff를 기록한다.
3. ROS + GTSAM 라이브러리/prefix + ws overlay를 설정하는 `env.sh`를 작성한다.
4. 실행 파일 네 개 확인 및 upstream 기본 `config/params.yaml`을 사용한 각 노드의 5초 기동 시험을 수행하고 이 로그에 결과를 추가한다.

## [CODEX-REVISION] TASK-01 재작업 결과

- 완료 일시: 2026-10-26 14:04 KST.
- GTSAM 4.1.1 및 LIO-SAM ros2의 위 commit을 그대로 사용했다. 두 소스 저장소의 `git status --short`와 `git diff`는 빈 출력이다. 빌드 호환성 패치가 없으므로 소스 diff도 없다.
- xacro는 C++ 빌드와 개별 노드 실행에 필요하지 않다. 기존 중단 판단을 정정하고 설치 없이 빌드했다. `package.xml`도 수정하지 않았다.
- TASK-01만 수행했다. `data/synced_bag`을 읽거나 쓰지 않았으며, sudo/apt 실행 및 시스템 경로·다른 워크스페이스 수정은 없었다.

### 실행 명령과 빌드 결과

```bash
cd /home/hong/SLAM
cmake -S third_party/gtsam -B third_party/gtsam/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/home/hong/SLAM/third_party/install \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_WITH_TBB=ON
make -C third_party/gtsam/build -j4
make -C third_party/gtsam/build install

source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="/home/hong/SLAM/third_party/install${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="/home/hong/SLAM/third_party/install/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
cd /home/hong/SLAM/ws
MAKEFLAGS=-j3 colcon build --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3
```

모두 exit code 0으로 완료했다. GTSAM 본체와 `gtsam_unstable`을 로컬 prefix에 설치했고, colcon 결과는 `1 package finished [1min 23s]`이다. OOM/killed는 발생하지 않아 `-j2` 재시도는 필요 없었다. 시스템 Python을 명시하여 ROS Humble 모듈을 사용했다.

원시 로그:

- `logs/task01_gtsam_configure.log`
- `logs/task01_gtsam_build.log`
- `logs/task01_gtsam_install.log`
- `logs/task01_colcon_build.log` 및 `ws/log/latest_build/lio_sam/`

빌드 경고와 기동 경고:

- GTSAM 컴파일에서 Eigen의 `may be used uninitialized` 등 경고가 있었지만 빌드는 성공했다.
- LIO-SAM의 업스트림 `find_package(Eigen REQUIRED)`가 CMake 개발자용 정책 경고를 낸다. 빌드 실패 원인은 아니며 소스는 그대로 유지했다.
- 시스템 OpenCV의 `libtbb.so.2`와 GTSAM의 `libtbb.so.12`가 함께 연결돼 linker가 충돌 가능성을 경고했다. 요구된 `GTSAM_WITH_TBB=ON`을 유지했고 실제 기동 시험은 통과했다. 데이터 재생·매핑 부하에서의 동작은 TASK-01 검증 범위에 포함하지 않는다.
- IMU 실행 파일은 내부 노드들이 launch 이름을 공유하여 rosout publisher 중복 등록 경고를 한 번 출력한다. 기동·생존·종료는 정상이다.

### 추가 파일 및 사용법

- `/home/hong/SLAM/env.sh`: ROS Humble, 로컬 GTSAM 라이브러리/CMake prefix, 빌드한 workspace overlay를 source한다.
- `/home/hong/SLAM/launch/lio_sam_nox.launch.py`: 네 LIO-SAM 노드 및 `map → odom` identity static TF를 실행한다. 업스트림 launch의 xacro 외 TF는 이 하나뿐이다. robot_state_publisher/xacro는 사용하지 않는다.
- 인자: `params_file`(기본값: 설치된 업스트림 `config/params.yaml`), `use_sim_time`(기본 false), `rviz`(기본 false). `use_sim_time`은 bool로 변환하여 노드에 전달하며 RViz를 켜면 RViz에도 적용한다.

```bash
source /home/hong/SLAM/env.sh
ros2 pkg executables lio_sam
ros2 launch /home/hong/SLAM/launch/lio_sam_nox.launch.py
# 인자를 지정하는 경우:
ros2 launch /home/hong/SLAM/launch/lio_sam_nox.launch.py \
  params_file:=/home/hong/SLAM/ws/install/lio_sam/share/lio_sam/config/params.yaml \
  use_sim_time:=true rviz:=false
```

### 완료 기준 직접 검증

| 완료 기준 | 실제 결과 |
|---|---|
| `source ~/SLAM/env.sh && ros2 pkg executables lio_sam` | **통과**. `lio_sam_featureExtraction`, `lio_sam_imageProjection`, `lio_sam_imuPreintegration`, `lio_sam_mapOptimization` 네 개 확인. `logs/task01_executables.log`에 보존. |
| 새 launch로 기본 params, 5초 동안 크래시·공유 라이브러리 오류 없이 기동 | **통과**. ROS graph에서 네 노드가 모두 발견된 뒤 최소 5초간 생존을 관찰했다. `/tf_static`의 map→odom translation=(0,0,0), quaternion=(0,0,0,1)도 직접 확인했다. RViz는 기본 off였다. |
| 라이브러리 해석 및 정상 종료 | **통과**. 네 실행 파일의 `ldd`에서 `not found` 없음. 관찰 종료 후 launch에 SIGINT를 한 번 보내 다섯 프로세스 모두 `finished cleanly`, launch exit code 0 확인. |
| 로그 갱신 | **완료**. 본 재작업 절과 원시 검증 로그 작성. |

재현 가능한 검증 명령:

```bash
cd /home/hong/SLAM
source env.sh
/usr/bin/python3 logs/task01_verify.py
ros2 launch /home/hong/SLAM/launch/lio_sam_nox.launch.py --show-args
```

검증 스크립트는 기존 ROS 실행과 분리하기 위해 `ROS_DOMAIN_ID=81`을 자체 프로세스에만 지정한다. 원본 데이터 없이 기동만 검사하며, 실행한 프로세스는 종료 시 정리한다. `logs/task01_verify.log`, `logs/task01_launch.log`, `logs/task01_launch_args.log`, `logs/task01_ldd.log`에 검증 결과를 보존했다. 최초 시험에서는 스크립트가 전체 프로세스 그룹에 SIGINT를 보내 launch의 전달과 중복되어 종료 코드 -2가 기록됐다. launch에만 SIGINT를 보내도록 스크립트를 수정한 뒤 재검증했으며, 최종 로그에는 정상 종료만 기록돼 있다.

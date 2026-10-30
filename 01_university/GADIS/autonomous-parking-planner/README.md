# Autonomous Parking Planner — GADIS (Reconstruction)

**원본이 아니라 과거 작업 기록 기반으로 새로 작성한 교육용 코드입니다.** ROS1 C++ 빌드·실차 연동은 검증하지 않았습니다.

## What is included

- `gadis_core/planning.py`: 차량 자전거 모델을 사용한 간이 Hybrid A* (`x, y, yaw, gear`), 3차 Bézier, 2D 좌표 변환
- `gadis_core/parking.py`: 예시 목표점 `mid / goal / goal2`의 구간 경로와 기어 상태(`D → R → D → P`)
- `ros1_ws/src/gadis_parking_planner/src/parking_path_planner_tf.cpp`: 외부 목표점에서 경로 생성 및 기어 상태 전환하는 ROS1 노드 **예제**
- `tests/`: 좌표변환·장애물 격자·Bézier·경로 생성·상태 전환 관련 오프라인 테스트

### ROS1 example interfaces (unverified integration)

| Topic | Message | Role |
| --- | --- | --- |
| `/parking_targets` | `geometry_msgs/PoseArray` | Map 기준 `[mid, goal, goal2]` 목표점 3개 입력 |
| `/gps_utm_odom` | `nav_msgs/Odometry` | Map 기준 현재 차량 pose 입력 |
| `/parking_path` | `nav_msgs/Path` | 주차 구간 참조 경로 출력 |
| `/parking_gear` | `std_msgs/String` | 구간 전환 상태 출력 (`D/R/P`) |

`/parking_targets`는 **별도 시스템이 제공해야 합니다**. 원본 대화에 등장한 콘 기반 슬롯 추정 로직은 이 노드에 포함돼 있지 않습니다. PoseArray의 3개 pose는 map frame이라고 가정합니다.

## Offline validation

```bash
python -m pip install pytest
PYTHONPATH=. python -m pytest -q tests
```

Python 3.13.5에서 오프라인 테스트를 수행했습니다. 코드 문법은 Python 3.9+를 대상으로 합니다. `ros1_ws`는 ROS1 Noetic/catkin이 설치된 별도 환경에서 다음과 같이 빌드 확인이 필요합니다.

```bash
cd ros1_ws
# source /opt/ros/noetic/setup.bash  # installed ROS1 host only
catkin_make
```

**Limitations:** 주차 공간 탐지·차체 footprint 충돌·기어 명령 승인·주행 안전 확보 로직이 없습니다. Python 경로와 ROS 노드는 동등 구현이라고 보장할 수 없습니다. 액추에이터와 직접 연결하지 마세요.

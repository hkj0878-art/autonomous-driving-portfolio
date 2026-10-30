# ROS Local Path Planner — GADIS (Reconstruction)

**과거 대화의 설계와 오류 수정 내용을 참고해 새로 작성한 예제이며, 원본 실차 코드가 아닙니다.**

## What is included

- `ros1_ws/src/gadis_local_path/src/localpath_pub.cpp`: ROS1 주행 상태에 따른 Path selection 및 빈 경로/frame mismatch 가드 예제
- `gadis_core/control.py`: Pure Pursuit steering 계산, Menger curvature, 곡률 기반 Lookahead Distance
- `gadis_core/planning.py`: 테스트용 공통 `Pose2D` 및 좌표 변환 코드 (Parking과 일부 중복)
- `tests/`: 조향 방향, LD bounds, 곡률 및 좌표 변환 검증

### ROS1 example interfaces (unverified integration)

| Topic | Message | Role |
| --- | --- | --- |
| `/global_path`, `/lane_path`, `/uturn_path`, `/avoidance_path`, `/parking_path` | `nav_msgs/Path` | 경로 후보 입력 |
| `/gps_utm_odom` | `nav_msgs/Odometry` | 차량 위치 입력 |
| `/region` | `std_msgs/String` | 주행 상태(`go`, `parking_1`~`parking_4`, `stop`, `uturn`) |
| `/local_path`, `/selected_path` | `nav_msgs/Path` | 선택된 참조 경로 출력 |
| `/parking_path_number` | `std_msgs/Int32` | 정지/주차 관련 코드 예시 출력 |
| `/parking_gear` | `std_msgs/String` | 기어 상태 예시 출력 |

**주의:** 거리 기반의 회피 경로 선택은 실제 장애물 충돌 판단을 대체하지 않습니다. 주차 노드와 동일한 `/parking_gear`를 발행할 경우 다중 Publisher 충돌 가능성이 있습니다. 실제 통합 시 단일 소유자 또는 명시적 arbitration이 필수입니다.

## Offline validation

```bash
python -m pip install pytest
PYTHONPATH=. python -m pytest -q tests
```

Python 3.9+ 환경에서 실행을 확인했습니다. ROS1 Noetic + catkin이 설치된 별도 시스템에서 `cd ros1_ws && catkin_make`를 사용해 빌드 검증이 필요합니다. 실제 차량 제어 출력으로 사용해서는 안 됩니다.

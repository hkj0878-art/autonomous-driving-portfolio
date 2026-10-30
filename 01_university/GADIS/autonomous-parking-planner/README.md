# 자율주차 경로 생성

1/2 스케일 차량의 주차 미션에서 사용했던 경로 생성 방식을 정리했다. 차량은 제자리에서 방향을 바꿀 수 없기 때문에, 주차 위치뿐 아니라 진입 방향과 전·후진 전환을 함께 고려했다.

## 코드 구성

- gadis_core/planning.py — 간이 Hybrid A*, 3차 Bézier, 좌표계 변환
- gadis_core/parking.py — mid/goal/goal2 기준 주차 경로와 D → R → D → P 상태 전환
- ros1_ws/src/gadis_parking_planner/ — ROS1 C++ 경로 발행 예제
- tests/ — 경로·좌표계·주차 상태 단위 테스트

~~~bash
PYTHONPATH=. python -m pytest -q tests
~~~

Python 오프라인 테스트 8개가 통과했다. ROS1 C++은 Noetic 환경에서 별도 빌드 확인이 필요하다.

## 남은 작업

당시 다뤘던 콘 위치 입력부터 주차 슬롯을 계산하는 부분은 아직 복원하지 못했다. 현재 C++ 예제는 /parking_targets (geometry_msgs/PoseArray)로 mid, goal, goal2를 전달받아 /parking_path를 발행한다. /gps_utm_odom은 map 좌표계로 받는다고 가정했다.

Hybrid A*는 격자 점유 여부만 검사하므로 차량 차폭과 동적 장애물 충돌 검사까지 포함한 실제 주차 Planner는 아니다. 경로 추종/기어 제어기에 바로 연결해서는 안 된다.

원본과 재구성 범위의 차이는 [GADIS 검증 기록](../VALIDATION.md)에 남겼다.

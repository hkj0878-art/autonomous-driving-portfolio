# Local Path 선택 및 경로 추종

주행 구간에 따라 입력 경로를 전환하는 ROS 노드와 Pure Pursuit 수식을 정리했다. 일반 주행, U-turn, 주차 미션에서 경로를 선택하고 필요하면 기존 경로로 복귀하는 구조다.

## 코드 구성

- ros1_ws/src/gadis_local_path/src/localpath_pub.cpp — 경로 후보 선택 및 빈 경로/frame 검사
- gadis_core/control.py — Pure Pursuit, 세 점 곡률(Menger curvature), 곡률에 따른 LD 조절
- tests/ — 조향 방향·곡률·LD·좌표계 변환 검사

~~~bash
PYTHONPATH=. python -m pytest -q tests
~~~

Python 오프라인 테스트 4개가 통과했다. ROS1 노드는 아직 실제 Noetic 환경에서 빌드·연동 검증을 하지 못했다.

## 입력과 주의사항

C++ 예제는 /global_path, /lane_path, /uturn_path, /avoidance_path, /parking_path에서 경로를 받고 /region 상태를 참고한다. 선택 결과는 /local_path로 발행한다.

이 코드는 당시 대화에서 확인된 역할을 바탕으로 다시 구현한 것이다. 특히 장애물 회피 경로를 선택하는 부분은 후보 경로와의 거리 기준 예제이며, 충돌 위험을 판단하는 완성된 회피 Planner가 아니다.

주차 경로 노드와 /parking_gear 발행 역할이 겹치므로 실제 시스템에 통합할 때 단일 기어 명령 주체를 정해야 한다. 자세한 검증 범위는 [GADIS 검증 기록](../VALIDATION.md)에 적었다.

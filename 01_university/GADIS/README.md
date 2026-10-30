# GADIS

자율주행 차량의 주차 경로 생성과 Local Path 제어 쪽에서 작업했던 내용을 정리했다. 주차 미션에서 경로 생성, 기어 전환, 경로 추종을 다뤘고, 일반 주행에서는 상황에 따라 경로를 선택하는 ROS 노드를 개발했다.

현재 올린 코드는 당시 최종 소스 전체가 아니다. 이전 대화에 남은 알고리즘·파일명·구현 내용을 기준으로 다시 작성해 동작을 확인한 버전이다.

| 폴더 | 내용 |
| --- | --- |
| [autonomous-parking-planner](autonomous-parking-planner/) | Hybrid A*, Bézier, 다단계 주차 경로, D/R/D/P 상태 전환 |
| [ros-local-path-planner](ros-local-path-planner/) | Local Path 선택, U-turn/주차 상태 처리, Pure Pursuit |

Python 테스트는 각각 8개, 4개 통과했다. ROS1 노드는 실제 Noetic 환경에서 빌드해 보지 못했으며, 차량 제어용으로 검증된 코드가 아니다.

[검증 기록](VALIDATION.md)에 구현 범위와 추가 확인 사항을 정리했다.

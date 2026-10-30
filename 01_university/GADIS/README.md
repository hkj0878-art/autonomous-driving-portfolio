# GADIS — Autonomous Driving Planning (Reconstructed)

GADIS에서 진행했던 자율주차 및 Local Path / Path Tracking 작업을 GitHub용으로 분리한 **재구성 코드**입니다.

> **IMPORTANT:** 원본 코드가 아닙니다. 과거 대화·프로젝트 기록에서 확인된 설계를 기준으로 독립 작성했으며 실제 차량용 코드를 복구했다고 주장하지 않습니다. ROS1 컴파일·통합·차량 시험은 아직 완료하지 않았습니다. 실제 차량에 연결하지 마세요.

| Project | 구현 범위 | 현재 검증 상태 |
| --- | --- | --- |
| [autonomous-parking-planner](autonomous-parking-planner/) | Python Hybrid A* / Bézier / D→R→D→P state machine 및 ROS1 계획 노드 예제 | Python 오프라인 테스트 8개 통과 |
| [ros-local-path-planner](ros-local-path-planner/) | Python Pure Pursuit / curvature-dependent lookahead 및 ROS1 Path selector 예제 | Python 오프라인 테스트 4개 통과 |

## 실행 및 검증

각 프로젝트 폴더에서 Python 3.9+ 및 `pytest` 환경으로 아래를 실행합니다.

```bash
PYTHONPATH=. python -m pytest -q tests
```

ROS1 Noetic + catkin 환경이 준비되면 `ros1_ws`의 catkin 빌드와 Topic/TF 연동을 **별도로 검증해야 합니다.** 하드웨어에 사용할 경우 차량 동역학, 충돌 footprint, 경로 갱신 주기, 좌표계, 기어 명령 안전성을 처음부터 재검토해야 합니다.

## 원본 대조가 필요한 항목

- 콘 인지/군집 → 주차 슬롯/목표점 산출은 포함하지 않았습니다 (`/parking_targets`로 외부에서 입력).
- Python Hybrid A*는 grid 점 충돌만 검사하며, 차량 footprint나 동적 장애물과 충돌 검사하지 않습니다.
- 주차 노드에서 구간 기어 출력은 상태 전환 이벤트 예제입니다. 액추에이터/기어 인터록이 아닙니다.
- ROS1 노드는 이전 기록에서 확인된 토픽 역할을 참고한 별도 구현으로, 기존 실차 인터페이스와 호환성을 보장하지 않습니다.

자세한 사항은 [VALIDATION.md](VALIDATION.md)를 확인하세요.

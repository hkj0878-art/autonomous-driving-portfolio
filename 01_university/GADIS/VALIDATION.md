# GADIS recovery validation record

## Provenance

이 소스는 **대화 기반 재구성(Reconstructed)** 입니다. 과거 마지막 답변의 바이트 단위 원본이 아닙니다. 코드에 쓰인 주차 목표점, 메시지 인터페이스와 설정 중 확인 불가능한 부분은 예시 값/가정입니다.

## Performed in this workspace (2026-10-30)

- [x] ZIP에 포함된 Python/ROS1 코드와 패키지 메타데이터 검토
- [x] Hybrid A* 입력 파라미터 검증 보강 및 시작·목표 점유 셀 처리 보강
- [x] 주차 상태 머신 임계치 검증 보강
- [x] ROS1 주차 C++의 명시적인 `<algorithm>` 헤더 및 catkin C++14 컴파일 옵션 추가
- [x] Python 3.13.5 + pytest 9.0.2 오프라인 단위 테스트: Parking 8개 / Local Path 4개
- [x] Python 소스 Syntax/Compileall 검사
- [x] ROS `package.xml` 구문 검사 및 패키지/프로젝트명 일치 확인
- [ ] ROS1 Noetic `catkin_make` 실제 컴파일: **미실시 (현재 실행 환경에 ROS가 없음)**
- [ ] ROS Topic/TF Integration: 미실시
- [ ] Vehicle / K-City / hardware-in-the-loop: 미실시

## Explicit limits

1. 실제 최종 원본을 재발굴한 것이 아니므로 구현의 코드 동등성을 보장할 수 없습니다.
2. Hybrid A*는 주변 연속 공간·차량 footprint에 대한 충돌 안전성 검증이 누락돼 있습니다.
3. 주차 C++ 노드는 3개 외부 목표점을 가정하며, 과거의 콘 기반 주차 위치 산출 코드는 복구되지 않았습니다.
4. Path selection C++는 실제 장애물 추적/예측/충돌 비용 기반 회피 Planning이 아닙니다.
5. Pure Pursuit는 오프라인 수학 예제이며 차량 조향/토크 액추에이터 출력으로 연결하지 않습니다.
6. 두 C++ 노드는 모두 `/parking_gear`를 발행할 수 있습니다. 실사용 이전에 Publisher authority를 통합해야 합니다.

## Reproduce offline tests

```bash
cd 01_university/GADIS/autonomous-parking-planner
PYTHONPATH=. python -m pytest -q tests
cd ../ros-local-path-planner
PYTHONPATH=. python -m pytest -q tests
```

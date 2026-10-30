# 검증 기록 (2026-10-30)

## 확인한 범위

- 기존 복원 ZIP의 `Ego_lanechange`, `Ego_stop`, `SimulationTimeCondition`, `StoryboardElementStateCondition`, `egoSpeed`를 대조했다.
- 원본 복원 예제의 `SimulationTimeCondition rule="greaterOrEqual"`은 OpenSCENARIO XML 1.0에서 허용되는 값이 아니어서 `greaterThan`으로 수정했다.
- 차선 변경 완료 이벤트를 참조하는 정지 트리거가 연결돼 있는지 확인했다.
- 조건 및 파라미터 참조를 검사하고, 이벤트 중복과 누락을 확인하는 로컬 검사 도구를 작성했다.
- `egoSpeed` 분포를 20~100 km/h, 5 km/h 간격으로 만들어 m/s로 저장했다. **원본 기록의 단위는 확인되지 않았다.**
- XML 파싱 및 로컬 규칙 검사, 오류 주입 테스트를 포함한 pytest **12개 통과**.
- Python 구문 검사 통과.

## 아직 검증하지 못한 항목

- 공식 ASAM XSD를 통한 스키마 전체 검증. 로컬 환경에 해당 버전 XSD가 없어 실행하지 않았다. `--xsd` 옵션은 제공한다.
- MORAI ScenarioRunner 1.6.2에서 파라미터 분포 파일을 직접 실행할 수 있는지 여부.
- MGeo 맵의 연결 경로와 실제 차량 초기 위치. `../MGeo`는 예제 경로이며 이 저장소에 포함하지 않았다.
- 차량 카탈로그/차선 변경 목표의 실제 도로 네트워크 적합성.
- UN R157 검증 조건 및 시뮬레이터 실행 결과. 이 XML만으로 규정 적합성 검증을 수행했다고 볼 수 없다.

회사의 원본 시나리오, 지도와 결과 로그는 포함하지 않았다. 현재 소스는 과거 작업 기록을 바탕으로 만든 재현용 예제다.

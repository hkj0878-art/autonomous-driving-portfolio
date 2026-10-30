# MORAI OpenSCENARIO 시나리오 검증

MORAI 인턴 기간(2025.01–02)에 작업했던 OpenSCENARIO 시나리오 중 차선 변경 후 정지하는 흐름을 정리했다. 당시 사용한 환경은 MORAI ScenarioRunner 1.6.2, `V_RHT_StraightHighway_1` 맵(MGeo)이었다.

여기에 올린 XML은 당시 대화에서 확인된 이벤트 이름과 트리거를 바탕으로 다시 작성한 예제다. 회사에서 사용한 최종 파일이나 시뮬레이터 실행 결과를 그대로 옮긴 것은 아니다.

## 시나리오 구성

`ego_lane_change_then_stop.xosc`는 `Ego` 차량이 주행을 시작하고, 시뮬레이션 시간이 2초를 지나면 차선을 변경한 뒤 해당 이벤트가 완료되면 정지하는 흐름이다.

| 순서 | 동작 | 조건 |
| --- | --- | --- |
| 초기화 | `egoSpeed`로 시작 속도 지정 | 시나리오 시작 |
| 1 | `Ego_lanechange` | 시뮬레이션 2초 초과 + `ego_event_enabled=true` |
| 2 | `Ego_stop` | `Ego_lanechange`가 `completeState`로 전환 |
| 종료 | 시나리오 종료 | 시뮬레이션 40초 초과 |

`ego_speed_distribution.xml`에는 20~100 km/h를 5 km/h 간격으로 바꾸는 실험값을 기록했다. OpenSCENARIO의 `AbsoluteTargetSpeed` 값은 m/s이므로 XML에는 변환된 값을 저장했다. 기존 대화에는 `20~100, step=5`로만 기록돼 있어 속도 단위는 별도 확인이 필요하다.

## 파일

```text
scenarios/
  ego_lane_change_then_stop.xosc
  ego_speed_distribution.xml
tools/
  validate_scenario.py
tests/
  test_scenarios.py
```

`tools/validate_scenario.py`는 선언된 파라미터·이벤트 참조, 트리거 조건, 속도 분포를 확인한다. 지도 파일 존재 여부도 검사하지만, 예제에는 실제 MGeo 데이터가 없어서 경고가 나온다.

## 실행

Python 3.10 이상에서 XML 검사와 단위 테스트를 실행할 수 있다. 기본 XML 검사는 별도 패키지가 필요 없다.

```bash
python -m tools.validate_scenario
python -m pip install pytest
python -m pytest -q
```

사용 중인 버전에 맞는 ASAM OpenSCENARIO XSD 파일이 있다면 추가로 검증할 수 있다. 이때는 `lxml`이 필요하다.

```bash
python -m pip install lxml
python -m tools.validate_scenario --xsd /path/to/OpenSCENARIO.xsd
```

실제 MORAI에서 실행하려면 맵 경로, 차량 모델, 초기 위치와 차선 방향, ScenarioRunner 버전에 맞는 파라미터 분포 지원 여부를 확인해야 한다. 현재 XML만으로는 실행이 보장되지 않는다. 특히 지도 파일은 포함하지 않았으며 MORAI 실행 테스트도 진행하지 못했다.

[검증 기록](VALIDATION.md)에는 확인한 항목과 남은 작업을 적었다.

## 참고

- [ASAM OpenSCENARIO XML 1.0 설명서](https://releases.asam.net/OpenSCENARIO/1.0.0/ASAM_OpenSCENARIO_BS-1-2_User-Guide_V1-0-0.html)
- [ASAM OpenSCENARIO 표준](https://www.asam.net/standards/detail/openscenario-xml/)

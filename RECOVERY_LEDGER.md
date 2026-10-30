# 코드 정리 기록

이 저장소의 코드는 두 가지 경로로 정리하고 있다. 실제 원본을 찾은 경우는 그 출처를 연결하고, 원본 파일이 없는 경우는 당시 대화에서 확인되는 동작을 기준으로 새로 작성한다. 두 경우를 같은 '최종 원본'으로 취급하지 않는다.

| 프로젝트 | 현재 상태 | 확인한 자료 | 검증 |
| --- | --- | --- | --- |
| GADIS 자율주차 | 대화 기반 재구성 | 주차 목표점, Hybrid A*/Bézier, 상태 전환 기록 | Python 8개 통과, ROS 빌드 미확인 |
| GADIS Local Path | 대화 기반 재구성 | 경로 선택 로직, Pure Pursuit, U-turn 처리 기록 | Python 4개 통과, ROS 빌드 미확인 |
| Capstone 혼잡도 예측 | 원본 확인 후 개정 | [팀 저장소 Jongp](https://github.com/ahffkrmsidQKffl/Jongp) ai/src/train.py, score.py | Python 8개 통과, 실제 데이터 검증 미완료 |
| Beyless 인턴 — LIO-SAM | 실제 작업 원본 | 매핑·평가 코드, 실행 로그, 결과 지표 (rosbag 제외) | Python 77개 통과, 리포트 수치 78개 출처 검증, ROS 2 Humble 실행 |
| MORAI·Beyless 인턴 (그 외) | 문서만 | 이력서와 과거 개발 기록 | 소스 미등록 |
| Beyless 정규직 | 폴더만 | 프로젝트 분류 | 소스 미등록 |
| 기타 경진대회 | 폴더만 | 참가 내역 | 자료 추가 전 |

캡스톤의 기존 랜덤 분할, 초기 점유율 70% 가정, 미래 시각 입·출차량 사용 문제는 그대로 가져오지 않았다. 수정 내용은 [Capstone README](01_university/Capstone/parking-congestion-prediction/README.md)에 적었다.

실차나 시뮬레이터 실행 여부는 Python 단위 테스트와 별도로 관리한다. 회사 관련 자료는 공개 권한을 확인하기 전까지 올리지 않는다.

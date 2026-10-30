# 자율주행 개발 프로젝트

학교에서 진행한 자율주행 프로젝트부터 인턴십, 현재 업무까지 개발 경험을 정리하는 저장소입니다. 경로 계획과 제어, 센서 기반 위치 추정, 학습 기반 주행 모델을 중심으로 작성하고 있습니다.

전체 프로젝트가 모두 공개된 상태는 아닙니다. 학교에서 작성한 코드는 원본을 확인하거나 다시 정리한 뒤 추가하고, 회사 업무는 공개 가능한 범위의 설명만 우선 남깁니다.

## 대학생 활동

| 프로젝트 | 주요 내용 | 정리 상태 |
| --- | --- | --- |
| [GADIS — 자율주차](01_university/GADIS/autonomous-parking-planner/) | Hybrid A*, Bézier, 전·후진 주차 | 코드·테스트 정리 |
| [GADIS — Local Path](01_university/GADIS/ros-local-path-planner/) | 주행 상태별 경로 선택, Pure Pursuit | 코드·테스트 정리 |
| [Capstone — 주차장 혼잡도 예측](01_university/Capstone/parking-congestion-prediction/) | XGBoost, 시간별 주차 데이터, 추천 점수 | 코드·테스트 정리 |

GADIS 소스는 이전 작업 기록을 참고해 다시 작성했다. ROS1 C++은 아직 빌드 및 차량 연동까지 검증한 상태는 아니다.

캡스톤은 [기존 팀 저장소](https://github.com/ahffkrmsidQKffl/Jongp)에서 실제 Python 코드를 확인해 학습/추론 부분을 다시 정리했다. 원본 데이터를 이용한 성능 재평가는 아직 진행하지 않았다.

## 인턴십

| 소속 | 내용 |
| --- | --- |
| [MORAI](02_internships/MORAI/openscenario-safety-validation/) | OpenSCENARIO 차선 변경·정지 이벤트, 속도 파라미터 스윕 (재구성 XML·테스트) |
| [Beyless — SLAM](02_internships/Beyless/lio-sam-integration/) | LIO-SAM 매핑, RTK GNSS 기준 ATE 0.31 m, 지도 기반 재주행 위치추정 (코드·결과 정리) |
| [Beyless — 논문 리뷰](02_internships/Beyless/e2e-paper-reviews/) | MTR/MTR++, HiVT, DistillDrive 구조 및 공개 코드 분석 (리뷰 문서 정리) |

SLAM 폴더에는 직접 작성한 파이프라인 코드와 결과를 올렸고, 원본 주행 데이터는 포함하지 않았다. MORAI 폴더에는 대화 기록을 참고해 다시 작성한 XML과 검증 스크립트를 올렸다. MORAI 시뮬레이터 실행은 확인하지 못했다. 논문 리뷰 폴더에는 공개 논문·공식 저장소 분석과 입력 데이터 파이프라인 점검 내용을 정리했으며, 회사 내부 코드는 포함하지 않았다.

## 정규직 경력 — Beyless

- [Alpamayo Action Expert](03_career/Beyless/alpamayo-action-expert/)
- [E2E Vehicle Integration](03_career/Beyless/e2e-vehicle-integration/)
- [GRPO Trajectory Optimization](03_career/Beyless/grpo-trajectory-optimization/)
- [MPC Evaluation Agent](03_career/Beyless/mpc-evaluation-agent/)

각 폴더는 주제별로 분류하기 위한 공간이다. 자료는 공개 범위를 확인하면서 추가할 계획이다.

## 기타 대회

[대회 참가 기록](04_other_competitions/)은 프로젝트 구현과 중복되지 않도록 결과 자료 위주로 정리할 예정이다.

| 대회 | 내용 | 결과 |
| --- | --- | --- |
| [2026 자율주행 AI 챌린지 — Motion Planning](04_other_competitions/2026_Ai_challenge/) | 가중치 없는 분석적 플래너 + 단기 안전 실드 (제출본 소스·채점 결과) | 공식 test 0.98606, 리더보드 4위권 |

---

코드의 출처와 검증 상태는 [정리 기록](RECOVERY_LEDGER.md)에 남겼다. 단위 테스트 통과 여부와 ROS 빌드·실차 검증 여부는 별도로 표시한다.

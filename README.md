# Autonomous Driving & AI Engineering Portfolio

자율주행 개발 관련 학부 프로젝트, 인턴십, 정규직 경력 및 경진대회 기록을 정리한 포트폴리오입니다.

> **현재 상태: 프로젝트별 문서와 폴더를 정리하는 단계입니다.** 실제 소스코드와 검증 결과는 원본 확인 및 공개 가능 여부를 검토한 후 업로드합니다. 대화 기반으로 재구성한 예제는 실제 원본과 구분합니다.

## 01 — University (대학생 활동)

| 구분 | 프로젝트 | 내용 |
| --- | --- | --- |
| GADIS | [Autonomous Parking Planner](01_university/GADIS/autonomous-parking-planner/README.md) | Hybrid A*, Bézier, 전/후진 주차 경로 계획 |
| GADIS | [ROS Local Path Planner](01_university/GADIS/ros-local-path-planner/README.md) | Path Arbitration, 장애물 회피, U-turn, Pure Pursuit |
| Capstone | [Parking Congestion Prediction](01_university/Capstone/parking-congestion-prediction/README.md) | XGBoost 기반 주차장 혼잡도 예측 |

## 02 — Internships (인턴 활동)

| 소속 | 프로젝트 | 내용 |
| --- | --- | --- |
| MORAI | [OpenSCENARIO Safety Validation](02_internships/MORAI/openscenario-safety-validation/README.md) | ASAM OpenSCENARIO 테스트 시나리오 |
| Beyless | [LIO-SAM Integration](02_internships/Beyless/lio-sam-integration/README.md) | LiDAR–IMU–GNSS, Mapping / Localization |
| Beyless | [E2E Paper Reviews](02_internships/Beyless/e2e-paper-reviews/README.md) | MTR / DistillDrive 논문 및 구조 분석 |

## 03 — Career (Beyless)

정규직 경력 프로젝트는 **주제별 작업 공간만 구성**했습니다. 소스코드와 내부 실험 자료는 공개 범위를 검토한 뒤 추가합니다.

- [Alpamayo Action Expert](03_career/Beyless/alpamayo-action-expert/README.md) — Flow Matching / Consistency Distillation / Step Reduction
- [E2E Vehicle Integration](03_career/Beyless/e2e-vehicle-integration/README.md) — Trajectory 및 차량 제어 연동
- [GRPO Trajectory Optimization](03_career/Beyless/grpo-trajectory-optimization/README.md) — Reward 및 경로 평가
- [MPC Evaluation Agent](03_career/Beyless/mpc-evaluation-agent/README.md) — 제어 성능 평가와 실험 자동화

## 04 — Other Competitions (기타 자율주행 대회)

[대회별 참여 기록](04_other_competitions/README.md)을 정리할 공간입니다. GADIS와 중복되는 코드는 연결만 유지합니다.

## Repository tree

```text
autonomous-driving-portfolio/
├── 01_university/
│   ├── GADIS/
│   │   ├── autonomous-parking-planner/
│   │   └── ros-local-path-planner/
│   └── Capstone/
│       └── parking-congestion-prediction/
├── 02_internships/
│   ├── MORAI/
│   │   └── openscenario-safety-validation/
│   └── Beyless/
│       ├── lio-sam-integration/
│       └── e2e-paper-reviews/
├── 03_career/
│   └── Beyless/
│       ├── alpamayo-action-expert/
│       ├── e2e-vehicle-integration/
│       ├── grpo-trajectory-optimization/
│       └── mpc-evaluation-agent/
└── 04_other_competitions/
```

## Source verification policy

- **Original:** 실제 최종 원본과 버전을 확인한 소스
- **Reconstructed:** 이전 작업 기록에서 알고리즘을 바탕으로 새로 구현한 예제
- **Planned:** 구조와 문서만 마련한 프로젝트
- 단위 테스트 통과와 ROS 빌드/시뮬레이션/실차 검증을 구분합니다.
- 회사의 비공개 코드, 데이터, 고객 정보, 모델 가중치 및 인증정보를 업로드하지 않습니다.

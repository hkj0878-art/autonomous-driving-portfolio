# 데이터 로더 / 입력 파이프라인 메모

논문 코드를 실제 데이터에 붙일 때는 모델 구조보다 먼저 입력 데이터가 기대한 형태인지 확인해야 한다. 인턴 기간에도 Motion Prediction 모델의 데이터 로더와 입력 파이프라인을 검토·수정하는 작업을 했다. 아래는 공개 코드와 당시 경험을 바탕으로 정리한 점검 항목이다. 회사 코드나 데이터 스키마는 포함하지 않았다.

## 1. 모델이 받는 입력 구분

| 모델 | 주요 입력 | 입력 측에서 확인할 일 |
| --- | --- | --- |
| MTR | 객체의 과거 상태, 지도 폴리라인, 예측 대상 인덱스 | 좌표 변환, 유효 마스크, 시간 샘플링, 객체 유형 |
| HiVT | 객체 과거 이동량과 지도 벡터 | 객체 로컬 좌표, 장면 회전/이동 처리, 이웃 관계 |
| DistillDrive | 다중 카메라 영상, 주행 상태 및 관련 주석 | 카메라 메타데이터, 시간 정렬, Teacher/Student 입력 구분 |

세 모델의 데이터 형태는 서로 다르다. MTR과 HiVT는 주변 객체 **motion forecasting**에 초점이 있고, DistillDrive는 Ego **planning**을 학습하는 E2E 주행 모델에 해당한다.

## 2. 실제 이식 시 점검할 내용

**시간축** — 데이터셋에서 기록한 Timestamp, FPS, 관측 프레임 수와 예측 Horizon이 모델 설정에 맞는지 확인한다. 과거 데이터로 현재 예측을 만들어야 하므로 target 시점 이후 데이터를 feature에 섞지 않는다.

**좌표계** — Global/ego/agent-local 좌표계 중 어느 기준을 사용하는지, heading 단위가 rad인지, x/y 축과 회전 방향이 일치하는지 확인한다.

**결측과 마스크** — 객체가 관측되지 않는 프레임을 0좌표로 오해하지 않도록 validity mask를 유지한다. 지도 폴리라인 수와 객체 수가 장면마다 다르므로 batch padding도 확인한다.

**클래스/라벨** — 실제 데이터의 차량·보행자·자전거 등 클래스가 공개 코드의 taxonomy와 어떻게 매핑되는지 확인한다. 미확인 클래스의 임의 변환은 피한다.

**평가 정합** — minADE, minFDE, Miss Rate 등은 예측 프레임과 GT의 시간축 및 좌표계가 같을 때 비교한다. Planning 성능을 검증하는 경우에는 별도의 collision/comfort 지표가 필요하다.

## 3. 살펴본 공개 코드

- [MTR Waymo Dataset](https://github.com/sshaoshuai/MTR/blob/master/mtr/datasets/waymo/waymo_dataset.py): 객체·지도 특징을 batch dictionary에 묶는 방식
- [MTR Data Preprocess](https://github.com/sshaoshuai/MTR/blob/master/mtr/datasets/waymo/data_preprocess.py): 원본 Waymo 데이터 변환
- [HiVT Argoverse Dataset](https://github.com/ZikangZhou/HiVT/blob/main/datasets/argoverse_v1_dataset.py): agent/map 그래프 구성
- [DistillDrive NuScenes3DDatasetLight](https://github.com/YuruiAI/DistillDrive/blob/master/projects/mmdet3d_plugin/datasets/nuscenes_3d_dataset_light.py): annotation 읽기, pipeline 구성, sample 생성
- [DistillDrive model](https://github.com/YuruiAI/DistillDrive/blob/master/projects/mmdet3d_plugin/models/distilldrive.py): Teacher/Student 별 입력 처리

원본 구현을 그대로 복사하거나 회사용 변환 코드를 여기에 올리지는 않는다. 이후 공개 가능한 예제 데이터를 확보하면 최소 샘플 단위에서 shape/dtype/time/frame 검사를 하는 스크립트를 별도로 추가할 수 있다.

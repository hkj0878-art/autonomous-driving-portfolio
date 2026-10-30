# MTR (NeurIPS 2022)

**논문:** [Motion Transformer with Global Intention Localization and Local Movement Refinement](https://arxiv.org/abs/2209.13508)  
**공개 코드:** [sshaoshuai/MTR](https://github.com/sshaoshuai/MTR)

## 읽은 이유

차량 주변 객체의 미래 궤적을 하나로 단정하지 않고, 좌회전·직진·감속처럼 가능한 경로를 여러 개 예측하는 구조를 확인하려고 살펴봤다.

## 구조 정리

입력은 객체의 과거 상태와 HD Map 폴리라인이다. Encoder가 객체·지도 특징을 묶고, Decoder가 여러 개의 motion query를 이용해 미래 경로를 만든다.

MTR은 의도 위치를 먼저 찾는 **Global Intention Localization**과 궤적을 단계적으로 조정하는 **Local Movement Refinement**를 분리한다. 정해진 후보 목표점을 전부 검사하는 방식 대신 learnable intention query를 사용하며, 예측 결과에는 궤적별 확률 정보를 포함한다.

개별 객체를 기준으로 여러 미래 경로를 예측한다는 점이 장점이다. 반면 여러 객체의 미래를 한 번에 일관되게 최적화하는 구조는 아니라서, 차량 간 상호작용이 복잡한 장면에서는 한계가 생길 수 있다.

## 코드에서 확인한 부분

| 파일 | 확인한 내용 |
| --- | --- |
| [mtr_encoder.py](https://github.com/sshaoshuai/MTR/blob/master/mtr/models/context_encoder/mtr_encoder.py) | Polyline Encoder와 global/local attention |
| [mtr_decoder.py](https://github.com/sshaoshuai/MTR/blob/master/mtr/models/motion_decoder/mtr_decoder.py) | Intention query, Dynamic Map Collection, 궤적 갱신 |
| [waymo_dataset.py](https://github.com/sshaoshuai/MTR/blob/master/mtr/datasets/waymo/waymo_dataset.py) | Waymo 데이터의 학습 입력 구성 |
| [tools/train.py](https://github.com/sshaoshuai/MTR/blob/master/tools/train.py) | 학습 실행 진입점 |

Decoder의 `build_motion_query()`, `apply_dynamic_map_collection()`, `apply_transformer_decoder()`를 이어서 보면 의도점 → 주변 지도 특징 선택 → 경로 보정 흐름을 따라가기 쉽다.

## 당시 확인했던 문제

학습 환경 점검 과정에서 `cluster_64_center_dict.pkl` 경로를 찾지 못하는 오류를 다룬 기록이 있다. 쿼리 초기화를 위한 의도점 파일이 데이터 준비 단계에서 생성·배치돼야 한다는 점을 확인했다. 현재 공개 저장소에는 해당 이름의 파일이 있으므로, 이 오류를 공개 코드 자체의 결함으로 단정할 수는 없다.

논문의 수치나 Waymo 평가 결과를 직접 재현한 것은 아니다. 여기에는 코드 분석과 학습 준비 과정에서 확인한 사항만 남긴다.

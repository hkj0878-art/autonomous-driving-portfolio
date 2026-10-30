# E2E / Motion Prediction 논문 리뷰

2025년 Beyless 인턴 기간에 살펴봤던 Motion Prediction과 E2E Planning 논문을 정리했다. 논문 내용만 요약하기보다는 **공개 코드에서 입력이 들어와 결과가 나오는 경로**를 확인하는 데 집중했다.

## 정리한 논문

| 논문 | 핵심 내용 | 리뷰 |
| --- | --- | --- |
| MTR (NeurIPS 2022) | Intention Query 기반 다중 미래 궤적 예측 | [MTR](notes/mtr.md) |
| MTR++ (2023/2024) | 여러 객체의 미래를 동시에 예측하고 상호작용 반영 | [MTR++](notes/mtrpp.md) |
| HiVT (CVPR 2022) | Local Encoder → Global Interaction 구조 | [HiVT](notes/hivt.md) |
| DistillDrive (ICCV 2025) | 구조화된 Planning Teacher에서 E2E Student로 지식 증류 | [DistillDrive](notes/distilldrive.md) |

MTR/MTR++와 HiVT는 주변 객체의 미래를 예측하는 모델이다. DistillDrive는 카메라 기반 E2E 주행 모델을 Planning 관점에서 학습시키는 작업이라 문제 설정부터 다르다.

## 당시 작업과 연결되는 부분

MTR은 [공식 저장소](https://github.com/sshaoshuai/MTR)의 Encoder·Decoder와 Waymo 데이터 로더를 중심으로 분석했다. 의도점을 이용한 쿼리 생성, Dynamic Map Collection, 여러 후보 경로 출력이 주요 확인 대상이었다.

DistillDrive는 [공개 구현](https://github.com/YuruiAI/DistillDrive)의 Teacher/Student 분기와 nuScenes Dataset 처리를 살펴봤다. 인턴 업무 중에는 Prediction 모델에 내부 데이터를 적용하기 위해 데이터 로더 구조와 입력 파이프라인을 수정한 경험이 있다. 이 작업의 비공개 코드는 포함하지 않았고, 공개 코드와 연결되는 일반적인 점검 내용만 [입력 파이프라인 메모](data_pipeline_notes.md)에 정리했다.

## 확인 범위

- 공개 논문과 공식 저장소에서 모델 구조, 코드 경로, 입력 데이터 구성을 확인했다.
- 회사 내부 수정 코드는 포함하지 않았다.
- 해당 논문들의 학습 결과나 벤치마크를 직접 재현했다는 뜻은 아니다.
- 문서에 언급한 논문 성능 수치는 모두 저자들의 결과로 구분한다.

별도 실험 결과가 확보되면 논문 리뷰와 혼동되지 않도록 실험 코드·로그를 나눠 정리할 예정이다.

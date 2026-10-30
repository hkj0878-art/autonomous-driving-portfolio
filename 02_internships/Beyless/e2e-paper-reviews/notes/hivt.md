# HiVT (CVPR 2022)

**논문:** [HiVT: Hierarchical Vector Transformer for Multi-Agent Motion Prediction](https://openaccess.thecvf.com/content/CVPR2022/html/Zhou_HiVT_Hierarchical_Vector_Transformer_for_Multi-Agent_Motion_Prediction_CVPR_2022_paper.html)  
**공개 코드:** [ZikangZhou/HiVT](https://github.com/ZikangZhou/HiVT)

## 확인한 부분

HiVT는 각 객체 주변의 움직임과 지도 관계를 먼저 처리한 후, 객체들 사이의 상호작용을 따로 처리한다.

- **Local Encoder:** 객체별 과거 움직임과 가까운 지도·주변 객체 관계 표현
- **Global Interactor:** 장면 속 여러 객체의 특징을 결합
- **Decoder:** 여러 미래 궤적 후보 출력

좌표 이동·회전에 대한 불변/등변 성질을 고려해 장면 표현의 중복을 줄이는 점도 특징이다. 한 번의 forward pass에서 여러 객체를 예측한다는 면에서 MTR의 개별 객체 중심 처리와 비교했다.

## 코드 확인 위치

| 파일 | 내용 |
| --- | --- |
| [models/local_encoder.py](https://github.com/ZikangZhou/HiVT/blob/main/models/local_encoder.py) | 지역 문맥 인코딩 |
| [models/global_interactor.py](https://github.com/ZikangZhou/HiVT/blob/main/models/global_interactor.py) | 객체 간 상호작용 |
| [models/decoder.py](https://github.com/ZikangZhou/HiVT/blob/main/models/decoder.py) | 미래 경로 분기 |
| [datasets/argoverse_v1_dataset.py](https://github.com/ZikangZhou/HiVT/blob/main/datasets/argoverse_v1_dataset.py) | Argoverse 기반 입력 구성 |

논문을 읽으며 MTR/MTR++의 query 기반 접근과 local→global 계층형 접근을 비교했다. 이 모델 역시 실제 주행에 적용하려면 데이터 분포, 지도 가용성, 상호작용이 많은 구간의 예측 안정성을 별도로 평가해야 한다.

모델을 직접 학습해 얻은 성능 수치는 이 기록에 포함하지 않았다.

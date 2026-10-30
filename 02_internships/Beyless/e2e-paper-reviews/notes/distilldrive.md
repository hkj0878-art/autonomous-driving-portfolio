# DistillDrive (ICCV 2025)

**논문:** [DistillDrive: End-to-End Multi-Mode Autonomous Driving Distillation by Isomorphic Hetero-Source Planning Model](https://openaccess.thecvf.com/content/ICCV2025/html/Yu_DistillDrive_End-to-End_Multi-Mode_Autonomous_Driving_Distillation_by_Isomorphic_Hetero-Source_Planning_ICCV_2025_paper.html)  
**공개 코드:** [YuruiAI/DistillDrive](https://github.com/YuruiAI/DistillDrive)

## 어떤 문제를 다루는가

E2E 주행 모델이 정답 경로 하나를 따라 하는 것에 집중하면, 같은 장면에서 가능한 여러 주행 방식과 계획 의도를 충분히 배우기 어렵다. DistillDrive는 구조화된 장면 표현을 사용하는 Planning Teacher에서 다양한 경로 인스턴스를 만들고, 이를 카메라 입력 기반 Student 학습에 활용한다.

일반적인 Detection/Prediction 모델처럼 주변 객체의 미래만 예측하는 논문은 아니다. **Ego의 Planning을 학습시키기 위한 지식 증류**가 중심이라 MTR과는 문제 설정이 다르다.

## 공개 코드에서 확인한 구조

| 파일 | 확인한 역할 |
| --- | --- |
| [models/distilldrive.py](https://github.com/YuruiAI/DistillDrive/blob/master/projects/mmdet3d_plugin/models/distilldrive.py) | `task_role`로 Teacher / Student 분기, 학습·추론 경로 |
| [datasets/nuscenes_3d_dataset_light.py](https://github.com/YuruiAI/DistillDrive/blob/master/projects/mmdet3d_plugin/datasets/nuscenes_3d_dataset_light.py) | `NuScenes3DDatasetLight`의 annotation, pipeline, sample 구성 |
| [models/distilldrive_head.py](https://github.com/YuruiAI/DistillDrive/blob/master/projects/mmdet3d_plugin/models/distilldrive_head.py) | 모델 출력 헤드 |
| [projects/configs](https://github.com/YuruiAI/DistillDrive/tree/master/projects/configs) | 단계별 학습 설정 |

`distilldrive.py`에서 `task_role == 'Teacher'`와 `task_role == 'Student'`일 때 입력 처리·학습·테스트 분기가 달라지는 것을 볼 수 있다. 데이터 로더 쪽에서는 annotation 형식뿐 아니라 각 sample이 model pipeline에 전달되는 구조까지 함께 봐야 한다.

## 데이터 적용 관점

사내 데이터에 공개 모델을 연결하는 과정에서는 원본 학습 코드를 먼저 바꾸기보다 데이터 경로와 형식을 맞추는 작업이 우선이었다. Dataset에서 annotation을 읽고, 필요한 필드를 묶고, 학습·평가 과정이 기대하는 키와 좌표계를 일치시키는 단계를 확인했다.

이력서에 적었던 데이터 로더 수정·입력 파이프라인 커스터마이징 경험은 이 부분과 연결된다. 다만 당시 수정한 내부 코드를 이 저장소에서 재현하거나 공개한 것은 아니다.

논문이 보고한 nuScenes/NAVSIM 성능은 저자들의 실험 결과이며 내 재현 결과가 아니다.

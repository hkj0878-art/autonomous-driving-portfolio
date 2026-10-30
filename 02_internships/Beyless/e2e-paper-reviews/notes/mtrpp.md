# MTR++ (2023 arXiv / 2024 TPAMI)

**논문:** [MTR++: Multi-Agent Motion Prediction with Symmetric Scene Modeling and Guided Intention Querying](https://arxiv.org/abs/2306.17770)  
**비교 대상:** [MTR 공식 저장소](https://github.com/sshaoshuai/MTR)

## MTR과 달라진 점

MTR은 특정 대상 객체를 중심으로 여러 미래 경로를 예측한다. MTR++는 이를 확장해 여러 객체의 미래를 동시에 예측하고, 예측 대상 간 상호작용이 결과에 반영되도록 설계했다.

핵심은 **Symmetric Scene Modeling**과 **Mutually-Guided Intention Querying**이다. 객체마다 별도의 타깃 중심 장면을 반복 구성하는 대신 대칭적인 장면 표현을 활용하고, intention query 사이의 영향을 고려한다.

## 구현 관점에서 본 차이

- 한 객체의 6개 후보 경로가 개별적으로 그럴듯하더라도, 여러 객체를 합치면 서로 충돌하는 장면이 나올 수 있다.
- 따라서 multi-agent prediction에서는 minADE/minFDE뿐 아니라 장면 수준의 일관성도 중요하다.
- 모델 출력이 타깃별로 독립된 텐서인지, 여러 객체의 예측을 묶어 내는 형태인지 확인해야 downstream planning과 연결할 수 있다.

인턴 당시 MTR과 함께 살펴본 비교 대상이다. 이 폴더에는 MTR++ 원본 구현이나 학습 결과를 올리지 않았다. 위 MTR 코드 링크는 기본 MTR 구현의 구조를 확인하기 위한 참고 자료이지, MTR++의 완전한 공식 구현이라고 표시하지 않는다.

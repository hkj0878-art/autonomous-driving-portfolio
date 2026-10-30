# 주차장 혼잡도 예측 및 추천

2025년 종합프로젝트에서 진행한 주차장 혼잡도 예측 및 추천 점수 계산 부분을 정리했다.

[당시 팀 저장소](https://github.com/ahffkrmsidQKffl/Jongp)의 ai/src/train.py, ai/src/score.py를 확인했다. 이 폴더는 기존 파일을 통째로 옮긴 것이 아니라 학습과 추론 코드를 분리하고, 평가 중 확인된 문제를 수정한 버전이다. 프론트엔드와 백엔드 코드는 포함하지 않았다.

## 작업 내용

- **특징 생성:** 직전 1시간 입·출차량과 7·14·21일 전 같은 시간의 혼잡도를 사용한다. 해당 시각의 과거 기록이 없으면 그 행은 학습에서 제외한다.
- **모델 학습:** XGBoost 회귀 모델을 사용한다. 마지막 20% 시각을 검증 구간으로 남겨두고, 학습된 전처리기와 컬럼 순서를 모델과 함께 저장한다.
- **주차장 추천:** 예측 혼잡도·거리·요금·리뷰를 점수로 바꾸고 네 가지 우선순위로 정렬한다. 가중치는 기존 score.py에 있던 값을 따른다.
- **입출력:** 기존 서버 연동 구조를 참고해 표준입력 JSON으로 예측과 추천을 실행한다. 별도 Flask 서버는 구현하지 않았다.

| 파일 | 역할 |
| --- | --- |
| parking_prediction/features.py | 시간별 입력 및 과거 관측값 결합 |
| parking_prediction/train.py | 모델 학습, 검증, 추론 |
| parking_prediction/recommender.py | 거리·요금 계산 및 추천 점수 |
| parking_prediction/cli.py | JSON 입력·출력 |
| parking_prediction/demo_data.py | 실행용 가상 데이터 |
| tests/test_project.py | 주요 로직 단위 테스트 |

## 실행

Python 3.10 이상을 기준으로 정리했다.

~~~bash
python -m pip install -r requirements.txt
PYTHONPATH=. python -m pytest -q tests

python -m parking_prediction.demo_data --output demo.csv
python -m parking_prediction.train demo.csv --model artifacts/model.joblib
~~~

추천 점수 계산은 모델 파일이 없어도 실행할 수 있다.

~~~bash
cat << 'JSON' | python -m parking_prediction.cli rank
{"candidates":[
  {"p_id":"A", "congestion_pct":20, "distance_km":2.5, "fee":3500, "review":4.2},
  {"p_id":"B", "congestion_pct":80, "distance_km":1.0, "fee":1500, "review":4.7}
]}
JSON
~~~

학습 데이터는 주차장명, 일시, 주차면수, 입차대수, 출차대수, **실측 혼잡도(0~100)** 컬럼이 필요하다.

## 원본 코드와 다른 점

원본 train.py에는 초기 주차 대수를 전체 주차면수의 70%로 가정하고 입·출차 누적으로 혼잡도를 만드는 부분이 있다. 이 방식은 실제 차량 점유 대수와 차이가 날 수 있다. 지금 코드는 혼잡도 정답이 주어진 경우에만 학습하며, 실측치가 없는 원본 CSV만으로는 학습하지 않는다.

당시 코드는 동일 시간대의 입·출차량과 랜덤 train/test 분할을 사용했다. 여기서는 미래 정보가 입력되지 않도록 이전 시간의 입·출차량으로 바꾸고, 시각 기준으로 학습/검증을 분리했다. 과거 혼잡도는 각 예측 시점에 관측 가능하다는 가정이다.

가상 데이터의 테스트 점수는 모델이 정상 작동하는지만 보여준다. 실제 데이터의 MAE나 R²를 다시 측정한 결과가 아니다. [검증 기록](VALIDATION.md)에 현재 상태를 남겼다.

## 참고

- [Jongp — 2025년 종합프로젝트 팀 저장소](https://github.com/ahffkrmsidQKffl/Jongp)
- 모델 파라미터는 기존 train.py와 동일하게 유지했다: n_estimators=300, max_depth=10, learning_rate=0.02, subsample=0.8, colsample_bytree=0.8.
- 원본 대용량 CSV, 학습 모델 파일, 다른 팀원이 작성한 프론트엔드·백엔드는 복사하지 않았다.

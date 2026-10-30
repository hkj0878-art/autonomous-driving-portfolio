from pathlib import Path
import joblib
import numpy as np
import pandas as pd
import pytest
from parking_prediction.demo_data import make_demo
from parking_prediction.features import FEATURES,adapt_csv,prepare_features
from parking_prediction.train import train_model,predict_congestion
from parking_prediction.recommender import haversine_km,parking_fee,rank_candidates

def test_lag_exact_timestamps():
    raw=make_demo(30)
    raw=raw[~((raw["주차장명"]=="가상주차장_A") &
              (raw["일시"]==pd.Timestamp("2025-01-04 08:00")))]
    feat=prepare_features(raw)
    assert set(FEATURES).issubset(feat.columns)
    row=feat[(feat["주차장명"]=="가상주차장_a") &
             (feat["일시"]==pd.Timestamp("2025-01-25 08:00"))]
    assert row.empty
    assert not feat[(feat["주차장명"]=="가상주차장_b") &
                    (feat["일시"]==pd.Timestamp("2025-01-25 08:00"))].empty

def test_future_flow_not_used_in_features():
    df=make_demo(31)
    original=prepare_features(df)
    edited=df.copy()
    target=(edited["주차장명"]=="가상주차장_A") & (edited["일시"]==pd.Timestamp("2025-01-30 10:00"))
    edited.loc[target,"입차대수"]=9999
    changed=prepare_features(edited)
    def select(df):
        return df[(df["주차장명"]=="가상주차장_a") &
                  (df["일시"]==pd.Timestamp("2025-01-30 10:00"))].iloc[0]
    assert select(changed)[FEATURES].equals(select(original)[FEATURES])

def test_input_validation():
    with pytest.raises(ValueError,match="혼잡도"):
        prepare_features(make_demo(28).drop(columns=["혼잡도"]))
    raw=make_demo(28)
    raw.loc[0,"주차면수"]=0
    with pytest.raises(ValueError,match="주차면수"):
        prepare_features(raw)

def test_input_schema_adapter():
    raw=make_demo(25).rename(columns={"일시":"시간대","혼잡도":"혼잡도(%)"})
    assert not prepare_features(adapt_csv(raw)).empty

def test_train_and_predict(tmp_path:Path):
    model_path=tmp_path/"model.joblib"
    metrics=train_model(make_demo(28),model_path,n_estimators=30)
    assert model_path.exists() and metrics["train_rows"]>0 and metrics["validation_rows"]>0
    model=joblib.load(model_path)
    assert model["fit_columns"]==FEATURES
    row=prepare_features(make_demo(28)).iloc[-1]
    value=predict_congestion(model,row[FEATURES].to_dict())
    assert 0<=value<=100 and np.isfinite(value)
    with pytest.raises(ValueError,match="누락"):
        predict_congestion(model,{"요일":3})

def test_rank_scenarios():
    candidates=[
        {"p_id":"A","congestion_pct":10,"distance_km":3,"fee":4000,"review":4},
        {"p_id":"B","congestion_pct":90,"distance_km":1,"fee":1000,"review":5},
    ]
    scores=rank_candidates(candidates)
    assert set(scores)=={"혼잡도우선","거리우선","요금우선","리뷰우선"}
    assert scores["혼잡도우선"][0]["p_id"]=="A"
    assert all(0<=row["score"]<=100 for rows in scores.values() for row in rows)

def test_fee_cap_and_haversine():
    assert parking_fee(120,1000,30,500,10)==5500
    assert parking_fee(120,1000,30,500,10,daily_cap=3500)==3500
    assert haversine_km(37.5,127,37.5,127)==0

def test_rank_reject_invalid_values():
    with pytest.raises(ValueError):
        rank_candidates([{"p_id":"A","congestion_pct":200,"distance_km":0,"fee":0,"review":4}])
    with pytest.raises(ValueError):
        rank_candidates([{"p_id":"A","congestion_pct":0,"distance_km":0,"fee":0,"review":4}]*2)

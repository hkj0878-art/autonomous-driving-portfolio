"""XGBoost 학습과 시계열 홀드아웃 평가."""
from __future__ import annotations
import argparse
from pathlib import Path
import joblib
import numpy as np
import pandas as pd
from sklearn.compose import ColumnTransformer
from sklearn.metrics import mean_absolute_error, r2_score
from sklearn.preprocessing import OrdinalEncoder, StandardScaler
from xgboost import XGBRegressor
from .features import CATEGORICAL, FEATURES, NUMERIC, TARGET, adapt_csv, prepare_features

def train_model(raw: pd.DataFrame, output: str | Path, *, n_estimators: int=300) -> dict:
    data=prepare_features(adapt_csv(raw))
    times=sorted(data["일시"].unique())
    if len(times)<10: raise ValueError("학습에 필요한 시각이 부족합니다")
    cutoff=times[int(len(times)*0.8)]
    train=data[data["일시"]<cutoff]
    valid=data[data["일시"]>=cutoff]
    if train.empty or valid.empty: raise ValueError("학습/검증 구간 중 하나가 비어 있습니다")
    preprocessor=ColumnTransformer([
        ("park",OrdinalEncoder(handle_unknown="use_encoded_value",unknown_value=-1),CATEGORICAL),
        ("numeric",StandardScaler(),NUMERIC),
    ])
    x_train=preprocessor.fit_transform(train[FEATURES])
    x_valid=preprocessor.transform(valid[FEATURES])
    model=XGBRegressor(n_estimators=n_estimators,max_depth=10,learning_rate=0.02,
        subsample=0.8,colsample_bytree=0.8,random_state=42,tree_method="hist",n_jobs=2)
    model.fit(x_train,train[TARGET])
    prediction=np.clip(model.predict(x_valid),0,100)
    metrics={
        "train_rows":len(train),"validation_rows":len(valid),
        "cutoff":str(pd.Timestamp(cutoff)),
        "mae":float(mean_absolute_error(valid[TARGET],prediction)),
        "r2":float(r2_score(valid[TARGET],prediction)),
    }
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    joblib.dump({"model":model,"preprocessor":preprocessor,"fit_columns":FEATURES.copy(),
                 "metrics":metrics},output)
    return metrics

def predict_congestion(artifact:dict,row:dict)->float:
    columns=artifact["fit_columns"]
    missing=set(columns).difference(row)
    if missing: raise ValueError(f"예측 입력 누락: {', '.join(sorted(missing))}")
    x=pd.DataFrame([row],columns=columns)
    for field in columns:
        if field=="주차장명":
            x[field]=x[field].astype(str).str.strip().str.lower()
        else:
            x[field]=pd.to_numeric(x[field],errors="raise")
            if not np.isfinite(x[field].iloc[0]): raise ValueError(f"유효하지 않은 입력: {field}")
    processed=artifact["preprocessor"].transform(x)
    return float(np.clip(artifact["model"].predict(processed)[0],0,100))

def main()->None:
    parser=argparse.ArgumentParser(description="주차장 혼잡도 예측 모델 학습")
    parser.add_argument("csv",help="실측 혼잡도 컬럼이 있는 CSV")
    parser.add_argument("--model",default="artifacts/parking_model.joblib")
    args=parser.parse_args()
    df=pd.read_csv(args.csv,encoding="utf-8-sig")
    print(train_model(df,args.model))

if __name__=="__main__": main()

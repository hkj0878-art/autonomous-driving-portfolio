"""시간별 주차 데이터의 과거 관측값으로 다음 시간 혼잡도 예측 특징을 생성한다."""
from __future__ import annotations
import pandas as pd

TARGET = "혼잡도"
CATEGORICAL = ["주차장명"]
NUMERIC = ["주차면수", "시간", "요일", "전시간_입차대수", "전시간_출차대수",
           "지난주_혼잡도", "지지난주_혼잡도", "지지지난주_혼잡도"]
FEATURES = CATEGORICAL + NUMERIC
REQUIRED = {"주차장명", "일시", "주차면수", "입차대수", "출차대수", TARGET}

def adapt_csv(raw: pd.DataFrame) -> pd.DataFrame:
    """원본 CSV와 정리본의 컬럼명 차이를 맞춘다. 실측 혼잡도는 자동 생성하지 않는다."""
    return raw.rename(columns={k: v for k, v in {"시간대":"일시", "혼잡도(%)":"혼잡도"}.items() if k in raw.columns})

def prepare_features(raw: pd.DataFrame) -> pd.DataFrame:
    missing = REQUIRED.difference(raw.columns)
    if missing:
        raise ValueError(f"필수 컬럼 누락: {', '.join(sorted(missing))}")
    df = raw.copy()
    df["일시"] = pd.to_datetime(df["일시"], errors="raise")
    if df["일시"].isna().any():
        raise ValueError("일시 컬럼에 누락된 값이 있습니다")
    df["주차장명"] = df["주차장명"].astype(str).str.strip().str.lower()
    for col in ["주차면수", "입차대수", "출차대수", TARGET]:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    if (df["주차면수"].dropna() <= 0).any():
        raise ValueError("주차면수는 0보다 커야 합니다")
    if ((df[TARGET].dropna() < 0) | (df[TARGET].dropna() > 100)).any():
        raise ValueError("혼잡도는 0~100 범위여야 합니다")
    if df.duplicated(["주차장명","일시"]).any():
        raise ValueError("같은 주차장/시각에 중복 레코드가 있습니다")
    df = df.sort_values(["주차장명","일시"]).reset_index(drop=True)
    df["시간"] = df["일시"].dt.hour
    df["요일"] = df["일시"].dt.dayofweek
    hist = df[["주차장명","일시","입차대수","출차대수",TARGET]]
    for offset, pairs in [
        (pd.Timedelta(hours=1), {"입차대수":"전시간_입차대수","출차대수":"전시간_출차대수"}),
        (pd.Timedelta(days=7), {TARGET:"지난주_혼잡도"}),
        (pd.Timedelta(days=14), {TARGET:"지지난주_혼잡도"}),
        (pd.Timedelta(days=21), {TARGET:"지지지난주_혼잡도"}),
    ]:
        past = hist[["주차장명","일시",*pairs.keys()]].rename(columns=pairs).copy()
        past["일시"] += offset
        df = df.merge(past,on=["주차장명","일시"],how="left",validate="one_to_one")
    return df.dropna(subset=FEATURES+[TARGET]).reset_index(drop=True)

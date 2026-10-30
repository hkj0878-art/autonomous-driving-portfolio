"""실행 예제에 사용할 가상 주차장 데이터."""
from __future__ import annotations
from math import pi,sin
import pandas as pd

def make_demo(days:int=45)->pd.DataFrame:
    if days<22: raise ValueError("21일 lag와 평가 분할을 위해 22일 이상 필요합니다")
    timestamps=pd.date_range("2025-01-01",periods=days*24,freq="h")
    rows=[]
    for index,name in enumerate(["가상주차장_A","가상주차장_B","가상주차장_C"]):
        capacity=100+30*index
        for t in timestamps:
            congestion=max(0,min(100,45+12*sin(2*pi*t.hour/24)
                       +5*sin(2*pi*t.dayofweek/7)+index*3))
            rows.append({"주차장명":name,"일시":t,"주차면수":capacity,
                "입차대수":max(0,15+5*sin(2*pi*t.hour/24)),
                "출차대수":max(0,12+3*sin(2*pi*(t.hour-6)/24)),
                "혼잡도":congestion})
    return pd.DataFrame(rows)

if __name__=="__main__":
    import argparse
    ap=argparse.ArgumentParser()
    ap.add_argument("--output",default="demo.csv")
    args=ap.parse_args()
    make_demo().to_csv(args.output,index=False,encoding="utf-8-sig")
    print(args.output)

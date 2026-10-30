"""주차장 혼잡도, 거리, 요금, 리뷰를 이용한 추천 점수 계산."""
from __future__ import annotations
from math import asin, ceil, cos, isfinite, radians, sin, sqrt

# 2025년 Jongp/ai/src/score.py에 기록된 네 가지 가중치.
WEIGHTS = {
    "혼잡도우선":(0.80,0.10,0.05,0.05),
    "거리우선":(0.50,0.30,0.10,0.10),
    "요금우선":(0.50,0.10,0.30,0.10),
    "리뷰우선":(0.50,0.10,0.10,0.30),
}

def haversine_km(lat1:float,lon1:float,lat2:float,lon2:float)->float:
    p1,p2=radians(lat1),radians(lat2)
    dp,dl=radians(lat2-lat1),radians(lon2-lon1)
    a=sin(dp/2)**2+cos(p1)*cos(p2)*sin(dl/2)**2
    return 6371.0*2*asin(min(1.0,sqrt(a)))

def parking_fee(duration_min:float,base_fee:float,base_minutes:float,
                extra_fee:float,extra_minutes:float=5,daily_cap:float|None=None)->float:
    if duration_min<0 or min(base_fee,base_minutes,extra_fee)<0 or extra_minutes<=0:
        raise ValueError("요금/시간 입력을 확인하세요")
    fee=base_fee+ceil(max(0,duration_min-base_minutes)/extra_minutes)*extra_fee
    if daily_cap is not None and daily_cap>0: fee=min(fee,daily_cap)
    return float(fee)

def _favorable(value:float,low:float,high:float)->float:
    return 50.0 if abs(high-low)<1e-12 else 100.0*(high-value)/(high-low)

def rank_candidates(candidates:list[dict],weights:dict=WEIGHTS)->dict[str,list[dict]]:
    """후보 내 min-max로 거리·요금을 정규화하므로 후보 집합에 따라 점수가 변한다."""
    if not candidates: return {name:[] for name in weights}
    names=[str(c.get("p_id","")).strip() for c in candidates]
    if any(not name for name in names) or len(set(names))!=len(names):
        raise ValueError("p_id가 비었거나 중복됐습니다")
    for item in candidates:
        for field in ("congestion_pct","distance_km","fee","review"):
            if field not in item or not isfinite(float(item[field])):
                raise ValueError(f"유효하지 않은 {field} 값")
        if not 0<=float(item["congestion_pct"])<=100 or not 0<=float(item["review"])<=5:
            raise ValueError("혼잡도는 0~100, 리뷰는 0~5 범위여야 합니다")
        if float(item["distance_km"])<0 or float(item["fee"])<0:
            raise ValueError("거리·요금은 음수가 될 수 없습니다")
    distances=[float(c["distance_km"]) for c in candidates]
    fees=[float(c["fee"]) for c in candidates]
    results={}
    for scenario,w in weights.items():
        if len(w)!=4 or abs(sum(w)-1.)>1e-9 or min(w)<0:
            raise ValueError(f"가중치가 잘못됐습니다: {scenario}")
        rows=[]
        for c in candidates:
            scores=(100-float(c["congestion_pct"]),
                _favorable(float(c["distance_km"]),min(distances),max(distances)),
                _favorable(float(c["fee"]),min(fees),max(fees)),
                float(c["review"])*20)
            rows.append({"p_id":c["p_id"],"score":round(sum(a*b for a,b in zip(w,scores)),2)})
        results[scenario]=sorted(rows,key=lambda r:(-r["score"],r["p_id"]))
    return results

"""표준입력 JSON을 받는 예측·추천 명령줄 도구."""
import argparse
import json
import sys
import joblib
from .train import predict_congestion
from .recommender import rank_candidates

def main():
    parser=argparse.ArgumentParser()
    sub=parser.add_subparsers(dest="command",required=True)
    p=sub.add_parser("predict")
    p.add_argument("--model",default="artifacts/parking_model.joblib")
    sub.add_parser("rank")
    args=parser.parse_args()
    payload=json.load(sys.stdin)
    if args.command=="predict":
        artifact=joblib.load(args.model)
        result={"predicted_congestion_pct":round(predict_congestion(artifact,payload),2)}
    else:
        result=rank_candidates(payload["candidates"])
    json.dump(result,sys.stdout,ensure_ascii=False,indent=2)
    print()

if __name__=="__main__": main()

# v1 잘못된 extrinsicRPY 결과 보존

2026-10-26 CODEX-REVISION TASK-03+04에 따라 기존 results의 모든 항목을 삭제 없이 이 디렉터리로 이동했다. extrinsicRot=R.T, extrinsicRPY=R.T 설정은 중력축 18.5687° 불일치를 일으켰다. 승인된 v2는 extrinsicRot=R.T를 유지하고 extrinsicRPY=R로 정정한다.

하위 run_meta.json/run.log의 절대 경로는 실행 당시 경로를 그대로 보존한다. data_report 및 split 진단도 함께 보존했으며 data/lio_bag{,_even,_odd}는 변경하지 않았다. 이 자료는 디버깅 사례용이며 v2 결과와 혼합하지 않는다.

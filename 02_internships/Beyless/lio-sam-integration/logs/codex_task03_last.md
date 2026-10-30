TASK-03 구현과 7개 실험 실행을 마쳤지만, **전체 완료 기준은 미충족**입니다.

- ON/OFF 6회 통과. 종점거리 평균 ON **0.82 m**, OFF **10.80 m**.
- `even_loop_on`은 재시도에서도 발산·루프 0회. 실패 결과 보존.
- pytest **25개 통과**, TBB 크래시 없음.
- sudo·원본 bag 쓰기·LIO-SAM 소스 수정 없음. save_map 경로 보호 적용.

[실험 결과표](/home/hong/SLAM/results/experiments_summary.md) · [작업·진단 기록](/home/hong/SLAM/logs/task03_04.md)

중력축 18.57° 불일치를 확인했습니다. 다음 검증에는 `extrinsicRPY`만 R로 바꾸는 것이 타당하지만, TASKS.md의 명시값 변경이므로 확인이 필요합니다.
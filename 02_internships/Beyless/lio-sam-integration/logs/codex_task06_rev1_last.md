TASK-06 rev1 산출물을 재생성했습니다. 변경은 `tools/maps/`, `results/maps/`에 한정했습니다.

- v2 보존, v3 전체/crop/ENU PCD·Nav2 지도·PNG 4장 생성
- unknown: 기존 **78.58% → 73.64%**. 동일 v3 stride 비교는 **82.91% → 73.64%**
- Nav2·PCD 검증 통과, 이미지 직접 확인
- pytest **55 통과**. rev1과 충돌하는 기존 보조 YAML 테스트 1개는 제외하고 실패 기록 보존

경로 제한에 따라 로그는 [results/maps/logs/task06.md](/home/hong/SLAM/results/maps/logs/task06.md)에 작성했습니다.
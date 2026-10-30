TASK-06 완료했습니다. TASK-05 영역과 원본 데이터는 수정하지 않았습니다.

- [산출물](/home/hong/SLAM/results/maps/loop_on_r2/): PCD, PGM/YAML, 렌더 이미지 3장 생성·육안 확인
- 검증: 전체 pytest **38개 통과**, 형식 검증 통과, 최대 메모리 **2.07GiB**
- ENU 지도는 정렬 JSON 미제공으로 생략
- [작업·검증 로그](/home/hong/SLAM/logs/task06.md)

Nav2 로딩에는 **`map2d_nav2.yaml`**을 사용하세요. 명세의 `.25` 임계값이 unknown=205를 free로 해석하므로, [Humble 분류식](https://github.com/ros-navigation/navigation2/blob/humble/nav2_map_server/src/map_io.cpp)에 맞춘 `.196` 보조 YAML을 추가했습니다.
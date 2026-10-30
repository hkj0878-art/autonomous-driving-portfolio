# TASK-06 완료 기록 — 2026-10-26

작업 범위: `tools/maps/`, `tools/tests/test_maps.py`, `results/maps/loop_on_r2/`,
이 로그. `tools/eval/`, `results/eval/`, 다른 태스크 코드와 TASKS.md는 수정하지 않았다.
`data/`와 `results/<run>/` 원본은 읽기 전용으로 사용했다. sudo/패키지 설치 없음.
모든 Python 명령은 ROS Humble을 먼저 source하고 지정된 venv로 실행했다.

## 재현 명령

```bash
source /opt/ros/humble/setup.bash
/usr/bin/time -v /home/hong/SLAM/.venv/bin/python tools/maps/export_maps.py --run loop_on_r2
/home/hong/SLAM/.venv/bin/python tools/maps/verify_maps.py results/maps/loop_on_r2
/home/hong/SLAM/.venv/bin/python -m pytest tools/tests -q
```

다른 run은 `--run`으로 지정한다. bag을 달리 쓰는 even run에는 `--bag data/lio_bag_even`을 지정한다.
`--align-json` 형식과 좌표변환 규약은 `tools/maps/README.md`에 있다.

## 산출물과 수치

출력 디렉터리: `results/maps/loop_on_r2/`

- `map_full.pcd`: GlobalMap.pcd 원본 바이트 복사, 1,014,229점. intensity 보존.
- `map_0p10.pcd`: 0.1m voxel centroid XYZ, 658,453점. intensity는 생략.
- `map_0p10_enu.pcd`: 선택 산출물. 정렬 JSON이 제공되지 않아 생략했고 stats에 기록.
- `map_top.png`, `map_oblique.png`: 높이 컬러맵 + 최종 keyframe 궤적, 200dpi.
- `map2d.pgm`, `map2d.yaml`: 요청한 P5/0·254·205, 0.05m, trinary, occupied_thresh=.65/free_thresh=.25.
- `map2d_nav2.yaml`: 아래 임계값 충돌에 대한 실사용 보조 YAML.
- `map2d.png`, `map2d_stats.json`, `validation.json`.

2D 크기: 4,835 × 5,112 셀, 241.75 × 255.60m.
origin: [-115.05, -158.25, 0]. 원거리 반사점을 포함하여 영역을 자르지 않았다.
occupied 130,892 / free 1,091,908 / unknown 23,493,720.
지면 제거 후 장애물 점 209,076개. 2,178 스캔 중 매 10번째를 선택해 211개 사용,
최종 keyframe 시간 구간 밖 7개 제외. 54,113개 레이 사용.

최종 전체 생성 명령 exit=0, wall time 18.26s, peak RSS 2,172,980KiB (약 2.07GiB), swap=0.
동시 TASK-05 작업을 고려한 4GB 제한 이내이다.

## 구현 방법과 한계

2m XY 셀별 하위 10 percentile을 로컬 지면으로 사용한다. 빈 셀은 가까운 관측 셀을
참조한다. 지면 위 0.3–2.0m를 장애물로 분류한다. 모든 값은 Config/상수에 정의했다.
최적화된 최종 keyframe의 위치 선형보간 + 자세 SLERP로 bag 스캔을 지도 좌표에 투영한다.
스캔별 deskew는 생략했다. 0.5도 방위각 bin별 가까운 장애물 또는 장애물이 없을 때 먼
지면 반사점을 선택한다. 최대 60m 수평거리, Bresenham + log-odds로 자유공간을 만든다.
전역 장애물 prior에서 레이를 중단해 벽 뒤가 free로 지워지는 것을 방지한다.
자유공간 증거가 불충분한 셀은 unknown으로 둔다.

국소 지면 추정은 수목/건물만 있는 셀, 경사와 희소 관측에서 오차가 생긴다.
2m 이상 장애물은 2D 지도에서 제외하고 footprint inflation은 하지 않는다.
원거리 지도 영역의 지면 추정 높이 범위는 -6.20~8.34m로, 전부 도로 지면을 뜻하지 않는다.
렌더는 OpenGL 없이 matplotlib Agg 3D scatter 대체 경로를 사용했다(최대 16만 점).
ROS system-site-packages의 mpl_toolkits와 사용자 matplotlib 버전이 달라 3D import가
실패하는 현상은 실행 프로세스 안에서 해당 matplotlib와 동일한 toolkit 경로를 선택해 해결했다.
사선 뷰는 높이 축을 확대해 표시하며 제목에 명시했다. 궤적은 가려지지 않게 겹쳐 그렸다.

## Nav2 임계값의 명세 충돌 및 해결

[Nav2 Humble map_io.cpp](https://github.com/ros-navigation/navigation2/blob/humble/nav2_map_server/src/map_io.cpp)의
PGM trinary 로딩은 `occ = 1 - gray/255`로 계산하고 `occ < free_thresh`를 free로 분류한다.
따라서 unknown=205는 약 0.196078이며, 명세 free_thresh=.25에서는 **free로 로드된다**.

명세대로 `map2d.yaml`을 제공하고, 동일 PGM을 참조하되 free_thresh만 **.196**인
`map2d_nav2.yaml`을 추가했다. 실제 Nav2에서는 **map2d_nav2.yaml**을 사용해야 한다.
기본 파일을 unknown 의미까지 정상이라고 판정하지 않았으며 `validation.json`에도
`requested_yaml_preserves_unknown: false`로 기록했다. 보조 YAML은 0/254/205를 각각
occupied/free/unknown으로 복원하는지 별도 검증했다.

시스템에 nav2_map_server가 없어 실제 노드 로딩 시험은 수행하지 않았다.
직접 작성한 검증기 + 공식 Humble 분류식에 의한 의미 검증이며, 노드 통합시험과 구분한다.

## 완료 기준 직접 검증

- [x] 기본 run의 모든 필수 파일 생성. ENU는 명세의 선택 출력 조건에 따라 생략.
- [x] PGM P5 헤더/크기/최댓값/데이터 길이/값 집합 검사 PASS.
- [x] YAML 모든 필드, 해상도, origin 격자 정렬, 크기 m 및 셀 수 일치 PASS.
- [x] PGM y축 반전 단위테스트 PASS. 실제 최종 궤적 좌표를 PGM에 역투영한 자유공간 비율 97.31%.
- [x] 원본 복사 바이트 일치, 원본 입력 SHA256 재확인, voxel PCD 점 수 일치 PASS.
- [x] PNG 해상도/비어 있지 않음 검사 PASS. top/oblique/2D 이미지를 직접 열어 육안 확인.
  사각 주행로, 도로 양쪽 선형 구조와 건물 윤곽 식별 가능. 궤적은 관측 도로 위에 있고 축 뒤집힘 없음.
  최종 사선 뷰에서도 주황 궤적이 보임을 재확인했다.
- [x] 지면 분리, Bresenham 가림/종점/8방향, YAML/PGM, 손상 파일 거부, Nav2 unknown 의미 검증 테스트 PASS.
- [x] `pytest tools/tests -q`: **38 passed in 0.89s** (TASK-06 테스트 13개 포함).
- [x] `verify_maps.py`: exit=0, valid=true. Nav2 보조 YAML semantics PASS.

원본 SHA256:

- GlobalMap.pcd: `e59e1ebcfd3814ba084232a8247ea2fd73e9c275b58739eb485cfa007c0a724c`
- keyframes_final_tum.txt: `9f0e6a4503a2d7d60e4326876e8638059edff469e9c724fa9d5e5808ebf40c85`

TASK-06 완료. TASK-05의 평가/정렬 수치 산출은 수행하지 않았다.

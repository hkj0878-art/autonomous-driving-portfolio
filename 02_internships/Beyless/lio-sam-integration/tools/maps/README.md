# TASK-06 rev1 지도 내보내기

프로젝트 루트에서 `.venv` Python과 `env.sh`를 사용한다. 입력 bag과 run은
읽기 전용이며 출력은 `results/maps/<run>/`이다.

```bash
source env.sh
PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 \
  .venv/bin/python tools/maps/export_maps.py --run loop_on_r2 \
  --align-json results/eval/loop_on_r2/align_se3.json --compare-stride10
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=2 \
  .venv/bin/python tools/maps/verify_maps.py results/maps/loop_on_r2
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q \
  tools/tests tools/maps/test_revision.py -k 'not spec_threshold_conflict' \
  -o cache_dir=results/maps/.pytest_cache
```

기본값은 `--scan-stride 2`, 여백 30 m, 해상도 0.05 m이다.
`--compare-stride10`은 같은 v3 입력/알고리즘으로 stride 10 기준값도 계산한다.
`--run`, `--bag`으로 입력을 바꿀 수 있다. even run은 `data/lio_bag_even`을 쓴다.
모든 설정과 입력 SHA256, 렌더러 결과, 주행 영역 통계는 `map2d_stats.json`에 남는다.

| 산출물 | 내용 |
|---|---|
| map_full.pcd | intensity를 포함한 원본 바이트 복사 |
| map_0p10.pcd | 전체 0.1 m voxel XYZ centroid, intensity 제외 |
| map_crop.pcd | 궤적 bounding box +30 m 영역의 원본 crop |
| map_crop_0p10.pcd | 동일 영역의 0.1 m voxel crop |
| map_0p10_enu.pcd, map_crop_0p10_enu.pcd | SE3 정렬을 적용한 전체/crop voxel 지도 |
| map2d.pgm, map2d.yaml | Nav2 trinary 지도, 유일한 YAML |
| map_top.png, map_oblique.png, map2d.png | crop 영역 높이/사선/occupancy 뷰 및 궤적 |
| map_loop_overlap.png | loop_on_r2 / loop_off_r2 공통 ENU 영역 확대 비교 |

정렬 JSON은 `R`(3x3), `t`(3,)를 포함한다. `p_enu = R @ p_map + t`이고
스케일이나 lever arm은 적용하지 않는다. 정렬 파일이 없으면 선택 ENU PCD는
생략하고 이유를 기록한다. 기본 PCD와 2D 지도, top/oblique는 map frame이다.
ON/OFF 비교 그림은 각각의 `results/eval/<run>/align_se3.json`을 읽어 ENU로 맞춘다.
대표 run의 비교 그림에는 loop_off_r2 입력과 정렬 파일도 필요하다.

지면은 2 m XY 셀별 하위 10 percentile이다. 빈 셀은 최근접 관측 셀을 사용한다.
장애물은 지면 위 0.3–2.0 m이다. bag 원본 스캔에 최종 keyframe 선형/SLERP
보간 자세를 적용하고, 동일 높이 슬라이스에서 0.5도 방위각별 최근접 점을
레이 끝점으로 쓴다. 수평거리 60 m 이내의 레이로 log-odds를 갱신하며, 전역
장애물 prior에 닿으면 중단한다. 포인트별 deskew는 적용하지 않는다.
미관측/불확실 셀은 unknown이며, 지면 추정 오차·가림·수평 슬라이스의 한계가 있다.

PGM은 P5, occupied=0 / free=254 / unknown=205이다. 첫 행은 지도 위쪽,
YAML origin은 왼쪽 아래 모서리이다. `occupied_thresh=0.65`,
`free_thresh=0.196`으로 gray 205가 unknown을 유지한다
(`1-205/255 = 0.196078`). 보조 YAML은 생성하지 않는다.

Open3D OffscreenRenderer를 별도 프로세스에서 먼저 시도하고, 실패하면 matplotlib
3D scatter로 대체한다. 렌더 voxel은 0.2 m, matplotlib 점 수는 최대 16만이다.
주행 영역 unknown 비율은 궤적 polyline의 30 m 버퍼에 속하는 셀 중심으로 계산한다.
궤적 샘플 간격은 해상도/2 이하이고 버퍼 경계의 최대 오차는 해상도/4이다.

`verify_maps.py`는 PGM/YAML, Nav2 셀 의미, 좌표/크기, 원본 및 정렬 SHA256,
PCD crop/ENU 좌표, 주행 영역 통계, 그림 크기와 비어 있지 않음을 검증한다.
실제 Nav2 노드 기동 대신 trinary 분류식을 검증한다.

위 pytest 명령에서 제외한 기존 테스트 1개는 rev1에서 제거한 보조 YAML을 요구한다.
수정 허용 범위 밖인 `tools/tests/`는 유지하며, `tools/maps/test_revision.py`에
단일 YAML의 새 요구사항을 검증하는 테스트를 두었다. 상세 결과와 원래 테스트의
실패 기록은 `results/maps/logs/task06.md`에 있다. 해당 로그는 사용자의 경로
제한에 따라 `logs/task06.md` 대신 작성했다.

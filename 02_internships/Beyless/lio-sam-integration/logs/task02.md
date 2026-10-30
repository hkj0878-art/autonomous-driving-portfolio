# TASK-02 완료 및 직접 검증

TASK-02만 수행했다. third_party, ws, env.sh, launch 및 TASKS.md를 수정하지 않았고 sudo를 사용하지 않았다. data/synced_bag은 rosbag2_py SequentialReader의 READ_ONLY 모드로만 열었다. TASK-01 빌드 결과나 overlay에 의존하지 않는다.

## 산출물

- tools/preprocess_bag.py: --in / --out / --overwrite CLI, rosbag2_py 스트리밍, 원본 및 하위/상위 경로 덮어쓰기 방지, 임시 bag 성공 후 교체, 필드/토픽/시각 검사, 실패 원인 리포트.
- tools/tests/test_preprocess.py: 합성 PointCloud2 단위 테스트 및 빈 ring 리포트 회귀 테스트.
- tools/verify_preprocess.py: 출력 bag 전체 재독립검증 및 원본 payload/샘플 비교.
- data/lio_bag/: sqlite3 bag, 약 1.7 GiB, 16,360 메시지.
- results/data_report/: data_report.md, gnss_traj.png, timing.png, statistics.json, verification.json.

## 완료 기준

- [x] ros2 bag info에서 /points_raw=2178, /imu_raw=10884, /gps/fix=3298. 원본 3토픽 수와 각각 일치한다. logs/task02_bag_info.txt 참조.
- [x] 첫/중간/마지막 스캔(index 0/1089/2177) 검사. 상대 time, 원본 좌표/intensity/ring 및 순서, header를 원본으로부터 독립 계산해 정확히 비교했다. 전체 2178스캔에 대해서도 is_dense=True, 필드 레이아웃, point_step=22, NaN/inf 없음, range≥0.5 m, 원점 없음, ring<128, time 범위를 검사했다.
- [x] 전체 16,360 메시지의 bag 기록시각과 header stamp 차이 **0 ns**.
- [x] pytest **11 passed**. 시간 변환, 무효점 제거, 시간 원점 유지, 필드 레이아웃/순서, big endian, organized cloud row padding, 잘못된 입력, 출력 경로 보호, 빈 ring 통계를 검증했다.
- [x] 데이터 리포트와 그림 2장 생성 및 이미지 확인.
- [x] IMU/GNSS는 원본과 직렬화 payload SHA-256이 각각 일치하며 IMU frame_id는 imu_link다. 해시는 verification.json에 기록했다.
- [x] 변환 최대 RSS **157,228 KiB (153.5 MiB)**, 검증 최대 RSS 79,604 KiB. 1 GB 이하. 변환 wall time 55.11초, 종료 코드 0. logs/task02_preprocess.log 및 logs/task02_verify.log 참조.

| 샘플 index | 입력점 | 출력점 | time 최소 s | time 최대 s |
|---:|---:|---:|---:|---:|
| 0 | 78336 | 26489 | 0.000000000 | 0.025435209 |
| 1089 | 78336 | 39133 | 0.000000000 | 0.025434017 |
| 2177 | 78336 | 25720 | 0.000000000 | 0.025434971 |

## 주요 데이터 결과

- 입력 170,108,160점 → 유효 84,181,256점. 원점 85,926,904점 제거(50.5131%); 비유한점 및 0.5 m 미만 비원점은 이 bag에서는 0점이다.
- offset 평균 169765152.463684797 s, float64 계산상 표준편차/최대편차/범위 모두 0 s. epoch 크기에서의 float64 정밀도 한계 내 일치이며, 물리적인 무오차를 의미하지 않는다. 1 ms 중단 기준을 통과했다.
- 전체 출력 time 범위 [0.000000000, 0.025436163] s.
- ring별 통계는 data_report.md의 128행 표 참조. 전체 구간에서 유효점이 전혀 없는 ring은 119,120,121,123,124,125,127이며 ring 번호를 재매핑하지 않았다.
- 대표 방위각 간격 약 0.200083°, 360/해상도≈1799.26. 명목상 약 0.2°/Horizon_SCAN 약 1800 참고값이며 TASK-03 설정은 변경하지 않았다. 실제 유효점 개수와 360° 투영 격자 수는 다르다.
- 시간/방위각 상관계수 중앙값 −0.999989, 링 내 시간 역전 0: 시간 증가에 따라 방위각이 거의 선형으로 감소하는 전방 스캔 패턴이다.
- GNSS WGS84 ECEF→ENU, 첫 fix 원점. 누적거리 238.679 m, 시작–끝 거리 0.473 m.

## 재현 명령

프로젝트 루트 /home/hong/SLAM에서 실행한다. 기존 출력이 있으면 명시적으로 --overwrite를 추가한다.

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m pytest -q tools/tests/test_preprocess.py
/usr/bin/time -v /usr/bin/python3 tools/preprocess_bag.py --in data/synced_bag --out data/lio_bag
ros2 bag info data/lio_bag
/usr/bin/python3 tools/verify_preprocess.py
```

## 이슈 및 해결

첫 변환 시 유효점이 없는 ring의 해상도를 숫자로 출력하면서 리포트 작성이 실패했다. 빈 통계를 N/A로 표시하고 대표 해상도에서 제외하도록 수정했으며 회귀 테스트를 추가했다. 임시 bag을 정리한 뒤 전체 변환을 재실행해 성공했다. 이후 리포트에 빈 ring 및 스캔 패턴 요약을 보강했다.

시스템/사용자 Matplotlib 중복 설치로 Axes3D 로드 경고 1건이 발생한다. 사용한 2D Agg 렌더링은 성공했고 두 PNG를 직접 확인했다. 시스템 패키지나 환경은 수정하지 않았다.

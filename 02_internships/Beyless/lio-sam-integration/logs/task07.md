# TASK-07 완료 및 직접 검증

TASK-07을 수행했다. Python 실행과 pytest에는 `/home/hong/SLAM/.venv/bin/python`을 사용했다. sudo, 시스템 패키지 설치, bag 재생 및 기존 평가 재계산은 하지 않았다. data/ 및 기존 results/에는 쓰지 않았다.

## 산출물

- `report/REPORT.md`: 한국어 요약, 센서, Mermaid 아키텍처, 시각·외부 회전·RPY·RPE·lever 관측성·저장 가드·루프 판정 이슈, 궤적/맵 품질, 지도, 위치추정, 한계 및 다음 단계.
- `report/figs/`: 원본에서 그대로 복사한 9장. `report/figures.json`에 원본 경로와 SHA256을 기록했다. 원본 경로는 보고서 caption에도 표기했다.
- `report/metrics_headline.json`: 기존 결과에서 추출한 78개 값, 표시 문자열, 원본 경로, JSON/CSV/정규식 selector 및 원본 SHA256. 평균·표준편차·개선율을 새로 계산하지 않고 이미 집계된 파일에서 읽는다.
- `tools/report/collect_metrics.py`, `sources.json`, `report/REPORT.template.md`: 출처 명세와 수치 placeholder로 보고서를 재생성한다. `--check`는 저장 값·출처·보고서의 일치를 읽기 전용으로 검증한다.
- `tools/report/verify_report.py`: 출처 대조, 바인딩되지 않은 숫자 검사, 문서/각주/그림 링크 및 복사 해시, README shell 구문과 진입 파일 존재, 보호 파일 상태를 검증한다.
- `README.md`: 영어 개요, 환경/로컬 빌드, 전처리→실험→평가→지도→위치추정→보고서 순서, 예상 시간, 결과 위치 및 검증 명령. 재현 명령은 출력이 없는 새 작업 복사본에서만 실행하도록 명시했다.
- `report/requirements.txt`: venv 로컬 배포판과 실제 사용한 inherited pytest/PyYAML/Pillow 버전.
- `tools/tests/test_report.py`: 값/출처 일치, 누락·중복 selector 거부, 원본 변화 감지, 보고서 숫자 변조 거부, 링크 및 README dry-run 검증.
- `TASKS.md`: TASK-07 목록과 완료 기준을 완료 표시했다.

## 출처 및 해석

현재 `results/data_report/`는 없으므로 보존된 `results/archive_v1_wrong_rpy/data_report/`를 사용했다. 지면 법선 0.55°와 과거 RPE 17% 리뷰는 독립 원시 결과 파일이 없어 `TASKS.md`의 설계/리뷰 기록에서 추출했고 보고서에서 이 차이를 밝혔다. 최신 궤적 평가에는 `results/eval/summary.md` rev2와 CSV를 사용했다. 구버전 평가 README의 접선·lever·루프 판정 방법은 최신 결과로 대체된 설명임을 명시했다.

루프 ON/OFF 평균 ATE 개선 21.4%는 n=3 반복 표준편차보다 작아 통계적 유의성을 주장하지 않았다. 동일 주행 even/odd 및 in-sample lever 적합의 낙관성, 재구성/위치추정의 deskew 부재, 평균 실시간 계수와 매 스캔 deadline의 차이, 처리 스캔 1089개와 GT 매칭 1088개의 차이를 공개했다. v1 실패 파일럿과 v2 배치 예시는 통제된 동일 반복 비교가 아니며 발산의 단독 원인을 확정하지 않았다.

## 전체 테스트에서 확인한 기존 문제와 수정

첫 전체 실행에서 `tools/eval/test_revision2.py::test_noise_floor_is_reproducible_and_nonzero`가 동일 seed 결과의 부동소수점 끝자리 차이로 실패했다. 예: 표준편차 0.017142509328441403 대 0.017142509328441406. 전체 dict의 정확한 동등성 검사를 수치 필드의 `pytest.approx(rel=1e-12, abs=1e-12)`로 바꾸고 메타데이터는 정확히 비교한다. 양수·clean GT·segment 검사는 유지했다. 평가 알고리즘이나 기존 수치는 수정하지 않았다.

## 직접 검증 결과

환경:

```bash
source env.sh
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
.venv/bin/python tools/report/collect_metrics.py --check
.venv/bin/python tools/report/verify_report.py --dry-run-readme
.venv/bin/python -m pytest -q
```

- [x] 기존 결과에서 읽은 값 및 provenance 78개 일치. 보고서가 동일 출처로 생성한 내용과 정확히 일치하고, 본문에 바인딩되지 않은 숫자가 없다. 고정된 표기법/버전/경로는 숫자 사실 검사에서 구별한다.
- [x] 문서 링크 110개, 모든 각주 및 그림 9장 통과. 복사본과 원본 SHA256 일치. `logs/task07_figure_contact.jpg`로 9장을 직접 열어 궤적·지도·히트맵·범례가 표시되는지 확인했다.
- [x] README 활성 명령 28개의 bash 구문과 Python script/module, shell script 및 전제 파일 존재 확인. `logs/task07_verification.json`에 각 명령과 해석한 진입 파일을 기록했다. dry-run은 명령을 실행하거나 결과를 생성하지 않으며 실제 재현 실행 성공을 뜻하지 않는다.
- [x] 전체 프로젝트 pytest: **77 passed in 1.27s, 0 failed**. 제외 필터 없이 `python -m pytest -q`를 실행했다. `pytest.ini`에 따른 tools/ 전체 검색이며 vendored upstream은 별도 환경이다. 원문: `logs/task07_pytest.log`.
- [x] `logs/task07_protected_before.json`과 작업 후 data/results 파일 목록·크기·mtime_ns가 1,132개 모두 동일하다. 수치 원본 및 그림은 추가로 SHA256을 대조했다. 보호 파일의 접근시각은 읽기에 따라 바뀔 수 있어 비교하지 않는다.

완료 기준 네 항목을 직접 검증했다. 네이티브 재빌드, 전체 bag 재생, 지도 재생성과 실제 Nav2 노드 로딩은 이번 문서 작업에서 실행하지 않았다.

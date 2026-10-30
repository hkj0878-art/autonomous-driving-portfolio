# TASK-05 revision / v3 evaluation

Status: **COMPLETE**, final canonical 7/7 PASS after two preserved even failures and a startup-readiness retry. All evaluation artifacts and independent audit PASS.

## Approved decisions and preservation

The bottom [CODEX-REVISION] TASK-05 in TASKS.md authorizes horizontal-only
GNSS sigma filtering, 2D and 3D ATE, marker endpoint recording, and v3 replay.
Canonical seven runs, both pilots and the fitness trial were moved intact to
results/archive_v3_preloop_pairs/ with ARCHIVE_NOTE.md; no deletion.
The prior summary and verification are archived there too. Existing results/maps
and tools/maps are protected by checksums in logs/task05_protected_maps_before.json.
No sudo, system installs, upstream LIO-SAM source edits or data writes.

## Reproduction

```bash
source env.sh
# Run only into absent canonical directories; archive previous runs first.
./tools/run_experiments.sh 1.0 > logs/task05_v3_batch.log 2>&1
PYTHONDONTWRITEBYTECODE=1 /home/hong/SLAM/.venv/bin/python -m tools.eval.run_all_eval
PYTHONDONTWRITEBYTECODE=1 /home/hong/SLAM/.venv/bin/python tools/verify_experiments.py
PYTHONDONTWRITEBYTECODE=1 /home/hong/SLAM/.venv/bin/python -m tools.eval.verify_eval
PYTHONDONTWRITEBYTECODE=1 /home/hong/SLAM/.venv/bin/python -m pytest tools/tests -q
```

The wrappers and recorder now use /home/hong/SLAM/.venv/bin/python.
The batch wrapper is frozen while running and its checksum is recorded in
logs/task05_v3_wrapper.sha256. The save_map rm -r destination checks and
savePCD=false remain intact. GPS input is absent and playback excludes GNSS.
Params hashes must match archived v2 per canonical run; rate remains 1.0,
fitness 0.5. Mapping runs execute sequentially.

## Methods and approximations

- First-fix WGS84 ECEF→ENU; status>=2 and sqrt(cov[0]), sqrt(cov[4]) <=0.05 m.
  Vertical sigma is reported without the 0.05 m quality cutoff.
- GT interpolation at estimator header stamps; no extrapolation and <=0.2 s bracket.
- Own Umeyama SE3 with no scale and joint nonlinear SE3/3D body lever fit.
  Both yield 2D horizontal and 3D ATE RMSE/mean/median/max/std. Horizontal ATE
  uses aligned ENU XY; all rotations/translations are still fitted in 3D.
  Vertical lever observability is weak on this planar drive; condition numbers,
  fitted offset and >3 m warnings are retained. Lever ATE is in-sample.
- evo Umeyama+ATE RMSE cross-check must differ by <1 mm.
- Endpoint displacement-vector error; KITTI-style all-start RPE for 10/25/50/100 m,
  actual GT distance denominator. GNSS orientation uses 2 m tangent yaw and
  roll/pitch=0. Antenna/body-origin mismatch and corners limit RPE interpretation.
  Fixed an actual-data endpoint bug: the tangent's left index must be the sample
  at/before the backward distance boundary, avoiding a zero-length last tangent
  when sparse final keyframes are more than 1 m apart. Regression test added.
- Upstream mapOptmization.cpp:724-774 emits cumulative loop_edges (LINE_LIST,
  namespace loop_edges, id=1) with current then previous endpoint per edge.
  Recorder retains endpoints, marker stamp, frame, wall and simulation receive times,
  and snapshots. After save and recorder shutdown, latest snapshot endpoints are
  independently nearest-matched to final keyframes with indices, stamps and distances.
  Evaluation rejects matches >0.2 m or identical indices. GT horizontal separation
  <2 m is TP, otherwise FP. Proximity alone does not validate relative rotation.
- Final vs online compares full trajectories; online includes past corrections,
  so this does not isolate ATE immediately before/after each factor insertion.
- Map comparison: same ENU GNSS XY bounding box, 0.1 m voxel, radius 0.3 m,
  >=6 neighbors, sample covariance eigenvalue floor 1e-9 m². MME is Gaussian
  differential entropy (nats); MPV is minimum covariance eigenvalue (m²).
- Overlap maps use first/last 15 s, scan-header translation+SLERP; no point deskew
  or pose extrapolation. Weighted voxels retain mean point-relative time and
  header time. Both NN directions use a 1 m support gate, with median/p95,
  >0.2 m discrepancy proxy and support fractions. Different visibility contributes.
- Agg/200 dpi plots, ENU axes in metres and color-friendly palette.
  Even evaluation uses even scans; held-out localization remains TASK-05b.

## Direct validation

Completed; detailed evidence and results below.

## v3 first batch failure and unchanged-setting retry

The first complete seven-run batch returned exit=1: the six ON/OFF runs passed,
but even_loop_on diverged (distance 1380.694 m, max speed 175.203 m/s,
endpoint 544.697 m, 14 IMU velocity resets, no loops, no process crash).
Archived intact under results/archive_v3_failed_attempts/even_loop_on_attempt1/,
with the first batch summary and verification. Original v2 even had PASS
under the identical params hash, so this is a repeatability issue that cannot
be attributed to a config change. Overlap preprocessing used <365 MiB and had
finished before divergence warnings, but scheduling sensitivity is possible;
no causal claim is made. Retry only the failed even run, identical settings,
rate 1.0, without concurrent evaluation. No threshold changes or deletions.
The final canonical set therefore includes a disclosed retry; it must not be
presented as a first-attempt 7/7 robustness result.

Second attempt (isolated, original startup) also failed: distance 4648.733 m,
max speed 657.138 m/s, endpoint 121.831 m, no loops. Preserved as
archive_v3_failed_attempts/even_loop_on_attempt2. The runner previously
waited only for recorder discovery, not the input sensor subscribers. Added
checks for >=1 /points_raw and >=2 /imu_raw subscribers, plus rosbag play
--delay 3.0 for DDS discovery. This changes startup only; params hashes,
rate 1.0, fitness 0.5 and all sensor timestamps remain unchanged. Third
attempt uses this startup protocol. Different initial delivered scans can
change this sparse 10 Hz input's initialization; this is a hypothesis, not
a proven causal diagnosis. No concurrent evaluation in either retry.

## Final direct verification (2026-10-26)

- Original seven-run wrapper exit=1 and first even retry exit=1 are retained;
  third even attempt (readiness +3 s delay) exit=0. Final verify_experiments.py
  exit=0, seven run checks PASS, ON/OFF endpoint comparison 3/3 PASS.
  Final even: 234.713571 m, max speed 3.795928 m/s, loops=2, zero velocity resets.
- Params YAML bytes identical to archived v2 for all seven canonical runs;
  rate=1.0, fitness=0.5. Wrapper checksum unchanged throughout execution.
  Only final even has extra startup discovery waiting; no algorithm tuning.
- All 3298/3298 GNSS fixes accepted; no status/covariance/invalid rejects.
  Vertical sigma min/mean/max=0.0512/0.0536204/0.0584 m, statistics only.
- 14 metrics.json plus 14 error CSVs, summary.md/csv, group_summary.csv,
  map_quality.csv and loop_validation.csv generated. Both 2D/3D ATE alignments,
  RPE and n=3 mean±sample std are present.
- evo maximum RMSE difference=5.329070518200751e-15 m (<0.001 m).
- Nine matched loop pairs: TP=7, FP=2. FP pairs are loop_on_r2 [148,0]
  (2.509418 m) and even_loop_on [164,0] (2.303191 m).
  Maximum marker/final matching error <7e-10 m; independent GT interpolation
  recomputes every classification. OFF runs have empty pairs and zero events.
- TASK-06 representative: loop_on_r2 (median final 2D SE3+lever ATE RMSE).
  results/eval/loop_on_r2/align_se3.json has standard rigid 4x4, R/t and
  frame convention; no lever correction is baked into the map transform.
- Seven MME/MPV and bidirectional overlap tables complete. Full evaluator
  wall time 60.93 s, peak RSS 393.27 MiB; overlap precompute <365 MiB.
- All 12 final figures directly inspected via contact sheets and the corrected
  even overlay separately: finite/nonempty, ENU axes East-right/North-up,
  matching GT path orientation, units and legends visible. Fixed even overlay
  color to distinguish it from black GNSS. Overlap plots show different FOV
  visibility and distant returns; NN support fractions must accompany distances.
- Independent eval audit exit=0, overall_completion=PASS. All original tests
  retained; final pytest tools/tests: 52 passed (47 existing +5 revision tests).
  Safety tests for save_map destination protection remain passing.
- All 15 protected tools/maps and results/maps file hashes match the initial
  pre-replay snapshot. Evaluator also hashes every canonical source before/after;
  no source experiment file changed during evaluation. Bags opened READ_ONLY;
  no data writes, upstream source edits, sudo or package installs.
- See logs/task05_v3_{batch,even_retry1,even_retry2,eval,experiments_verify,
  independent_verify,pytest}.log. The inherited experiment verifier writes its
  current results as verification_v2.json; a version-labeled copy is saved as
  results/verification_v3.json. The original v2 audit is archived, not overwritten.

## Interpretation (three lines)

1. Final horizontal SE3 ATE RMSE is ON 0.718±0.059 m vs OFF 0.793±0.119 m (n=3); joint lever fits give 0.306±0.114 vs 0.385±0.223 m, but >3 m warnings in all OFF final and all online fits limit physical offset interpretation.
2. Two pairs fail the specified <2 m proximity rule; MME/MPV and overlap do not show a consistent ON advantage, so fitness=0.5 cannot be presented as uniformly beneficial or free of false positives.
3. Two preserved even failures expose initialization/repeatability risk; the successful startup-wait retry does not prove causation or robustness, and tangent-based RPE / no-deskew overlap / in-sample lever fits remain approximations.

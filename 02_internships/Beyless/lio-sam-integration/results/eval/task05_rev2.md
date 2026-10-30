# TASK-05 rev2 work log

Scope: tools/eval/ and results/eval/ only. No experiments rerun. Existing canonical v3 trajectories, maps and loop records are read-only.

Loop closure changes horizontal ATE from OFF 0.3893 ± 0.2204 m to ON 0.3059 ± 0.1141 m; OFF−ON = 0.0834 m (21.4% reduction; negative means degradation).

The mean difference is smaller than the larger group std (0.2204 m). With only three runs per group, this does not establish statistical significance or generalization to another drive.

Yaw-only RPE removes the artificial mounting-pitch contribution; 100 m translation is now consistent with sub-metre ATE. The GNSS noise floor below uses ten independent 1.4 cm perturbations and matched final-pose sampling.

Lever2D estimates are fitted on the same GT used to score ATE; agreement across repeats supports a plausible mounting offset but is not independent calibration. ON/OFF variation can also absorb trajectory distortion.

Loop consistency checks the relative position vector, not an ICP rotation or an independent ground-truth loop transform. Online odometry already contains earlier loop updates, so online/final differences do not isolate a single constraint.

Methods and limitations are documented in summary.md; verification in independent_verification.json and pytest_rev2.txt. Per the user scope, logs/task05.md is not modified.

## Direct completion checks

- Recomputed 14 trajectory metrics and all trajectory figures from saved v3 data. No SLAM experiment, bag replay or map-building experiment was rerun.
- Independent evaluator exit 0; 14/14 RPE 100 m means <2% (maximum 1.68873%); 14/14 horizontal lever lengths <3 m (maximum 1.71314 m); group lx/ly sample std independently checked. Legacy SE3 RPE and 3-DoF lever fits retained as diagnostics.
- Nine loop pairs independently checked against interpolated GT: 9 CONSISTENT(TP), 0 INCONSISTENT. The even pair at 0.49243 m is close to the 0.5 m threshold; position-only consistency does not certify rotation.
- GNSS self/noisy-GT floor: 7 runs × (4 translation + 3 yaw segment lengths), ten trials each, clean self-errors numerically zero. See rpe_noise_floor.csv.
- Twelve plots directly inspected via regenerated contact sheets: no empty charts or inverted ENU axes; overlays include the corrected antenna trajectory.
- Evaluation tests: 16 passed. Full tools suite: 59 passed, 1 failed. The outstanding test is tools/tests/test_maps.py::test_nav2_roundtrip_and_spec_threshold_conflict, which requires map2d_nav2.yaml removed by TASK-06 rev1. This is an out-of-scope old expectation; no map or tools/tests files were changed. The whole-pytest completion criterion remains unmet.
- Canonical source checksums match before/after evaluation. SE3-only map alignment and map-quality methods remain unchanged; representative is loop_on_r2. Concurrent TASK-06 directories were not included in the immutable-source check.

Commands (from /home/hong/SLAM):

```bash
source env.sh
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m tools.eval.run_all_eval --reuse-map-quality --reuse-overlap
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m tools.eval.verify_eval
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest tools/tests/test_eval.py tools/eval/test_revision2.py -p no:cacheprovider -q
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest tools -p no:cacheprovider -q
```

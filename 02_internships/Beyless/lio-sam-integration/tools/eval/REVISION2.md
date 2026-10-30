# TASK-05 rev2

Run from `/home/hong/SLAM` with the existing v3 experiment products. No ROS launch, bag replay, map rebuild or experiment rerun is involved:

```bash
source env.sh
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m tools.eval.run_all_eval --reuse-map-quality --reuse-overlap
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m tools.eval.verify_eval
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest tools -p no:cacheprovider -q
```

Writes are limited to `tools/eval/` and `results/eval/`. Regression tests therefore live in `tools/eval/test_revision2.py`, and the revision work log is `results/eval/task05_rev2.md`. `logs/task05.md`, `TASKS.md`, `tools/tests/`, `tools/maps/` and `results/maps/` are outside this revision's write scope.

`se3` and `align_se3.json` retain the original rigid alignment, including the evo crosscheck. `se3_lever` now denotes the joint horizontal two-variable lever model with zero vertical offset. Unconstrained three-variable fits are diagnostics under `se3_lever_3d_diagnostic`.

Primary `rpe` uses the same fitted antenna trajectory and alignment as primary ATE, with aligned yaw-only rotations. `rpe_se3_diagnostic` retains SE3-only RPE, exposing residual bias from comparing different sensor origins. This distinction is explicit in `summary.md`; the joint calibration is fitted in-sample, so its lower error must not be interpreted as independent calibration validation. The low-level `rpe()` function retains the 10 m yaw statistic for compatibility with existing tests; published metrics suppress it and reports include yaw at 25 m and above only.

`rpe_noise_floor.csv` contains clean-GT self comparison and ten deterministic independent isotropic 1.4 cm Gaussian perturbations per run, using synchronized final-keyframe sampling. The reference stays fixed and noisy-estimate yaw uses its own 5 m tangent. This specified perturbation is a sensitivity test, not a claim that vertical GNSS sigma is 1.4 cm.

Loop consistency uses the full 3D relative-position-vector error after unchanged SE3 alignment, strictly `<0.5 m`; horizontal GT distance is reference only. Passing does not verify relative rotation or remove GT measurement uncertainty. One even-run pair is close to the threshold (about 0.492 m).

Map-quality metrics and overlap clouds are reused for rev2. Without the reuse flags the existing map-quality pipeline can regenerate evaluation maps from the saved poses, but still never reruns SLAM. The source manifest covers immutable canonical experiment files; the simultaneously updated TASK-06 directories are intentionally excluded.

See `results/eval/final_audit.json` for separate evaluation, image-inspection and full-suite statuses. A full-suite failure in an out-of-scope map test must not be hidden by marking the suite passed.

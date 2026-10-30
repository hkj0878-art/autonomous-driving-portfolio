# Held-out fixed-map localization

From `/home/hong/SLAM`:

```bash
source env.sh
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv/bin/python -m tools.loc.run_loc_eval --resume
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv/bin/python -m pytest -q
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv/bin/python -m tools.loc.verify_loc
```

`run_loc_eval --help` exposes map run/bag, scan parity, query bag, GNSS GT,
initial pose and query-time map reference arguments. `--map-bag` defaults to the
full original preprocessed bag, and **only even indices** are used in the prior.
For an already-even bag, select `--map-scan-filter all` to avoid splitting twice.
All outputs are constrained to `results/loc/` (including child experiment dirs).
Use a separate `--out results/loc/<experiment>` for different inputs; resume
rejects changed configuration, trajectory/GT hashes or implementation hashes.

Default: 1089 odd scans, range 1–80 m, voxel 0.3 m, fixed 0.2 m prior,
20-neighbor planar GICP covariances, epsilon 0.001, 1.0/0.5 m correspondence
gates, 30 iterations per stage. No IMU/GNSS or ongoing map poses enter tracking.
Mapping poses initialize the baseline and each robustness trial only. The first
map pose is later than the first scan; this explicit endpoint hold is disclosed
in the summary. Interpolation for robustness references never extrapolates.

`localize.py` provides the reusable registration, predictor, failure state and
streaming tracker. Its standalone CLI consumes a prior NPZ plus initial TUM.
`run_loc_eval.py` produces the baseline, paired-seed 200-trial benchmark,
GNSS metrics and plots. Trial JSON checkpoints are retained for inspection.
`verify_loc.py` independently audits the default experiment, parity separation,
input file preservation, trajectory coverage, thresholds and all trial cells,
and compares cached GICP with the public Open3D API on three real scans.

Open3D's public `registration_generalized_icp` rebuilds the target search tree
on every invocation and does not expose iterations. The cached outer loop uses
its actual `TransformationEstimationForGeneralizedICP.compute_transformation`
solver with the same convergence criteria and planar covariance model, using
one persistent scipy cKDTree. Unit tests and the real-scan audit compare it with
the public API. This is an explicit API adaptation to meet fixed-tree caching
and actual iteration accounting; it is not a different registration solver.
Reference: [Open3D implementation](https://github.com/isl-org/Open3D/blob/main/cpp/open3d/pipelines/registration/GeneralizedICP.cpp).

The two ATE variants use the unchanged TASK-05 `evaluate()` function. In
addition to the requested keyframe baseline, a common-timestamp comparison
avoids differences caused by keyframe versus dense-scan sampling. The primary
lever2D fit remains an in-sample nuisance estimate, and the same-journey split
is an optimistic estimate of true revisit performance.

`PYTHONNOUSERSITE=1` is necessary here because the user site contains numpy 2.x
and a conflicting matplotlib. Dependencies installed for this task are confined
to `.venv`; the recorded environment is `results/loc/venv_packages.txt`.
The project `pytest.ini` limits discovery to `tools/`; vendored GTSAM/pybind11
source tests need their own build environments and are not project tests.

For a different-day revisit, `--initial-pose X Y Z QX QY QZ QW` supplies the
known initial map-frame pose; `--reference-traj` supplies query-time map-frame
poses for the 200-trial convergence reference. In that mode, the table and
`mapping_baseline/` metrics refer to this supplied query reference. An independent
query-time reference is necessary to score map-frame robustness on a new bag.
For GNSS-only revisit evaluation without that reference, reuse the streaming
tracker and evaluate its output separately:

```bash
# initial_map_frame_tum.txt: >=3 valid TUM rows covering the query start time;
# only its interpolated pose at the first scan initializes tracking.
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv/bin/python -m tools.loc.localize \
  --prior results/loc/prior_map.npz --query-bag path/to/revisit_bag \
  --initial-traj initial_map_frame_tum.txt --out results/loc/revisit
PYTHONNOUSERSITE=1 .venv/bin/python -m tools.eval.traj_eval \
  --traj results/loc/revisit/loc_traj_tum.txt --gt query_gnss_tum.txt \
  --out results/loc/revisit/eval
```

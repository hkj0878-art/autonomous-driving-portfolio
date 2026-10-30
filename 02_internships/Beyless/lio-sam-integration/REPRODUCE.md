# LIO-SAM mapping and GNSS evaluation

A ROS 2 Humble pipeline for front-facing LiDAR and IMU mapping, evaluated against RTK GNSS without feeding GNSS into the estimator. The project implements timestamp conversion, frame diagnostics, safe experiment orchestration, final-keyframe evaluation, map export and held-out fixed-map localization around upstream LIO-SAM.

Read the [Korean portfolio report](report/REPORT.md) for the results, figures and per-value source references. Headline values are copied from existing results into [metrics_headline.json](report/metrics_headline.json); they are not recalculated by the report tooling. Loop closure reduced mean corrected horizontal ATE, but the difference is within repeat variability. Same-journey localization and GT-fitted lever offsets are optimistic estimates.

```text
config/              Shared mapping parameters and loop-off variant
launch/              Launch without xacro; RViz disabled by default
tools/               Preprocessing, orchestration, evaluation, maps, localization, tests
third_party/         GTSAM source and user-local installation
ws/                  Upstream LIO-SAM ROS workspace
data/                Original and processed bags (read-only in this task)
results/             Canonical runs, evaluations, maps, localization and archives
report/              Portfolio, copied figures, sourced metrics and environment lock
logs/                Build/work logs and task verification
```

## Environment without sudo

The recorded host is Ubuntu 22.04 with ROS 2 Humble, compiler/CMake, Boost, Eigen, PCL and TBB already available. See [build log](logs/task01_build.md) for dependency checks, upstream revisions and dependency decisions. ROS and native build dependencies are prerequisites; these commands do not install system packages. Do not modify unrelated ROS workspaces.

Run from `/home/hong/SLAM`. The source trees under `third_party/gtsam` (GTSAM 4.1.1) and `ws/src/LIO-SAM` (ros2 branch, exact commit in run metadata) are already present. For a separate machine, obtain those same revisions and the private input bag first. The build log records that no upstream source patches were needed.

Create the venv first on a fresh machine, then build native dependencies. Skip the two commented setup commands when using the delivered venv.

```bash
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
# Fresh environment only, when .venv does not exist:
# /usr/bin/python3 -m venv --system-site-packages .venv
# .venv/bin/python -m pip install -r report/requirements.txt
.venv/bin/python -c 'import sys; print(sys.executable)'
```

[requirements.txt](report/requirements.txt) records the venv distributions plus the inherited pytest, PyYAML and Pillow versions. ROS Python packages come from the ROS environment, not pip. The tools use numpy below 2 to avoid ROS binary compatibility problems.

```bash
# Run only if rebuilding the native dependencies is needed.
cmake -S third_party/gtsam -B third_party/gtsam/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/third_party/install" -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_WITH_TBB=ON
cmake --build third_party/gtsam/build -j4
cmake --install third_party/gtsam/build
export CMAKE_PREFIX_PATH="$PWD/third_party/install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$PWD/third_party/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
(cd ws && MAKEFLAGS=-j3 colcon build --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE="$PWD/../.venv/bin/python" -DCMAKE_PREFIX_PATH="$PWD/../third_party/install")
source env.sh
```

Reduce build concurrency if memory is limited. After building, `env.sh` sources ROS, the local GTSAM prefix and the workspace overlay. `launch/lio_sam_nox.launch.py` avoids the unavailable xacro dependency. Native OpenCV/GTSAM TBB versions differ; no observed experiment had a TBB crash (see the run log). No sudo is required.

## Reproduction order

The commands below describe **a fresh workspace copy with absent output directories**. They are not an overwrite recipe for the delivered data/results. Keep the source bag read-only; preserve or copy the delivered workspace before replaying canonical run names. Do not add `--overwrite` or run the experiment batch over existing outputs. Input data is private and is not downloaded by this repository.

```bash
source env.sh
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
# 1. Preprocess and create parity splits (only in a fresh output workspace).
.venv/bin/python tools/preprocess_bag.py --in data/synced_bag --out data/lio_bag
.venv/bin/python tools/preprocess_bag.py --in data/synced_bag --out data/lio_bag_even --keep-every 2 --phase 0
.venv/bin/python tools/preprocess_bag.py --in data/synced_bag --out data/lio_bag_odd --keep-every 2 --phase 1
# 2. Run ON/OFF three times each and the even-only mapping run at rate 1.0.
bash tools/run_experiments.sh 1.0
# 3. Evaluate final and online trajectories, map quality and loop consistency.
.venv/bin/python -m tools.eval.run_all_eval
# 4. Export the recorded representative run's PCDs, renderings and Nav2 grid.
.venv/bin/python tools/maps/export_maps.py --run loop_on_r2 --align-json results/eval/loop_on_r2/align_se3.json --compare-stride10
# 5. Build an even-scan fixed prior and localize odd scans; benchmark robustness.
.venv/bin/python -m tools.loc.run_loc_eval
# 6. Collect existing values and regenerate the report.
.venv/bin/python tools/report/collect_metrics.py
```

Mapping is nondeterministic across repeats. A new evaluation may choose a different median ON run; inspect its `summary.md` before selecting `--run` and `--align-json`. Report sources intentionally describe the delivered experiment set and should be reviewed when substituting a new dataset. [TASKS.md](TASKS.md) and [archive notes](results/archive_v2_fitness03/ARCHIVE_NOTE.md) explain the RPY correction, fitness threshold revision and preserved failed attempts. The latest evaluation methods are in [results/eval/summary.md](results/eval/summary.md); the older evaluation README contains superseded method descriptions.

Recorded timing guidance: initial preprocessing took about one minute ([TASK-02](logs/task02.md)); seven mapping replays require at least seven bag durations, plus startup, drain and saving ([run metadata](results/loop_on_r2/run_meta.json)). Initial localization plus robustness took about ten minutes ([TASK-05b](logs/task05b.md)). Map export and full map-quality evaluation depend strongly on CPU/memory and may take several minutes or longer; no fixed runtime is promised. A resumed localization run's wall time is not the full benchmark runtime. GTSAM compilation may dominate a fresh setup.

## Outputs and verification

- `results/<run>/`: logs, metadata, online TUM, optimized final-keyframe TUM, saved PCDs and loop pairs.
- `results/eval/`: GT ENU, per-run metrics/alignment, CSV aggregates and plots.
- `results/maps/loop_on_r2/`: original/voxel/cropped PCDs, `map2d.pgm`, the single `map2d.yaml`, and renders.
- `results/loc/`: prior map, localization TUM, per-scan timing, robustness trials and GNSS metrics.
- The original data report is preserved in `results/archive_v1_wrong_rpy/data_report/`.

These verification commands read existing results. The report dry-run checks referenced files and command entry points without replaying bags or creating result outputs.

```bash
source env.sh
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
.venv/bin/python tools/report/collect_metrics.py --check
.venv/bin/python tools/report/verify_report.py --dry-run-readme
.venv/bin/python -m pytest -q
```

See [TASK-07 verification log](logs/task07.md). Full project test discovery is limited to `tools/`; vendored upstream tests require their own environments. Tests use temporary directories. Live ROS mapping, full evaluation and map export were not replayed during report verification, to preserve existing data/results.

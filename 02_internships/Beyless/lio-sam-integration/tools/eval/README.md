# TASK-05 evaluation

Run from `/home/hong/SLAM`:

```bash
source env.sh
PYTHONDONTWRITEBYTECODE=1 /home/hong/SLAM/.venv/bin/python -m tools.eval.run_all_eval
PYTHONDONTWRITEBYTECODE=1 /home/hong/SLAM/.venv/bin/python -m pytest tools/tests -q
```

Every product, including `gt_gnss_enu_tum.txt`, is under `results/eval/`.
This resolves TASKS.md's GT-path example in favor of its explicit output-only
restriction. Source bags, experiment directories and TASK-06 trees are untouched.
Individual CLIs use `python -m tools.eval.gnss_gt`, `traj_eval`,
`build_map_from_poses` and `map_quality`; see `--help`.

`build_map_from_poses` accepts all/even/odd zero-based scan indices. SLERP
interpolates rotations and linear interpolation translates scan headers;
there is no point deskew or pose extrapolation. NPZ stores xyz,
relative_time_s, stamp_s and count. Times are weighted voxel means, allowing
provenance through downsampling rather than pretending each voxel is an
original return. Overlap caches retain those fields for first/last windows.
Voxel merges are incremental and enforce a 3-million-voxel limit.

MME is differential Gaussian entropy in nats (negative values are valid).
MPV is the smallest sample-covariance eigenvalue in m². Both use radius
0.3 m, min 6 neighbors, floor 1e-9 m², all eligible 0.1 m voxel points.
Saved GlobalMap points are transformed into ENU before voxelization and
cropped to the same GNSS XY bounding box, with height unconstrained.
Coverage and eligible-neighbor fractions accompany each value.

Overlap NN statistics concatenate both directions, restricting each to
points within 1 m of the other cloud. The >0.2 m fraction is a discrepancy
proxy that includes different visibility and sampling, not a detector of
semantic double walls. Acquisition windows are the first/last 15 s of GNSS
time, intersected with final pose support.

RPE uses every matched start and the first end at or beyond each requested
GT distance, with actual distance as denominator. GNSS yaw comes from a 2 m
centered position tangent, roll/pitch zero. It includes antenna/body-origin
and body-heading effects and is an approximation, particularly at corners.

The default GNSS filter uses status >=2 and horizontal sigma <=0.05 m on
cov[0]/cov[4]. Vertical sigma is reported without the 5 cm cutoff.
Horizontal (2D) ATE after SE3 alignment is primary; 3D ATE is retained.

`loop_validation.csv` evaluates v3 recorded marker pairs. The latest cumulative
`loop_edges` LINE_LIST records current/previous endpoint coordinates, marker time,
wall and simulation receive times; snapshots retain earlier observations. After
saving the final keyframes, endpoints are matched independently to nearest final
poses. Evaluation rejects matches over 0.2 m. GT is interpolated at the paired
keyframe stamps; horizontal distance <2 m is TRUE_POSITIVE, otherwise FALSE_POSITIVE.
The proximity criterion does not independently test rotational consistency.
Final versus online ATE does not isolate a single loop insertion.

The first line of summary.md gives the median loop ON final 2D SE3+lever ATE run
for TASK-06. Each run has align_se3.json with R, t and a homogeneous 4x4
transform from LIO-SAM odom to first-fix GNSS ENU; it uses the standard SE3 fit.

Joint lever estimation uses unbounded ordinary least squares over global
rotation, translation, and a 3D body-frame lever. Planar motion weakly
observes its vertical component; singular values and condition number are
recorded. Treat its improved in-sample ATE as diagnostic, not independent
validation of an estimated antenna offset. `|l| > 3 m` is flagged.

# TASK-06 rev1 — v3 portfolio maps (2026-10-26)

User scope is strictly tools/maps/ and results/maps/. Therefore this log replaces
logs/task06.md for this revision, and revised tests live in tools/maps/test_revision.py.
TASKS.md, tools/tests/, tools/eval/, results/eval/, data/ and input run directories
were not edited. No external tiles or aerial imagery were used.

## Outputs and method

The old loop_on_r2 directory is preserved under results/maps/archive_v2/loop_on_r2/.
The canonical results/maps/loop_on_r2/ now uses v3 source map/poses (SHA256 in
map2d_stats.json). map_full.pcd is a byte-identical source copy; map_0p10.pcd
is its full 0.1 m voxel map. map_crop.pcd and map_crop_0p10.pcd use the final
trajectory bounding box plus 30 m in x/y. Full and cropped 0.1 m ENU PCDs apply
results/eval/loop_on_r2/align_se3.json, rigid SE3 only, with no lever correction.
The Nav2 map remains in the local map frame; its YAML origin and plotted path
use that same frame. All frames are labelled in stats/figures.

2D rays use every second original /points_raw scan, final-pose SLERP/translation
interpolation, the local-ground-relative 0.3–2.0 m slice and nearest return in
0.5 degree azimuth bins. Local ground is the 10th percentile in 2 m cells.
Bresenham log-odds stop at the static global obstacle prior. Ground-only rays
are no longer used. The 60 m range cap is retained. No per-point deskew is
applied; sparse ground and structures can bias the local ground estimate.
Unseen regions, occlusions and uncertain cells remain unknown.

The sole map2d.yaml uses free_thresh=0.196, with an explanatory comment.
For gray 205: occupancy=1-205/255=0.196078, which is between 0.196 and 0.65.
It therefore remains unknown. PGM is P5 with values 0/254/205 and a vertical
flip so its YAML origin is the world lower-left corner. No auxiliary YAML is
present in the active output; historical YAMLs remain in the untouched archive.

Rendering first tries Open3D OffscreenRenderer in a separate process with
EGL_PLATFORM=surfaceless. It succeeded using Open3D's Vulkan backend on RADV.
Post-processing is disabled to retain the height colors. A matplotlib 3D
fallback is available. All previews use the crop; render voxels are 0.2 m.
The ON/OFF comparison uses the same 40 m square ENU neighborhood centered at
the ON start, with each run's SE3 alignment; maps are cropped from each run's
trajectory +30 m region. Early/late path colors identify the loop segment.
This is a visual comparison of final maps, not a separate early/late cloud metric.

## Unknown coverage

Driving region = cells whose centers are within 30 m of the trajectory polyline.
It is sampled at <=0.025 m spacing; buffer-boundary error <=0.0125 m.

| Input/method | Region cells | Free cells | Unknown fraction |
|---|---:|---:|---:|
| Archived v2 original export, stride 10 | 5,479,420 | 1,078,871 | 78.5786% |
| v3, revised slice/crop/YAML, stride 10 | 5,476,537 | 847,810 | 82.9134% |
| v3, revised slice/crop/YAML, stride 2 | 5,476,537 | 1,355,497 | 73.6431% |

Against the historical output: unknown decreases 4.9355 percentage points;
the input run also changed, so this is not an isolated stride experiment.
The controlled v3 comparison changes only stride: unknown decreases 9.2702
percentage points, with 507,687 additional free cells (about 1,269.2 m²).
Scans used: 212 -> 1059; rays: 51,949 -> 259,204. The remaining 73.64% includes
building interiors, areas behind walls and the wide 30 m buffer.

## Validation

Independent tools/maps/verify_maps.py checks P5 header/payload/value set,
Nav2 threshold round-trip (0/254/205 -> occupied/free/unknown), YAML fields,
origin/grid dimensions, all source SHA256 values including OFF and alignments,
byte identity of the full PCD, crop bounds/counts, ENU coordinates to 20 µm,
coverage counts and image size/nonblank content. Grid: 2712 x 2401 cells,
135.60 x 120.05 m, origin [-58.1,-90.0,0], resolution 0.05 m.
The final-keyframe cells are 94.70% free. See validation.json and
results/maps/verification_rev1.log for the actual validation result.

All four final PNGs were opened and visually inspected: rectangular building
outlines, surrounding road and the path on that road are visible; occupancy
orientation matches the top view; ON/OFF panel axes match. The initially
crowded comparison title was adjusted and the image inspected again.

Pytest: 55 passed, 1 deselected. The excluded existing test is
`test_nav2_roundtrip_and_spec_threshold_conflict`, which explicitly requires
the obsolete companion YAML and primary threshold 0.25. It conflicts with rev1.
It was separately executed and its expected failure retained in
results/maps/obsolete_test_rev1.log. It cannot be edited under the user's scope.
The new test verifies the replacement single-YAML behavior; existing ground,
Bresenham, YAML corruption/orientation tests pass. Thus the unrestricted old
suite is not claimed to pass; the suite excluding that superseded test passes.

Observed raycast VmHWM: 506820 KiB. Timed render maximum RSS: 918924 KiB
(results/maps/render_resources.log), below the 4 GB budget.

## Reproduction

```bash
source env.sh
PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 \
  .venv/bin/python tools/maps/export_maps.py --run loop_on_r2 \
  --align-json results/eval/loop_on_r2/align_se3.json --compare-stride10
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=2 \
  .venv/bin/python tools/maps/verify_maps.py results/maps/loop_on_r2
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=2 \
  .venv/bin/python tools/maps/compare_archive.py
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q \
  tools/tests tools/maps/test_revision.py -k 'not spec_threshold_conflict' \
  -o cache_dir=results/maps/.pytest_cache
```

#!/usr/bin/env python3
"""Export immutable LIO-SAM results to PCD, previews and a Nav2 trinary map.

Scan poses use SLERP/linear interpolation of final keyframes; no deskew.
The 2-D map is an offline projection, not a navigation safety certification.
"""
import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
import shutil
import os
import subprocess
import sys
from scipy.spatial import cKDTree

import matplotlib
matplotlib.use('Agg')
# ROS system-site-packages can pre-load an incompatible mpl_toolkits namespace.
# Match the toolkit to the selected matplotlib without changing the environment.
import mpl_toolkits
_toolkit = Path(matplotlib.__file__).parent.parent / 'mpl_toolkits'
if (_toolkit / 'mplot3d').is_dir():
    mpl_toolkits.__path__.insert(0, str(_toolkit))
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.ndimage import distance_transform_edt
from scipy.spatial.transform import Rotation, Slerp
import yaml

ROOT = Path(__file__).resolve().parents[2]
OCCUPIED, FREE, UNKNOWN = 0, 254, 205
NAV2_FREE_THRESH = 0.196
DPI = 200
RENDER_POINTS = 160000
MAX_GRID_CELLS = 40000000


@dataclass(frozen=True)
class Config:
    resolution: float = 0.05
    voxel: float = 0.1
    ground_cell: float = 2.0
    ground_percentile: float = 10.0
    obstacle_min: float = 0.3
    obstacle_max: float = 2.0
    ground_tolerance: float = 0.2
    margin: float = 30.0
    scan_stride: int = 2
    angular_bin_deg: float = 0.5
    ray_max_range: float = 60.0
    log_hit: float = 0.85
    log_miss: float = -0.4
    prior_occupied: float = 2.0
    log_limit: float = 5.0
    occupied_thresh: float = 0.65
    free_thresh: float = NAV2_FREE_THRESH


class GroundModel:
    """Local low percentile, with nearest populated cell for missing terrain."""
    def __init__(self, points, cfg):
        self.cell = cfg.ground_cell
        self.origin = np.floor(points[:, :2].min(axis=0) / self.cell) * self.cell
        ij = np.floor((points[:, :2] - self.origin) / self.cell).astype(int)
        self.shape = ij.max(axis=0) + 1
        heights = np.full(tuple(self.shape[::-1]), np.nan)
        ids = ij[:, 1] * self.shape[0] + ij[:, 0]
        order = np.argsort(ids)
        starts = np.r_[0, np.flatnonzero(np.diff(ids[order])) + 1]
        ends = np.r_[starts[1:], len(order)]
        for a, b in zip(starts, ends):
            k = ids[order[a]]
            heights.flat[k] = np.percentile(points[order[a:b], 2], cfg.ground_percentile)
        missing = ~np.isfinite(heights)
        nearest = distance_transform_edt(missing, return_distances=False, return_indices=True)
        self.heights = heights[tuple(nearest)]

    def at(self, points):
        ij = np.floor((points[:, :2] - self.origin) / self.cell).astype(int)
        ij = np.clip(ij, 0, self.shape - 1)
        return self.heights[ij[:, 1], ij[:, 0]]


def obstacle_mask(points, ground, cfg):
    relative = points[:, 2] - ground.at(points)
    return (relative >= cfg.obstacle_min) & (relative <= cfg.obstacle_max)


def bresenham_batch(start, ends, shape, blocked):
    """Yield visited flat indices for parallel integer Bresenham rays.

    Excludes origin, includes endpoint, stops at first known obstacle. Rows are
    world-positive y; PGM row flipping is performed only during serialization.
    """
    ends = np.asarray(ends, dtype=int)
    if not len(ends):
        return
    x = np.full(len(ends), int(start[0])); y = np.full(len(ends), int(start[1]))
    dx = np.abs(ends[:, 0] - x); dy = -np.abs(ends[:, 1] - y)
    sx = np.where(x < ends[:, 0], 1, -1); sy = np.where(y < ends[:, 1], 1, -1)
    err = dx + dy
    active = (x != ends[:, 0]) | (y != ends[:, 1])
    while active.any():
        e2 = 2 * err.copy()
        mx = active & (e2 >= dy); my = active & (e2 <= dx)
        err[mx] += dy[mx]; x[mx] += sx[mx]
        err[my] += dx[my]; y[my] += sy[my]
        inside = active & (x >= 0) & (y >= 0) & (x < shape[1]) & (y < shape[0])
        ids = y[inside] * shape[1] + x[inside]
        terminal = (x == ends[:, 0]) & (y == ends[:, 1])
        hits = np.zeros(len(ends), dtype=bool)
        hits[inside] = blocked.ravel()[ids]
        yield np.unique(ids[~blocked.ravel()[ids]]), np.unique(ids[blocked.ravel()[ids]])
        active &= inside & ~terminal & ~hits


def map_yaml(origin, cfg):
    return dict(image='map2d.pgm', resolution=cfg.resolution,
                origin=[float(origin[0]), float(origin[1]), 0.0], negate=0,
                occupied_thresh=cfg.occupied_thresh, free_thresh=cfg.free_thresh, mode='trinary')


def write_grid(out, pixels, origin, cfg):
    # PGM first row is world upper edge; origin is world lower-left corner.
    h, w = pixels.shape
    with (out / 'map2d.pgm').open('wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(np.flipud(pixels).astype(np.uint8).tobytes())
    (out / 'map2d.yaml').write_text(
        '# free_thresh 0.196 preserves gray 205 as unknown in Nav2 trinary mode.\n'
        + yaml.safe_dump(map_yaml(origin, cfg), sort_keys=False))
    (out / 'map2d_nav2.yaml').unlink(missing_ok=True)


def read_xyz(msg):
    fields = {f.name: f for f in msg.fields}
    if any(n not in fields or fields[n].datatype != 7 for n in ('x', 'y', 'z')):
        raise ValueError('PointCloud2 requires float32 x/y/z fields')
    dtype = np.dtype(dict(names=['x','y','z'], formats=[('>' if msg.is_bigendian else '<')+'f4']*3,
                          offsets=[fields[n].offset for n in ('x','y','z')], itemsize=msg.point_step))
    a = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                   strides=(msg.row_step, msg.point_step))
    xyz = np.column_stack([a[n].ravel() for n in ('x','y','z')])
    return xyz[np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 0.5)]


def raycast_bag(bag, poses, ground, blocked, origin, cfg):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    if '/points_raw' not in {t.name for t in reader.get_all_topics_and_types()}:
        raise ValueError('Bag missing /points_raw')
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/points_raw']))
    t0 = poses[0, 0]
    times = poses[:, 0] - t0
    slerp = Slerp(times, Rotation.from_quat(poses[:, 4:8]))
    odds = np.where(blocked, cfg.prior_occupied, 0.0).astype(np.float32)
    observed = blocked.copy()
    scans = used = rays = skipped = 0
    while reader.has_next():
        _, data, _ = reader.read_next()
        index = scans; scans += 1
        if index % cfg.scan_stride:
            continue
        msg = deserialize_message(data, PointCloud2)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - t0
        if stamp < times[0] or stamp > times[-1]:
            skipped += 1
            continue
        rot = slerp(stamp)
        pos = np.array([np.interp(stamp, times, poses[:, k]) for k in (1,2,3)])
        xyz = read_xyz(msg)
        world = rot.apply(xyz) + pos
        relative = world[:, 2] - ground.at(world)
        distance = np.linalg.norm(world[:, :2] - pos[:2], axis=1)
        obs = (relative >= cfg.obstacle_min) & (relative <= cfg.obstacle_max)
        eligible = obs & (distance <= cfg.ray_max_range) & (distance > cfg.resolution)
        world, distance, obs = world[eligible], distance[eligible], obs[eligible]
        angles = np.arctan2(world[:,1]-pos[1], world[:,0]-pos[0])
        bins = np.floor((np.rad2deg(angles)+180) / cfg.angular_bin_deg).astype(int)
        # Horizontal obstacle slice from the original bag; nearest return per azimuth.
        order = np.lexsort((np.where(obs, distance, -distance), ~obs, bins))
        _, first = np.unique(bins[order], return_index=True)
        endpoints = world[order[first], :2]
        ends = np.floor((endpoints-origin)/cfg.resolution).astype(int)
        start = np.floor((pos[:2]-origin)/cfg.resolution).astype(int)
        for free_ids, hit_ids in bresenham_batch(start, ends, blocked.shape, blocked):
            odds.ravel()[free_ids] += cfg.log_miss
            odds.ravel()[hit_ids] += cfg.log_hit
            observed.ravel()[free_ids] = True
            observed.ravel()[hit_ids] = True
        np.clip(odds, -cfg.log_limit, cfg.log_limit, out=odds)
        used += 1; rays += len(ends)
    pixels = np.full(blocked.shape, UNKNOWN, dtype=np.uint8)
    probability = 1 / (1 + np.exp(-odds))
    pixels[observed & (probability <= cfg.free_thresh)] = FREE
    pixels[observed & (probability >= cfg.occupied_thresh)] = OCCUPIED
    return pixels, dict(scans_total=scans, scans_used=used, scans_outside_pose_time=skipped, rays=rays)


def render(out, points, poses, pixels, origin, cfg):
    points = np.asarray(o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)).voxel_down_sample(.2).points)
    bounds = trajectory_bounds(poses, cfg.margin)
    points = points[::max(1, int(np.ceil(len(points)/RENDER_POINTS)))]
    lo, hi = np.percentile(points[:,2], [2,98])
    fig, ax = plt.subplots(figsize=(10,9))
    sc = ax.scatter(points[:,0], points[:,1], c=points[:,2], s=.3, cmap='viridis', vmin=lo, vmax=hi, rasterized=True)
    ax.plot(poses[:,1], poses[:,2], color='#D55E00', lw=1.5, label='Final keyframe trajectory')
    ax.scatter(*poses[0,1:3], color='#CC79A7', s=35, label='Start', zorder=4)
    ax.set(xlabel='Map x (m)', ylabel='Map y (m)', title='LIO-SAM map: top view', aspect='equal')
    ax.set_xlim(bounds[0][0], bounds[1][0]); ax.set_ylim(bounds[0][1], bounds[1][1])
    ax.legend(); fig.colorbar(sc, ax=ax, label='Height in map frame (m)')
    fig.savefig(out/'map_top.png', dpi=DPI); plt.close(fig)
    # Try EGL in a child process: unsupported drivers can abort in native code.
    renderer_status = try_egl(out, points, poses)
    # Keep a labelled matplotlib fallback if EGL failed.
    fig = plt.figure(figsize=(11,9)); ax = fig.add_subplot(projection='3d', computed_zorder=False)
    sc = ax.scatter(*points.T, c=points[:,2], s=1.0, cmap='viridis', vmin=lo, vmax=hi, rasterized=True, zorder=1)
    ax.plot(poses[:,1], poses[:,2], poses[:,3], color='#D55E00', lw=2, label='Final keyframes', zorder=10)
    ax.set(xlabel='Map x (m)', ylabel='Map y (m)', zlabel='Map z (m)', title='LIO-SAM map: oblique view (vertical scale exaggerated)')
    spans = np.maximum(np.ptp(points, axis=0), 1)
    spans[2] = max(spans[2], .15 * max(spans[:2]))
    ax.set_box_aspect(spans); ax.view_init(elev=52, azim=-65)
    ax.set_xlim(bounds[0][0], bounds[1][0]); ax.set_ylim(bounds[0][1], bounds[1][1])
    from matplotlib.ticker import MaxNLocator
    ax.zaxis.set_major_locator(MaxNLocator(4))
    ax.legend(); fig.colorbar(sc, ax=ax, shrink=.55, label='Height (m)')
    if renderer_status['success']:
        plt.close(fig)
        fig, ax = plt.subplots(figsize=(11,9))
        ax.imshow(plt.imread(out/'egl_preview.png')); ax.axis('off')
        ax.set_title('LIO-SAM cropped map: EGL oblique view / orange trajectory')
    fig.savefig(out/'map_oblique.png', dpi=DPI); plt.close(fig)
    (out/'egl_preview.png').unlink(missing_ok=True)
    from matplotlib.colors import ListedColormap
    categories = np.full(pixels.shape, 2, dtype=np.uint8)
    categories[pixels == OCCUPIED] = 0
    categories[pixels == FREE] = 1
    extent = [origin[0], origin[0]+pixels.shape[1]*cfg.resolution,
              origin[1], origin[1]+pixels.shape[0]*cfg.resolution]
    fig, ax = plt.subplots(figsize=(11,9))
    ax.imshow(categories, origin='lower', extent=extent, interpolation='nearest', vmin=0, vmax=2,
              cmap=ListedColormap(['#222222','#FAFAFA','#9BA5AD']))
    ax.plot(poses[:,1], poses[:,2], color='#D55E00', lw=1.5, label='Final keyframes')
    ax.scatter(*poses[0,1:3], color='#CC79A7', s=35, label='Start')
    ax.set(xlabel='Map x (m)', ylabel='Map y (m)', title='2D occupancy: black occupied / white free / gray unknown')
    ax.legend(); fig.savefig(out/'map2d.png', dpi=DPI); plt.close(fig)
    return renderer_status


def trajectory_bounds(poses, margin=30.):
    return poses[:,1:3].min(axis=0)-margin, poses[:,1:3].max(axis=0)+margin


def crop_points(points, bounds):
    lo, hi = bounds
    return points[((points[:,:2] >= lo) & (points[:,:2] <= hi)).all(axis=1)]


def driving_region(pixels, origin, poses, cfg):
    """Euclidean trajectory buffer at cell centers, using dense path samples."""
    # Dense samples at <= half a cell; nearest-distance ambiguity <= 0.0125 m.
    samples = []
    for a, b in zip(poses[:-1,1:3], poses[1:,1:3]):
        n = max(1, int(np.ceil(np.linalg.norm(b-a)/(cfg.resolution/2))))
        samples.append(a+(b-a)*np.arange(n)[:,None]/n)
    tree = cKDTree(np.vstack([*samples, poses[-1:,1:3]]))
    counts = dict(occupied=0, free=0, unknown=0)
    x = origin[0]+(np.arange(pixels.shape[1])+.5)*cfg.resolution
    for start in range(0, pixels.shape[0], 64):
        rows = np.arange(start, min(start+64, pixels.shape[0]))
        xx, yy = np.meshgrid(x, origin[1]+(rows+.5)*cfg.resolution)
        distance, _ = tree.query(np.column_stack([xx.ravel(), yy.ravel()]))
        selected = pixels[rows].ravel()[distance <= cfg.margin]
        for name, value in [('occupied',OCCUPIED),('free',FREE),('unknown',UNKNOWN)]:
            counts[name] += int(np.count_nonzero(selected == value))
    total = sum(counts.values())
    return dict(buffer_m=cfg.margin, cells=counts, total_cells=total,
                unknown_fraction=counts['unknown']/total, area_m2=total*cfg.resolution**2,
                method='cell centers within trajectory polyline buffer; sampling error <= resolution/4')


def try_egl(out, points, poses):
    scratch = out/'render_input.npz'
    np.savez(scratch, xyz=points, path=poses[:,1:4])
    env = dict(os.environ, EGL_PLATFORM='surfaceless', OMP_NUM_THREADS='2')
    try:
        result = subprocess.run([sys.executable, str(Path(__file__).with_name('render_egl.py')),
                                 str(scratch), str(out/'egl_preview.png')],
                                capture_output=True, text=True, env=env, timeout=60)
        status = dict(success=result.returncode == 0 and (out/'egl_preview.png').is_file(),
                      returncode=result.returncode, diagnostic=(result.stdout+result.stderr)[-2000:])
    except subprocess.TimeoutExpired:
        status = dict(success=False, diagnostic='EGL attempt timed out at 60 seconds')
    finally:
        scratch.unlink(missing_ok=True)
    return status


def render_overlap(out, on_poses, cfg):
    """ON/OFF in common ENU coordinates, zoomed at the loop start/end."""
    align_root = ROOT/'results/eval'
    on_align = json.loads((align_root/'loop_on_r2/align_se3.json').read_text())
    center = on_poses[0,1:4]@np.array(on_align['R']).T+on_align['t']
    fig, axes = plt.subplots(1,2,figsize=(14,7),sharex=True,sharey=True)
    provenance = {}
    for ax, run in zip(axes, ['loop_on_r2','loop_off_r2']):
        src = ROOT/'results'/run
        alignment_path = align_root/run/'align_se3.json'
        alignment = json.loads(alignment_path.read_text())
        r,t = np.array(alignment['R']), np.array(alignment['t'])
        poses = np.loadtxt(src/'keyframes_final_tum.txt')
        cloud = o3d.io.read_point_cloud(str(src/'map/GlobalMap.pcd')).voxel_down_sample(.2)
        xyz = np.asarray(cloud.points)
        xyz = crop_points(xyz, trajectory_bounds(poses,cfg.margin))@r.T+t
        xyz = crop_points(xyz, (center[:2]-20, center[:2]+20))
        lo,hi = np.percentile(xyz[:,2],[2,98])
        ax.scatter(xyz[:,0],xyz[:,1],c=xyz[:,2],s=2,cmap='viridis',vmin=lo,vmax=hi,rasterized=True)
        path = poses[:,1:4]@r.T+t
        for mask,color,label in [(poses[:,0]<=poses[0,0]+15,'#D55E00','First 15 s'),
                                 (poses[:,0]>=poses[-1,0]-15,'#0072B2','Last 15 s')]:
            ax.plot(path[mask,0],path[mask,1],c=color,lw=2,label=label)
        ax.set(xlim=(center[0]-20,center[0]+20),ylim=(center[1]-20,center[1]+20),
               xlabel='ENU east (m)',ylabel='ENU north (m)',aspect='equal',title=run+' / loop overlap')
        ax.legend()
        provenance[run] = {str(p.relative_to(ROOT)):sha256(p) for p in
                           [src/'map/GlobalMap.pcd',src/'keyframes_final_tum.txt',alignment_path]}
    fig.suptitle('Loop neighborhood / height-colored map / 0.2 m voxel / independent SE3 alignment')
    fig.tight_layout(rect=(0,0,1,.94)); fig.savefig(out/'map_loop_overlap.png',dpi=DPI); plt.close(fig)
    return provenance


def sha256(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def export(run='loop_on_r2', bag=None, align_json=None, cfg=Config(), compare_stride10=False):
    if Path(run).name != run or run in ('.', '..'):
        raise ValueError('--run must be a single run directory name')
    if cfg.scan_stride < 1:
        raise ValueError('scan stride must be positive')
    src = ROOT/'results'/run
    out = ROOT/'results/maps'/run
    cloud_path = src/'map/GlobalMap.pcd'; pose_path = src/'keyframes_final_tum.txt'
    for path in (cloud_path, pose_path):
        if not path.is_file():
            raise FileNotFoundError(path)
    poses = np.loadtxt(pose_path, ndmin=2)
    if poses.shape[1] != 8 or len(poses) < 2 or not np.isfinite(poses).all() or np.any(np.diff(poses[:,0]) <= 0):
        raise ValueError('Invalid final keyframe TUM')
    original = o3d.io.read_point_cloud(str(cloud_path))
    if original.is_empty() or not np.isfinite(np.asarray(original.points)).all():
        raise ValueError('Empty or nonfinite map')
    cloud = original.voxel_down_sample(cfg.voxel)
    full_points = np.asarray(cloud.points)
    bounds = trajectory_bounds(poses, cfg.margin)
    points = crop_points(full_points, bounds)
    ground = GroundModel(points, cfg)
    obstacles = points[obstacle_mask(points, ground, cfg)]
    if not len(obstacles):
        raise ValueError('No obstacles after ground filtering')
    origin = np.floor(bounds[0]/cfg.resolution)*cfg.resolution
    size = np.ceil((bounds[1]-origin)/cfg.resolution).astype(int)
    if np.prod(size) > MAX_GRID_CELLS:
        raise ValueError(f'Grid too large: {size}')
    blocked = np.zeros(tuple(size[::-1]), bool)
    ij = np.floor((obstacles[:,:2]-origin)/cfg.resolution).astype(int)
    ij = np.clip(ij,0,size-1)
    blocked[ij[:,1],ij[:,0]] = True
    print(f'Cloud {len(original.points)} -> {len(points)}; grid {size}; raycasting...', flush=True)
    baseline = None
    if compare_stride10:
        from dataclasses import replace
        old_pixels, old_scans = raycast_bag(bag or ROOT/'data/lio_bag', poses, ground, blocked, origin, replace(cfg,scan_stride=10))
        baseline = dict(driving_region=driving_region(old_pixels,origin,poses,cfg), **old_scans)
    pixels, scan_stats = raycast_bag(bag or ROOT/'data/lio_bag', poses, ground, blocked, origin, cfg)
    if not scan_stats['scans_used'] or not np.any(pixels == FREE):
        raise ValueError('Raycasting produced no observed free space')
    out.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(cloud_path, out/'map_full.pcd')
    if not o3d.io.write_point_cloud(str(out/'map_0p10.pcd'), cloud, compressed=True):
        raise RuntimeError('PCD write failed')
    cropped = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    o3d.io.write_point_cloud(str(out/'map_crop_0p10.pcd'),cropped,compressed=True)
    original_crop = original.select_by_index(np.flatnonzero(((np.asarray(original.points)[:,:2]>=bounds[0]) & (np.asarray(original.points)[:,:2]<=bounds[1])).all(axis=1)))
    o3d.io.write_point_cloud(str(out/'map_crop.pcd'),original_crop,compressed=True)
    alignment = 'omitted: --align-json not provided'
    if align_json is not None and Path(align_json).is_file():
        alignment_data = json.loads(Path(align_json).read_text())
        matrix = np.asarray(alignment_data['R'], dtype=float)
        translation = np.asarray(alignment_data['t'], dtype=float)
        if matrix.shape != (3,3) or translation.shape != (3,) or not np.isfinite(matrix).all() or not np.isfinite(translation).all():
            raise ValueError('Alignment must contain finite R (3x3), t (3,)')
        if not np.allclose(matrix.T@matrix, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(matrix), 1, atol=1e-5):
            raise ValueError('Alignment R must be a proper SE3 rotation')
        aligned = o3d.geometry.PointCloud()
        aligned.points = o3d.utility.Vector3dVector(full_points@matrix.T+translation)
        if not o3d.io.write_point_cloud(str(out/'map_0p10_enu.pcd'), aligned, compressed=True):
            raise RuntimeError('ENU PCD write failed')
        aligned.points = o3d.utility.Vector3dVector(points@matrix.T+translation)
        o3d.io.write_point_cloud(str(out/'map_crop_0p10_enu.pcd'),aligned,compressed=True)
        alignment = str(align_json)
    elif align_json is not None:
        alignment = f'omitted: alignment file missing: {align_json}'
    # Avoid a stale optional export being mistaken for the current result.
    if alignment.startswith('omitted'):
        (out/'map_0p10_enu.pcd').unlink(missing_ok=True)
        (out/'map_crop_0p10_enu.pcd').unlink(missing_ok=True)
    write_grid(out, pixels, origin, cfg)
    renderer = render(out, points, poses, pixels, origin, cfg)
    overlap_sources = render_overlap(out,poses,cfg) if run == 'loop_on_r2' else {}
    stats = dict(run=run, frame='LIO-SAM map', config=asdict(cfg), origin=origin.tolist(),
                 width_cells=int(size[0]), height_cells=int(size[1]), size_m=(size*cfg.resolution).tolist(),
                 cells={name:int(np.count_nonzero(pixels==value)) for name,value in [('occupied',OCCUPIED),('free',FREE),('unknown',UNKNOWN)]},
                 input_points=len(original.points), voxel_points=len(full_points), crop_points=len(points), obstacle_points=len(obstacles),
                 crop_bounds_xy=[v.tolist() for v in bounds],
                 driving_region=driving_region(pixels,origin,poses,cfg),
                 stride10_baseline=baseline, overlap_sources=overlap_sources,
                 ground_height_range_m=[float(ground.heights.min()),float(ground.heights.max())],
                 nav2_load_yaml='map2d.yaml',
                 nav2_threshold_note='free_thresh=0.196 preserves PGM gray 205 as unknown.',
                 alignment=alignment, alignment_sha256=sha256(Path(align_json)) if not alignment.startswith('omitted') else None,
                 rendering=renderer,
                 source_sha256={str(p.relative_to(ROOT)):sha256(p) for p in (cloud_path,pose_path)},
                 approximations=['No per-point deskew; scans use final-keyframe interpolated pose.',
                                 'Local ground percentile can fail on slopes, vegetation and sparse cells.',
                                 'Raw bag obstacle-height slice only; nearest azimuth endpoint. Static global obstacle prior blocks rays.',
                                 '2D projection omits overhanging obstacles above 2 m; no robot footprint inflation.'],
                 **scan_stats)
    (out/'map2d_stats.json').write_text(json.dumps(stats, indent=2)+'\n')
    print(json.dumps(stats, indent=2), flush=True)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', default='loop_on_r2')
    parser.add_argument('--bag', type=Path, default=ROOT/'data/lio_bag')
    parser.add_argument('--align-json', type=Path, help='SE3 JSON: {"R": [[...],...], "t": [x,y,z]}')
    parser.add_argument('--compare-stride10', action='store_true')
    parser.add_argument('--scan-stride', type=int, default=Config.scan_stride)
    args = parser.parse_args()
    export(args.run, args.bag, args.align_json, Config(scan_stride=args.scan_stride), args.compare_stride10)


if __name__ == '__main__':
    main()

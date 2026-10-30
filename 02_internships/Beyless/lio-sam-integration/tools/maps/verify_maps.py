#!/usr/bin/env python3
"""Independently validate Nav2 PGM/YAML, dimensions, counts and provenance."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[2]
MIN_TRAJECTORY_FREE_FRACTION = 0.8


def read_pgm(path):
    with Path(path).open('rb') as f:
        def line():
            while True:
                s = f.readline()
                if not s:
                    raise ValueError('Truncated PGM header')
                s = s.split(b'#', 1)[0].strip()
                if s:
                    return s
        if line() != b'P5':
            raise ValueError('Expected binary PGM P5')
        width, height = map(int, line().split())
        if width <= 0 or height <= 0 or line() != b'255':
            raise ValueError('Invalid PGM dimensions/maxval')
        raw = f.read()
        if len(raw) != width * height:
            raise ValueError('PGM payload size mismatch')
    pixels = np.frombuffer(raw, dtype=np.uint8).reshape(height,width)
    if not set(np.unique(pixels)).issubset({0,205,254}):
        raise ValueError('PGM contains invalid trinary values')
    return pixels


def nav2_decode(gray, meta):
    """Humble map_io.cpp trinary equations, for opaque grayscale PGM."""
    shade = np.asarray(gray, dtype=float) / 255.0
    occ = shade if meta['negate'] else 1.0 - shade
    return np.where(occ > meta['occupied_thresh'], 100,
                    np.where(occ < meta['free_thresh'], 0, -1))


def validate(directory, check_sources=True):
    directory = Path(directory)
    meta = yaml.safe_load((directory/'map2d.yaml').read_text())
    required = {'image','resolution','origin','negate','occupied_thresh','free_thresh','mode'}
    if required - meta.keys():
        raise ValueError('Missing Nav2 YAML fields')
    if (meta['image'] != 'map2d.pgm' or meta['mode'] != 'trinary' or meta['negate'] != 0
            or meta['resolution'] != .05 or meta['occupied_thresh'] != .65 or meta['free_thresh'] != .196):
        raise ValueError('Unexpected Nav2 YAML parameters')
    if len(meta['origin']) != 3 or not np.isfinite(meta['origin']).all() or meta['origin'][2] != 0:
        raise ValueError('Invalid origin')
    pixels = read_pgm(directory/meta['image'])
    if (directory/'map2d_nav2.yaml').exists():
        raise ValueError('Obsolete auxiliary YAML present')
    values = np.array([0,254,205],dtype=np.uint8)
    if nav2_decode(values,meta).tolist() != [100,0,-1]:
        raise ValueError('Nav2 unknown/free/occupied round-trip mismatch')
    stats = json.loads((directory/'map2d_stats.json').read_text())
    if pixels.shape != (stats['height_cells'],stats['width_cells']):
        raise ValueError('PGM/stats dimension mismatch')
    np.testing.assert_allclose(np.array(pixels.shape[::-1]) * meta['resolution'], stats['size_m'], atol=1e-9)
    np.testing.assert_allclose(meta['origin'][:2], stats['origin'], atol=1e-9)
    np.testing.assert_allclose(np.array(meta['origin'][:2])/meta['resolution'], np.round(np.array(meta['origin'][:2])/meta['resolution']), atol=1e-8)
    for name, value in [('occupied',0),('free',254),('unknown',205)]:
        if np.count_nonzero(pixels==value) != stats['cells'][name] or stats['cells'][name] == 0:
            raise ValueError(f'Wrong/empty {name} count')
    # Cross-check projected occupancy against the known final path. This detects
    # a vertical PGM flip even if file sizes/counts are otherwise valid.
    if check_sources:
        poses = np.loadtxt(ROOT/'results'/stats['run']/'keyframes_final_tum.txt')
        ij = np.floor((poses[:,1:3]-meta['origin'][:2])/meta['resolution']).astype(int)
        if np.any(ij < 0) or np.any(ij[:,0] >= pixels.shape[1]) or np.any(ij[:,1] >= pixels.shape[0]):
            raise ValueError('Trajectory outside grid bounds')
        path_values = pixels[pixels.shape[0]-1-ij[:,1],ij[:,0]]
        stats['trajectory_cells_free_fraction'] = float(np.mean(path_values == 254))
        if stats['trajectory_cells_free_fraction'] < MIN_TRAJECTORY_FREE_FRACTION:
            raise ValueError('Trajectory/free-space mismatch: check PGM orientation and origin')
        for path, expected in stats['source_sha256'].items():
            h = hashlib.sha256()
            with (ROOT/path).open('rb') as f:
                for block in iter(lambda:f.read(1024*1024),b''):
                    h.update(block)
            if h.hexdigest() != expected:
                raise ValueError(f'Source modified: {path}')
        for sources in stats.get('overlap_sources',{}).values():
            for path,expected in sources.items():
                with (ROOT/path).open('rb') as f:
                    actual=hashlib.file_digest(f,'sha256').hexdigest() if hasattr(hashlib,'file_digest') else hashlib.sha256(f.read()).hexdigest()
                if actual != expected:
                    raise ValueError(f'Overlap source changed: {path}')
        with Path(stats['alignment']).open('rb') as f:
            if hashlib.sha256(f.read()).hexdigest() != stats['alignment_sha256']:
                raise ValueError('Alignment provenance mismatch')
        if (directory/'map_full.pcd').read_bytes() != (ROOT/'results'/stats['run']/'map/GlobalMap.pcd').read_bytes():
            raise ValueError('Full map differs from source')
        import open3d as o3d
        cloud = o3d.io.read_point_cloud(str(directory/'map_0p10.pcd'))
        if len(cloud.points) != stats['voxel_points']:
            raise ValueError('Voxel PCD point count mismatch')
        cropped = np.asarray(o3d.io.read_point_cloud(str(directory/'map_crop_0p10.pcd')).points)
        lo,hi = np.asarray(stats['crop_bounds_xy'])
        if len(cropped) != stats['crop_points'] or np.any(cropped[:,:2]<lo-1e-6) or np.any(cropped[:,:2]>hi+1e-6):
            raise ValueError('Crop PCD bounds/count mismatch')
        full = np.asarray(cloud.points)
        alignment = json.loads(Path(stats['alignment']).read_text())
        r,t = np.array(alignment['R']), np.array(alignment['t'])
        aligned = np.asarray(o3d.io.read_point_cloud(str(directory/'map_0p10_enu.pcd')).points)
        np.testing.assert_allclose(aligned,full@r.T+t,atol=2e-5,rtol=0)
        aligned_crop = np.asarray(o3d.io.read_point_cloud(str(directory/'map_crop_0p10_enu.pcd')).points)
        np.testing.assert_allclose(aligned_crop,cropped@r.T+t,atol=2e-5,rtol=0)
        from export_maps import Config, driving_region
        cfg = Config(**stats['config'])
        region = driving_region(np.flipud(pixels),np.array(meta['origin'][:2]),poses,cfg)
        if region != stats['driving_region']:
            raise ValueError('Driving region statistics mismatch')
        if stats['stride10_baseline'] and region['unknown_fraction'] >= stats['stride10_baseline']['driving_region']['unknown_fraction']:
            raise ValueError('Stride 2 did not improve unknown fraction')
        for name in ('map_top.png','map_oblique.png','map2d.png','map_loop_overlap.png'):
            with Image.open(directory/name) as image:
                if min(image.size) < 1000 or np.asarray(image).std() < 5:
                    raise ValueError(f'Empty/low resolution image: {name}')
    return dict(valid=True, pgm_shape=list(pixels.shape), origin=meta['origin'],
                resolution=meta['resolution'], cells=stats['cells'],
                nav2_load_yaml='map2d.yaml', nav2_semantics_valid=True,
                requested_yaml_preserves_unknown=True,
                trajectory_cells_free_fraction=stats.get('trajectory_cells_free_fraction'))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory', type=Path)
    args = p.parse_args()
    result = validate(args.directory)
    (args.directory/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

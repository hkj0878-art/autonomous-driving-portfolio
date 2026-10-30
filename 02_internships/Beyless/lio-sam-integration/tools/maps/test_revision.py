"""TASK-06 rev1 tests, kept inside the authorized tools/maps scope."""
import json
import sys
from pathlib import Path
import numpy as np
import pytest
import yaml
sys.path.insert(0,str(Path(__file__).parent))
from export_maps import Config, trajectory_bounds, crop_points, driving_region, write_grid
from verify_maps import nav2_decode, validate


def test_single_yaml_preserves_unknown(tmp_path):
    pixels=np.array([[254,254,205],[0,205,205]],np.uint8)
    (tmp_path/'map2d_nav2.yaml').write_text('obsolete')
    write_grid(tmp_path,pixels,[-.1,.2],Config())
    assert not (tmp_path/'map2d_nav2.yaml').exists()
    meta=yaml.safe_load((tmp_path/'map2d.yaml').read_text())
    assert meta['free_thresh']==.196
    assert nav2_decode([0,254,205],meta).tolist()==[100,0,-1]
    (tmp_path/'map2d_stats.json').write_text(json.dumps(dict(width_cells=3,height_cells=2,size_m=[.15,.1],origin=[-.1,.2],cells=dict(occupied=1,free=2,unknown=3))))
    assert validate(tmp_path,False)['requested_yaml_preserves_unknown']


def test_crop_uses_trajectory_bounds():
    poses=np.array([[0,0,1,0],[1,10,5,0]])
    bounds=trajectory_bounds(poses,30)
    np.testing.assert_allclose(bounds,[[-30,-29],[40,35]])
    xyz=np.array([[-30,-29,100],[40,35,-100],[40.01,0,0],[0,35.01,0]])
    np.testing.assert_array_equal(crop_points(xyz,bounds),xyz[:2])


def test_buffer_follows_polyline_not_only_keyframes():
    poses=np.array([[0,0,0,0],[1,10,0,0]])
    cfg=Config(resolution=1,margin=1)
    pixels=np.full((6,12),205,np.uint8)
    pixels[2:4,:]=254
    result=driving_region(pixels,np.array([-1,-3]),poses,cfg)
    assert result['unknown_fraction']==0
    assert result['cells']['free']==24


def test_revision_defaults():
    cfg=Config()
    assert cfg.scan_stride==2 and cfg.margin==30 and cfg.free_thresh==.196

import json
from pathlib import Path
import sys
import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).parents[1]))
from maps.export_maps import Config, GroundModel, obstacle_mask, bresenham_batch, write_grid
from maps.verify_maps import validate, read_pgm, nav2_decode


def test_local_ground_removes_floor_and_high_returns():
    cfg = Config()
    points = []
    for x,z in [(0.,0.),(2.,1.)]:
        points.extend([[x+.1+i*.01,.1,z] for i in range(30)])
        points.extend([[x+.1,.1,z+.5],[x+.1,.1,z+2.5]])
    points = np.array(points)
    ground = GroundModel(points,cfg)
    mask = obstacle_mask(points,ground,cfg)
    assert np.flatnonzero(mask).tolist() == [30,62]
    np.testing.assert_allclose(ground.at(np.array([[.5,.5,0],[2.5,.5,0]])), [0,1])


def test_bresenham_stops_at_wall_and_leaves_unobserved_unknown():
    blocked = np.zeros((7,8),bool); blocked[2,4] = True
    free, hit = set(), set()
    for f,h in bresenham_batch([1,2],[[6,2]],blocked.shape,blocked):
        free.update(f); hit.update(h)
    assert free == {2*8+2,2*8+3}
    assert hit == {2*8+4}
    assert 2*8+5 not in free
    assert 2*8+1 not in free


@pytest.mark.parametrize('end',[[1,1],[6,1],[1,6],[6,6],[2,6],[6,2],[3,3]])
def test_bresenham_all_octants_and_endpoint(end):
    blocked = np.zeros((8,8),bool)
    visited = []
    for f,h in bresenham_batch([3,3],[end],blocked.shape,blocked):
        assert not len(h)
        visited.extend(f.tolist())
    assert len(visited) == max(abs(end[0]-3),abs(end[1]-3))
    if end != [3,3]:
        assert visited[-1] == end[1]*8+end[0]


def fixture_grid(tmp_path):
    pixels = np.array([[254,254,205],[0,205,205]],np.uint8)
    cfg = Config(); origin = [-.1,.2]
    write_grid(tmp_path,pixels,origin,cfg)
    stats = dict(width_cells=3,height_cells=2,size_m=[.15,.1],origin=origin,
                 cells=dict(occupied=1,free=2,unknown=3))
    (tmp_path/'map2d_stats.json').write_text(json.dumps(stats))
    return pixels


def test_nav2_yaml_pgm_and_vertical_orientation(tmp_path):
    pixels = fixture_grid(tmp_path)
    np.testing.assert_array_equal(read_pgm(tmp_path/'map2d.pgm'), pixels[::-1])
    assert validate(tmp_path,check_sources=False)['valid']
    meta = yaml.safe_load((tmp_path/'map2d.yaml').read_text())
    assert meta['origin'] == [-.1,.2,0.]


def test_validator_rejects_corrupt_pgm(tmp_path):
    fixture_grid(tmp_path)
    with (tmp_path/'map2d.pgm').open('ab') as f:
        f.write(b'x')
    with pytest.raises(ValueError,match='payload'):
        validate(tmp_path,check_sources=False)


def test_validator_rejects_wrong_resolution(tmp_path):
    fixture_grid(tmp_path)
    path = tmp_path/'map2d.yaml'
    meta = yaml.safe_load(path.read_text()); meta['resolution'] = .1
    path.write_text(yaml.safe_dump(meta))
    with pytest.raises(ValueError,match='parameters'):
        validate(tmp_path,check_sources=False)


def test_nav2_roundtrip_single_yaml_preserves_unknown(tmp_path):
    fixture_grid(tmp_path)
    meta = yaml.safe_load((tmp_path/'map2d.yaml').read_text())
    assert meta['free_thresh'] == .196
    assert not (tmp_path/'map2d_nav2.yaml').exists()
    assert nav2_decode(np.array([0,254,205]),meta).tolist() == [100,0,-1]

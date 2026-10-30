import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]))
import run_lio_sam as run

@pytest.mark.parametrize('name',['','/','SLAM/results','../x','foo/bar','x;touch pwn','x y','x$(id)','x`id`'])
def test_unsafe_names(name):
    with pytest.raises(ValueError): run.safe_destination(name)

def test_exact_destination_and_home():
    assert run.safe_destination('loop_on_r1')=='/SLAM/results/loop_on_r1/map'
    with pytest.raises(ValueError): run.safe_destination('loop_on_r1',home='/tmp')

def test_symlink_protection(tmp_path,monkeypatch):
    home=tmp_path/'home'; root=home/'SLAM'; (root/'results').mkdir(parents=True)
    monkeypatch.setattr(run,'ROOT',root)
    (root/'results'/'evil').symlink_to(tmp_path,target_is_directory=True)
    with pytest.raises(ValueError): run.safe_destination('evil',home)

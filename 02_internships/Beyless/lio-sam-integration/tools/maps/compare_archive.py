"""Recompute historical 30 m-buffer unknown fraction without changing sources."""
import json
from pathlib import Path
import numpy as np
from export_maps import ROOT, Config, driving_region, sha256
from verify_maps import read_pgm

if __name__ == '__main__':
    archive=ROOT/'results/maps/archive_v2/loop_on_r2'
    source=ROOT/'results/archive_v3_preloop_pairs/loop_on_r2/keyframes_final_tum.txt'
    old=json.loads((archive/'map2d_stats.json').read_text())
    expected=old['source_sha256']['results/loop_on_r2/keyframes_final_tum.txt']
    if sha256(source)!=expected:
        raise ValueError('Archived trajectory does not match old map provenance')
    region=driving_region(np.flipud(read_pgm(archive/'map2d.pgm')),
                          np.array(old['origin']),np.loadtxt(source),Config())
    result=dict(driving_region=region,trajectory_source=str(source.relative_to(ROOT)),trajectory_sha256=sha256(source))
    (archive.parent/'buffer_baseline.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

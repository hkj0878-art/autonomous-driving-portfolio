#!/usr/bin/env python3
import json
from pathlib import Path
root=Path(__file__).resolve().parents[1]
for directory in sorted((root/'results').iterdir()):
    if not directory.is_dir() or not (directory.name.startswith('loop_') or directory.name=='even_loop_on'): continue
    if (directory/'run_meta.json').exists():
        m=json.loads((directory/'run_meta.json').read_text())
        print(directory.name,'PASS' if m['success'] else 'FAIL', {k:m.get(k) for k in ('keyframes','loop_events','distance_m','endpoint_m','max_speed_mps','crash_detected')})
    elif (directory/'recorder_status.json').exists():
        m=json.loads((directory/'recorder_status.json').read_text()); print(directory.name,'RUNNING',m)

# TASK-06 rev1 archive

The former `results/maps/loop_on_r2/` is preserved unchanged here because it used
pre-v3 poses/maps. Its recorded source pose SHA256 matches
`results/archive_v3_preloop_pairs/loop_on_r2/keyframes_final_tum.txt`.
`buffer_baseline.json` measures the old PGM within that trajectory's 30 m buffer.
The legacy primary YAML uses 0.25 and decodes gray 205 as free in Nav2; the old
auxiliary YAML remains here solely as part of the historical artifact.
The regenerated v3 export uses a single primary YAML with free_thresh=0.196.

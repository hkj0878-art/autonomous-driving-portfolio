#!/usr/bin/env bash
# Run sequentially; each subprocess has its own failure metadata and cleanup.
set -o pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
RATE="${1:?Usage: run_experiments.sh <rate>}"
failed=0
for mode in on off; do
  params=config/params_base.yaml
  if [[ "$mode" == off ]]; then params=config/params_loop_off.yaml; fi
  for repeat in 1 2 3; do
    ./tools/run_lio_sam.sh "loop_${mode}_r${repeat}" "$params" data/lio_bag "$RATE" || failed=1
  done
done
./tools/run_lio_sam.sh even_loop_on config/params_base.yaml data/lio_bag_even "$RATE" || failed=1
source "$ROOT/env.sh"
/home/hong/SLAM/.venv/bin/python tools/verify_experiments.py || failed=1
exit "$failed"

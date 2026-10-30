#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/env.sh"
exec /home/hong/SLAM/.venv/bin/python "$ROOT/tools/run_lio_sam.py" "$@"

#!/bin/bash
# Container entry point: run_case.sh <case> <threads> [<threads> ...]
# /data/<case>/{facets.bin,rays.bin} is mounted read-only, /out is writable.
set -euo pipefail
case="${1:?case name}"
shift
mkdir -p /out
python -W ignore /bench/facets_to_h5m.py "/data/$case/facets.bin" "/out/$case.h5m" --report "/out/$case.build.json"
for t in "$@"; do
  /bench/dagmc_driver "/out/$case.h5m" "/data/$case/rays.bin" --threads "$t" | grep '^{' > "/out/$case.dagmc.t$t.json"
done

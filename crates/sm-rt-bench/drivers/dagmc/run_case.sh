#!/bin/bash
# Container entry point: run_case.sh <case> <threads> [<threads> ...]
# /data/<case>/{facets.bin,rays.bin} is mounted read-only. Rows (one JSON object per line) and the
# tracks line go to stdout; everything else goes to stderr. Environment: SMRT_TRACKER (row name),
# SMRT_MODE, SMRT_PRECISION, SMRT_REPS, SMRT_WARMUP (defaults: dagmc, the driver's backend, f64, 7, 3).
set -euo pipefail
case="${1:?case name}"
shift
work="$(mktemp -d)"
/bench/facets_to_h5m "/data/$case/facets.bin" "$work/$case.h5m" --report "$work/build.json" >&2
first=1
for t in "$@"; do
  extra=()
  [ "$first" = 1 ] && extra+=(--tracks)
  first=0
  /bench/dagmc_driver "$work/$case.h5m" "/data/$case/rays.bin" --threads "$t" \
      --tracker "${SMRT_TRACKER:-dagmc}" ${SMRT_MODE:+--mode "$SMRT_MODE"} --precision "${SMRT_PRECISION:-f64}" \
      --reps "${SMRT_REPS:-7}" --warmup "${SMRT_WARMUP:-3}" "${extra[@]}" | grep '^{'
done

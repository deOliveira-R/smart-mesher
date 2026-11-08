#!/bin/bash
# Container entry point: run_case.sh <case> <threads> [<threads> ...]
# /data/<case>/{facets.bin,rays.bin[,sigma.txt]} is mounted read-only. Rows (one JSON object per
# line) and the dump lines go to stdout; everything else goes to stderr. Environment:
# SMRT_TRACKER (row name), SMRT_MODE, SMRT_PRECISION, SMRT_REPS, SMRT_WARMUP (defaults: dagmc, the
# driver's backend, f64, 7, 3), SMRT_WORKLOADS (comma list of track, fire, fire-locate, flight;
# default track), SMRT_HISTORIES and SMRT_SEED (flight; defaults 10000 and 0x5eed). The dump line
# of each workload (tracks, fires, tally) is printed for the first thread count only.
set -euo pipefail
case="${1:?case name}"
shift
work="$(mktemp -d)"
/bench/facets_to_h5m "/data/$case/facets.bin" "$work/$case.h5m" --report "$work/build.json" >&2
IFS=',' read -r -a workloads <<< "${SMRT_WORKLOADS:-track}"
for w in "${workloads[@]}"; do
  first=1
  for t in "$@"; do
    extra=()
    [ "$first" = 1 ] && extra+=(--tracks)
    first=0
    if [ "$w" = flight ]; then
      extra+=(--histories "${SMRT_HISTORIES:-10000}" --seed "${SMRT_SEED:-0x5eed}" --sigma "/data/$case/sigma.txt")
    fi
    /bench/dagmc_driver "$work/$case.h5m" "/data/$case/rays.bin" --threads "$t" --workload "$w" \
        --tracker "${SMRT_TRACKER:-dagmc}" ${SMRT_MODE:+--mode "$SMRT_MODE"} --precision "${SMRT_PRECISION:-f64}" \
        --reps "${SMRT_REPS:-7}" --warmup "${SMRT_WARMUP:-3}" "${extra[@]}" | grep '^{'
  done
done

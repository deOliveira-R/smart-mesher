#!/bin/bash
# Container entry point: run_case.sh <case> <threads> [<threads> ...]
# /data/<case>/{facets.bin,rays.bin[,sigma.txt,tets.bin]} is mounted read-only. Rows (one JSON
# object per line) and the dump lines go to stdout; everything else goes to stderr. Environment:
# SMRT_TRACKER (row name), SMRT_MODE, SMRT_PRECISION (defaults: xdg, embree, mixed: Embree
# traverses in single precision), SMRT_REPS, SMRT_WARMUP (7, 3), SMRT_WORKLOADS (comma list of
# track, fire, fire-locate, flight, walk; default track), SMRT_HISTORIES and SMRT_SEED (flight;
# defaults 10000 and 0x5eed). track, fire, fire-locate and flight run on the facets of the case;
# walk runs on its tet mesh and is skipped (stderr note, no rows) for a case without tets.bin.
# The dump line of each workload is printed for the first thread count only.
set -euo pipefail
case="${1:?case name}"
shift
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
IFS=',' read -r -a workloads <<< "${SMRT_WORKLOADS:-track}"
for w in "${workloads[@]}"; do
  extra=()
  if [ "$w" = walk ]; then
    if [ ! -f "/data/$case/tets.bin" ]; then
      echo "walk: $case has no tets.bin; skipped" >&2
      continue
    fi
    [ -f "$work/tets.h5m" ] || /bench/smtets_to_moab "/data/$case/tets.bin" "$work/tets.h5m" --report "$work/tets.json" >&2
    model="$work/tets.h5m"
    extra+=(--tets "/data/$case/tets.bin")
  else
    [ -f "$work/facets.h5m" ] || /bench/facets_to_h5m "/data/$case/facets.bin" "$work/facets.h5m" --no-watertight --report "$work/build.json" >&2
    model="$work/facets.h5m"
  fi
  if [ "$w" = flight ]; then
    extra+=(--histories "${SMRT_HISTORIES:-10000}" --seed "${SMRT_SEED:-0x5eed}" --sigma "/data/$case/sigma.txt")
  fi
  first=1
  for t in "$@"; do
    dump=()
    [ "$first" = 1 ] && dump+=(--tracks)
    first=0
    /bench/xdg_driver "$model" "/data/$case/rays.bin" --threads "$t" --workload "$w" \
        --tracker "${SMRT_TRACKER:-xdg}" --mode "${SMRT_MODE:-embree}" --precision "${SMRT_PRECISION:-mixed}" \
        --reps "${SMRT_REPS:-7}" --warmup "${SMRT_WARMUP:-3}" "${dump[@]}" "${extra[@]}" | grep '^{'
  done
done

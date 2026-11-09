#!/usr/bin/env bash
# Hash census (docs/design/w4/plan.md section 3.3 and 7): measures, at the checked-out commit,
# every value of the hash population and prints one sorted line per value,
#   <group>|<key>|<os-arch or any>|<value>
# after a `#` header (commit, uname -sm, rustc -V, CI run). No date: two runs are byte-identical.
#
#   tools/census/census.sh                     > census.txt
#   tools/census/census.sh --compare FILE      prints keys that differ or are on one side only; exit 1 if any
#
# Options: --no-linux      skip the Linux rows (no gh; --compare then ignores linux-* rows)
# Env:     CENSUS_CI_RUN   GitHub run id for the Linux rows (default: the last green `ci` run on
#                          main; with --compare, the run id recorded in FILE)
#          CARGO_TARGET_DIR (honoured), RAYON_NUM_THREADS (passed through; output must not depend on it)
# Values come from running code: tools/platform/hashes.sh, the CLI on committed scripts, tests with
# --nocapture, smrt-bench, a small probe crate (tools/census/probe) for values no test prints,
# and sha256 of committed files. Needs a few cores and ~30 minutes (a debug build runs smrt-bench full).
set -euo pipefail
cd "$(dirname "$0")/../.."
root="$PWD"
here="$root/tools/census"

compare=""
linux=1
while [ $# -gt 0 ]; do
  case "$1" in
    --compare) compare="${2:?--compare needs a file}"; shift 2 ;;
    --no-linux) linux=0; shift ;;
    -h | --help) sed -n '2,16p' "$0"; exit 0 ;;
    *) echo "census.sh: unknown argument $1" >&2; exit 2 ;;
  esac
done
if [ -n "$compare" ]; then
  [ -f "$compare" ] || { echo "census.sh: no such file $compare" >&2; exit 2; }
  compare="$(cd "$(dirname "$compare")" && pwd)/$(basename "$compare")"
fi

tmp="$(mktemp -d "${TMPDIR:-/tmp}/census.XXXXXX")"
trap 'rm -rf "$tmp"' EXIT
census() { python3 -I "$here/census.py" "$@"; }

tgt="${CARGO_TARGET_DIR:-$root/target}"
smesh="$tgt/debug/smesh"

cargo build -p sm-cli -p sm-rt-bench --bin smesh --bin smrt-bench --locked >&2

# platform (62 recomputed lines of tools/platform/expected.txt plus its sha256), submap_graded, w3c_pin hexes
PLATFORM_SCRATCH="$tmp/platform" SMESH_SKIP_BUILD=1 tools/platform/hashes.sh "$tmp/h.txt" >&2
: > "$tmp/rows"
# shellcheck disable=SC2129
census platform tools/platform/expected.txt "$tmp/h.txt" >> "$tmp/rows"

# w3c_pin enclosed/leftout/warped scripts, mesh.auto_tet warnings
census w3c "$smesh" "$root" >> "$tmp/rows"

# p0_rows
census p0 "$smesh" >> "$tmp/rows"

# flat_golden on this machine
cargo test -p sm-rt --locked --all-features --test flat_golden -- --nocapture > "$tmp/flat.log" 2>&1 || {
  tail -30 "$tmp/flat.log" >&2
  echo "census.sh: flat_golden failed" >&2
  exit 1
}
census flat "$tmp/flat.log" >> "$tmp/rows"

# rtbench: every file hash of the three scales
: > "$tmp/rtb.txt"
for scale in tiny reduced full; do
  "$tgt/debug/smrt-bench" gen --scale "$scale" --print >> "$tmp/rtb.txt"
done
census rtbench "$tmp/rtb.txt" >> "$tmp/rows"

# export formats and format goldens
census export "$smesh" "$root" >> "$tmp/rows"

# probe: poly counts and acc7 hashes, DAGMC export reports, sm-csgio content hashes
osarch="$(uname -s | sed 's/Darwin/macos/; s/Linux/linux/')-$(uname -m | sed 's/arm64/aarch64/')"
# The probe is its own workspace; its lock is the workspace lock (same versions), never committed.
cp "$root/Cargo.lock" "$here/probe/Cargo.lock"
CARGO_TARGET_DIR="$tgt/census-probe" cargo build --quiet --manifest-path "$here/probe/Cargo.toml" >&2
"$tgt/census-probe/debug/census-probe" "$osarch" "$root/crates/sm-csgio/tests/fixtures/openmc" >> "$tmp/rows"

# committed files: conformance fixtures, csgio oracles, goldens
census files "$root" "$tmp/platform/w3a-emit" >> "$tmp/rows"

# Linux rows from the CI log
ciheader=""
if [ "$linux" = 1 ]; then
  run_id="${CENSUS_CI_RUN:-}"
  if [ -z "$run_id" ] && [ -n "$compare" ]; then
    run_id="$(sed -n 's/^# ci-run: \([0-9][0-9]*\) .*/\1/p' "$compare" | head -n 1)"
  fi
  census linux "$tmp" "$run_id" > "$tmp/linux.txt"
  ciheader="$(grep '^# ci-run:' "$tmp/linux.txt" || true)"
  grep -v '^#' "$tmp/linux.txt" >> "$tmp/rows"
else
  ciheader="# ci-run: not collected (--no-linux)"
fi

{
  echo "# smart-mesh hash census (tools/census/census.sh)"
  echo "# commit: $(git rev-parse HEAD)"
  echo "# uname: $(uname -sm)"
  echo "# rustc: $(rustc -V)"
  echo "$ciheader"
} > "$tmp/header"

if [ -n "$compare" ]; then
  census finish "$tmp/rows" "$tmp/header" > "$tmp/now.txt"
  if [ "$linux" = 1 ]; then
    census compare "$compare" "$tmp/now.txt"
  else
    census compare "$compare" "$tmp/now.txt" skip-linux
  fi
else
  census finish "$tmp/rows" "$tmp/header"
fi

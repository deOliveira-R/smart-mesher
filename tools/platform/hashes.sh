#!/usr/bin/env bash
# Print sorted `script|key|value` identity-hash lines for the milestone scripts (W2 milestone and
# fork, W3a hybrid and tet, W3b milestone, the SUBMAP_GRADED fixture, w3c_pin.smesh, and the
# information-only curved probe). Exits 1 if any step left an `error.*` key. Runs the same on macOS and in the Linux container (tools/platform/run_container.sh).
#
#   tools/platform/hashes.sh OUT_FILE [--extras]
#
# Env: SMESH_BIN (default target/debug/smesh; built with cargo unless SMESH_SKIP_BUILD=1),
#      CARGO_TARGET_DIR (honoured), RAYON_NUM_THREADS (passed through; the control runs use it).
# --extras also writes OUT_FILE.extras: platform line, flat_golden tuples, `smrt-bench gen
# --scale tiny --print` lines (raw-bit hashes, per platform by nature).
set -euo pipefail
cd "$(dirname "$0")/../.."
out="${1:?usage: hashes.sh OUT_FILE [--extras]}"
extras="${2:-}"
here="$PWD"
tgt="${CARGO_TARGET_DIR:-$here/target}"
smesh="${SMESH_BIN:-$tgt/debug/smesh}"
scratch="${PLATFORM_SCRATCH:-$tgt/platform-scratch}"
mkdir -p "$scratch"

if [ -z "${SMESH_SKIP_BUILD:-}" ]; then
  cargo build -p sm-cli --locked
fi
emit="$scratch/w3a-emit"
rm -rf "$emit"
MILESTONE_W3A_EMIT="$emit" cargo test -p sm-cli --locked --test milestone_w3a emit_conformance_scripts >/dev/null

: > "$out.tmp"
run() { # name script
  python3 tools/platform/collect.py "$smesh" "$1" "$2" "$scratch/run-$1" >> "$out.tmp"
}
run w2_milestone crates/sm-cli/tests/data/milestone.smesh
run w2_fork crates/sm-cli/tests/data/fork.smesh
run w3a_hybrid "$emit/w3a_hybrid.smesh"
run w3a_tet "$emit/w3a_tet.smesh"
run w3b_milestone tools/w3b_milestone.smesh
run w3b_submap_graded tools/platform/submap_graded.smesh
run curved_probe tools/platform/curved_probe.smesh   # information only (see platform-hashes.md)
run w3c_pin tools/w3c_pin.smesh
LC_ALL=C sort "$out.tmp" > "$out"
rm -f "$out.tmp"
echo "wrote $out ($(wc -l < "$out") lines)"
if grep -E '^[^|]*\|error\.' "$out"; then
  echo "hashes.sh: a step failed (error.* keys above); the table is not usable" >&2
  exit 1
fi

if [ "$extras" = "--extras" ]; then
  {
    echo "platform $(uname -s | tr A-Z a-z)-$(uname -m) $(getconf GNU_LIBC_VERSION 2>/dev/null || echo 'libc: macOS')"
    cargo test -p sm-rt --locked --all-features --test flat_golden -- --nocapture 2>&1 \
      | grep -E "\(structure hash, bundle hash\)|no recorded bundle|test result|panicked|moved" || true
    cargo run -q -p sm-rt-bench --locked --bin smrt-bench -- gen --scale tiny --print 2>&1 || true
  } > "$out.extras"
  echo "wrote $out.extras"
fi

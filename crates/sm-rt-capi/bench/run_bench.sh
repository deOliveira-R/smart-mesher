#!/usr/bin/env bash
# Record-only benchmark driver: DAGMC (Docker) vs sm-rt through the same rays, plus the C-ABI
# overhead over the analytic stub. See bench/README.md.
#
#   run_bench.sh --check-gen                 regenerate the cases and compare with expected_sha256.txt
#   run_bench.sh [--cases C1,C2,C3] [--rays N] [--threads N] [--skip-docker] [--skip-rust]
#
# Results: bench/results/<case>-<UTC timestamp>-<pid>.json, one file per case and run (never
# overwritten). Nothing here gates anything; the numbers are measurements, not claims.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/../../.." && pwd)"
cases="C1,C2,C3"
rays=200000
threads="$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)"
check_gen=0
skip_docker=0
skip_rust=0
image="${SMRT_DAGMC_IMAGE:-smrt-dagmc-bench}"

while [ $# -gt 0 ]; do
  case "$1" in
    --check-gen) check_gen=1 ;;
    --cases) cases="$2"; shift ;;
    --rays) rays="$2"; shift ;;
    --threads) threads="$2"; shift ;;
    --skip-docker) skip_docker=1 ;;
    --skip-rust) skip_rust=1 ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "run_bench.sh: unknown argument $1" >&2; exit 2 ;;
  esac
  shift
done

if [ "$check_gen" = 1 ]; then
  tmp="$(mktemp -d)"
  trap 'rm -rf "$tmp"' EXIT
  python3 "$here/gen_case.py" --out "$tmp" >/dev/null
  (cd "$tmp" && if command -v sha256sum >/dev/null 2>&1; then sha256sum -c "$here/expected_sha256.txt"; else shasum -a 256 -c "$here/expected_sha256.txt"; fi)
  echo "run_bench.sh --check-gen: gen_case.py output matches expected_sha256.txt"
  exit 0
fi

data="$here/data"
results="$here/results"
mkdir -p "$data" "$results"
run_id="$(date -u +%Y%m%dT%H%M%SZ)-$$"

echo "== generating cases ($cases, $rays rays)"
python3 "$here/gen_case.py" --out "$data" --cases "$cases" --rays "$rays"

# ---- DAGMC side -----------------------------------------------------------------------------
dagmc_ok=0
if [ "$skip_docker" = 1 ]; then
  echo "== DAGMC side SKIPPED (--skip-docker)"
elif ! command -v docker >/dev/null 2>&1 || ! docker info >/dev/null 2>&1; then
  echo "run_bench.sh: Docker is not available (is the daemon running?). The DAGMC side cannot run." >&2
  echo "              Re-run with --skip-docker to measure the Rust side only." >&2
  exit 2
else
  echo "== building $image"
  build_args=()
  if [ -n "${HTTPS_PROXY:-}" ]; then
    ca="${SSL_CERT_FILE:-/root/.ccr/ca-bundle.crt}"
    [ -f "$ca" ] && cp "$ca" "$here/dagmc/certs/ca.crt"
    build_args+=(--network=host --build-arg "HTTPS_PROXY=$HTTPS_PROXY" --build-arg "https_proxy=$HTTPS_PROXY")
  fi
  docker build "${build_args[@]}" -t "$image" "$here/dagmc"
  dagmc_ok=1
fi

IFS=',' read -r -a case_list <<<"$cases"
for c in "${case_list[@]}"; do
  work="$(mktemp -d)"
  if [ "$dagmc_ok" = 1 ]; then
    echo "== DAGMC $c (1 thread and $threads)"
    run_args=(--rm -v "$data:/data:ro" -v "$work:/out")
    [ -n "${HTTPS_PROXY:-}" ] && run_args+=(--network=host)
    thread_list=(1)
    [ "$threads" != 1 ] && thread_list+=("$threads")
    docker run "${run_args[@]}" "$image" "$c" "${thread_list[@]}"
  fi

  if [ "$skip_rust" = 0 ]; then
    echo "== C ABI over the analytic stub, $c"
    for t in 1 "$threads"; do
      (cd "$root" && cargo run --quiet --release -p sm-rt-capi --features stubs --example rt_capi_bench -- \
        --case "$c" --rays "$data/$c/rays.bin" --threads "$t") >"$work/$c.capi.t$t.json"
      [ "$threads" = 1 ] && break
    done
    if [ -f "$root/crates/sm-rt/examples/trace_rays.rs" ]; then
      echo "== sm-rt trace_rays, $c"
      for t in 1 "$threads"; do
        (cd "$root" && cargo run --quiet --release -p sm-rt --example trace_rays -- \
          "$data/$c/facets.bin" "$data/$c/rays.bin" --threads "$t") >"$work/$c.smrt.t$t.json" \
          || echo '{"error":"trace_rays failed"}' >"$work/$c.smrt.t$t.json"
        [ "$threads" = 1 ] && break
      done
    else
      echo "== sm-rt side SKIPPED: crates/sm-rt/examples/trace_rays.rs does not exist yet (sm-rt not on main)"
    fi
  fi

  python3 - "$work" "$c" "$run_id" "$results" "$threads" <<'PY'
import glob, json, os, sys, datetime
work, case, run_id, results, threads = sys.argv[1:6]
def load(pattern):
    out = {}
    for p in sorted(glob.glob(os.path.join(work, pattern))):
        key = os.path.basename(p).rsplit(".", 2)[-2]  # t1 / tN
        with open(p) as f:
            out[key] = json.load(f)
    return out
merged = {
    "case": case,
    "run_id": run_id,
    "date_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
    "threads_requested": int(threads),
    "dagmc": load(f"{case}.dagmc.t*.json"),
    "dagmc_build": None,
    "sm_rt": load(f"{case}.smrt.t*.json") or {"skipped": "sm-rt trace_rays example not available"},
    "capi_stub": load(f"{case}.capi.t*.json"),
    "note": "record-only measurements; the stub numbers are analytic overhead numbers, not competitor numbers",
}
b = os.path.join(work, f"{case}.build.json")
if os.path.exists(b):
    merged["dagmc_build"] = json.load(open(b))
if not merged["dagmc"]:
    merged["dagmc"] = {"skipped": "DAGMC side not run"}
path = os.path.join(results, f"{case}-{run_id}.json")
with open(path, "x") as f:
    json.dump(merged, f, indent=2, sort_keys=True)
    f.write("\n")
print("wrote", path)
PY
  rm -rf "$work"
done

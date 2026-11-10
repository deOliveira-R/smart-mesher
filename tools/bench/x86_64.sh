#!/usr/bin/env bash
# One benchmark session of smart-mesh's ray-tracing bench on a bare-metal Linux x86_64 machine
# (ruling D6). Runbook: docs/design/w4/bench-x86.md. Run it three times on three UTC days.
#
#   tools/bench/x86_64.sh [--dry] [--smoke] [--out DIR] [--cases P1,H1,P2]
#
#   --dry    run the host self-check only, build and run nothing
#   --smoke  do not refuse a host that fails the self-check (the report then says SMOKE RESULT);
#            the output directory gets a -SMOKE suffix and is not a session
#   --out    parent directory of the session directory (default: $HOME/smrt-bench-sessions)
#   --cases  cases to run (default P1,H1,P2)
#
# Exit codes: 0 ok, 1 a stage failed or a run reported disagreements, 2 usage,
# 3 the host is refused (the first reason is printed as "refused: ...").
set -euo pipefail

usage() { sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'; }

dry=0
smoke=0
parent="${HOME:-.}/smrt-bench-sessions"
cases="P1,H1,P2"
while [ $# -gt 0 ]; do
  case "$1" in
    --dry) dry=1 ;;
    --smoke) smoke=1 ;;
    --out)
      [ $# -ge 2 ] || { usage; exit 2; }
      parent="$2"
      shift
      ;;
    --cases)
      [ $# -ge 2 ] || { usage; exit 2; }
      cases="$2"
      shift
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# ---- host self-check -------------------------------------------------------------------------
refusals=()
warnings=()
refuse() { refusals+=("$1"); }

os="$(uname -s)"
arch="$(uname -m)"
if [ "$arch" != "x86_64" ]; then
  # The first thing a user on the wrong machine sees; checked before anything Linux-specific.
  echo "refused: not x86_64 (this host is $os $arch). The published table comes from bare-metal Linux x86_64 only." >&2
  [ "$smoke" -eq 1 ] || exit 3
  refuse "not x86_64 ($os $arch)"
fi
if [ "$os" != "Linux" ]; then
  echo "refused: not Linux (this host is $os $arch)" >&2
  [ "$smoke" -eq 1 ] || exit 3
  refuse "not Linux ($os)"
fi

virt="unknown"
if command -v systemd-detect-virt >/dev/null 2>&1; then
  # Prints "none" and exits non-zero on bare metal.
  virt="$(systemd-detect-virt --vm 2>/dev/null || true)"
  [ -n "$virt" ] || virt="unknown"
else
  refuse "systemd-detect-virt is missing, so bare metal cannot be confirmed (install systemd)"
fi
[ "$virt" = "none" ] || refuse "not bare metal (systemd-detect-virt --vm says: $virt)"

governors=""
for f in /sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor; do
  if [ -r "$f" ]; then
    governors="$governors$(cat "$f")"$'\n'
  fi
done
governor_set="$(printf '%s' "$governors" | sort -u | paste -sd, -)"
if [ -z "$governor_set" ]; then
  refuse "no cpufreq governor readable; cannot confirm 'performance'"
elif [ "$governor_set" != "performance" ]; then
  refuse "governor is '$governor_set', not performance (fix: sudo cpupower frequency-set -g performance)"
fi

for t in docker cargo git taskset lscpu tar sha256sum; do
  command -v "$t" >/dev/null 2>&1 || refuse "missing tool: $t"
done
if command -v docker >/dev/null 2>&1 && ! docker info >/dev/null 2>&1; then
  refuse "docker daemon not reachable for this user (docker info fails)"
fi

smt="n/a"
if [ -r /sys/devices/system/cpu/smt/active ]; then
  smt="$(cat /sys/devices/system/cpu/smt/active)"
fi
if [ "$smt" = "1" ]; then
  warnings+=("SMT is on; the plan recommends off (echo off | sudo tee /sys/devices/system/cpu/smt/control). One thread per physical core is used either way")
fi
isolated="$(cat /sys/devices/system/cpu/isolated 2>/dev/null || true)"
mem_kb="$(awk '/^MemTotal:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0)"
if [ "${mem_kb:-0}" -lt 33554432 ]; then
  warnings+=("less than 32 GiB of RAM ($((${mem_kb:-0} / 1024 / 1024)) GiB); the full scale may not fit")
fi

# Cores: the first logical CPU of every physical core. The same list pins native runs
# (taskset) and containers (--cpuset-cpus, through the docker shim below).
cores=""
ncores=0
if command -v lscpu >/dev/null 2>&1; then
  cores="$(lscpu -p=CPU,CORE,SOCKET 2>/dev/null | awk -F, '!/^#/ && !seen[$2 "," $3]++ {printf "%s%s", (n++ ? "," : ""), $1}')"
  if [ -n "$cores" ]; then
    ncores="$(printf '%s' "$cores" | tr ',' '\n' | wc -l | tr -d ' ')"
  fi
fi
[ -n "$cores" ] || refuse "cannot read the physical core list from lscpu"
export SMRT_BENCH_CORES="$cores"

utc_day="$(date -u +%Y-%m-%d)"
session="$parent/session-$utc_day"
[ "$smoke" -eq 0 ] || session="$session-SMOKE"

echo "host    : $os $arch, kernel $(uname -r)"
echo "cpu     : $(lscpu 2>/dev/null | awk -F: '/Model name/ {gsub(/^ +/, "", $2); print $2; exit}')"
echo "virt    : $virt"
echo "governor: ${governor_set:-unreadable}"
echo "smt     : $smt   isolated cores: ${isolated:-none}"
echo "cores   : $ncores physical, list $cores"
echo "session : $session (UTC day $utc_day)"
# Every result file carries its own date_utc; a session spanning 00:00 UTC is refused by the report.
midnight_local="$(date -d "$utc_day 00:00 UTC + 1 day" '+%H:%M %Z on %Y-%m-%d' 2>/dev/null || echo "unknown (GNU date missing)")"
now_utc="$(date -u +%H:%M)"
warnings+=("UTC day rollover: it is $now_utc UTC now. The whole session must finish before 00:00 UTC (= $midnight_local local time), or the report refuses it and the session is lost. The first session also builds the images. If in doubt, Ctrl-C now and start earlier")
for w in "${warnings[@]+"${warnings[@]}"}"; do echo "warning : $w"; done
for r in "${refusals[@]+"${refusals[@]}"}"; do echo "refused : $r"; done

if [ "${#refusals[@]}" -gt 0 ]; then
  if [ "$smoke" -eq 0 ]; then
    echo "refused: this host cannot produce a published table. Fix the lines above, or use --smoke for a labelled smoke run." >&2
    exit 3
  fi
  echo "smoke: continuing; the report will say SMOKE RESULT and this directory is not a session"
fi
if [ "$dry" -eq 1 ]; then
  echo "dry: host check done; nothing was built or run"
  exit 0
fi

# ---- session ----------------------------------------------------------------------------------
if [ -e "$session" ] && [ -n "$(ls -A "$session" 2>/dev/null)" ]; then
  echo "refused: $session already exists and is not empty (one session per UTC day; never mix the files of two runs)" >&2
  exit 3
fi
dirty="$(git -C "$repo" status --porcelain --untracked-files=no)"
if [ -n "$dirty" ] && [ "$smoke" -eq 0 ]; then
  echo "refused: the checkout has local changes; the three sessions must run one clean commit" >&2
  exit 3
fi
mkdir -p "$session/results"
commit="$(git -C "$repo" rev-parse HEAD)"
echo "$commit" >"$session/commit.txt"
exec > >(tee -a "$session/session.log") 2>&1
started="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo "== session start $started, commit $commit"

# Containers: no network, pinned to the same cores as the native run. Only `docker run` is
# touched; image builds keep their network (they are not part of the timed run).
shim="$(mktemp -d "${TMPDIR:-/tmp}/smrt-docker-shim.XXXXXX")"
# check_days: every results/*.json must carry the same UTC day in date_utc (and the start day).
# shellcheck disable=SC2317  # runs from the EXIT trap
check_days() {
  local days n
  days="$(grep -ho '"date_utc" *: *"[0-9-]*' "$session"/results/*.json 2>/dev/null | grep -o '[0-9]\{4\}-[0-9]\{2\}-[0-9]\{2\}$' | sort -u | paste -sd' ' -)" || true
  n="$(printf '%s' "$days" | wc -w | tr -d ' ')"
  if [ "$n" -eq 0 ]; then
    echo "UTC days: no date_utc found in results/*.json (no result files)"
  elif [ "$n" -gt 1 ]; then
    echo "UTC DAY MISMATCH: the result files fall on different UTC days ($days). The report refuses this session: it is NOT usable. Run it again on a later day, starting earlier."
  elif [ "$days" != "$utc_day" ]; then
    echo "UTC DAY MISMATCH: the session started on $utc_day but the result files say $days. It is NOT usable as one session. Run it again on a later day."
  else
    echo "UTC days: all result files are on $days (ok)"
  fi
}
# On every exit (also a failed stage) the session is checksummed and packed for sending.
# shellcheck disable=SC2317  # runs from the EXIT trap
finish() {
  local rc=$? tarball
  trap - EXIT
  rm -rf "$shim"
  echo "== session end $(date -u +%Y-%m-%dT%H:%M:%SZ), exit $rc"
  (cd "$session" && find results -type f | sort | xargs sha256sum >SHA256SUMS) || true
  check_days
  tarball="$parent/smrt-bench-$(basename "$session").tar.gz"
  tar -C "$parent" -czf "$tarball" "$(basename "$session")" && echo "SEND BACK: $tarball"
  exit "$rc"
}
trap finish EXIT
real_docker="$(command -v docker)"
cat >"$shim/docker" <<SHIM
#!/bin/sh
if [ "\${1:-}" = run ]; then
  shift
  exec "$real_docker" run --network none --cpuset-cpus "\$SMRT_BENCH_CORES" "\$@"
fi
exec "$real_docker" "\$@"
SHIM
chmod +x "$shim/docker"
export PATH="$shim:$PATH"

cd "$repo"
stage_fail=0
# stage NAME COMMAND...: run, time and record one stage; returns the command's exit code.
stage() {
  local name="$1" t0 t1 rc=0
  shift
  t0=$(date +%s)
  echo "== [$name] $*"
  "$@" || rc=$?
  t1=$(date +%s)
  echo "== [$name] exit $rc after $((t1 - t0)) s"
  echo "$name $rc $((t1 - t0))" >>"$session/stages.txt"
  return "$rc"
}

{
  echo "commit $commit"
  echo "date_utc $started"
  uname -a
  lscpu
  echo "virt $virt"
  echo "governors $governor_set"
  echo "smt $smt"
  echo "isolated ${isolated:-none}"
  echo "cores $cores"
  docker version 2>&1 || true
  cargo --version
  rustc --version
} >"$session/host.txt" 2>&1

stage build cargo build --release --locked -p sm-rt-bench || {
  echo "build failed" >&2
  exit 1
}
bin="$repo/target/release/smrt-bench"

# shellcheck disable=SC2317  # both are run through `stage`
docker_check() { "$bin" docker-check | tee "$session/docker-check.txt"; }
# shellcheck disable=SC2317
gen_print() {
  local s
  for s in reduced full; do
    echo "# scale $s"
    taskset -c "$cores" "$bin" gen --scale "$s" --cases "$cases" --print || return 1
  done >"$session/gen-print.txt"
}

stage self-test "$bin" self-test || stage_fail=1
# The images are built here (network: conda, git clones); they are pinned in docker/pins.env.
stage docker-check docker_check || {
  echo "an image did not build: no competitor row is possible, stopping" >&2
  exit 1
}
# The linux-x86_64 conda locks (docker/locks/README.md): untracked files, so the clean check is
# unaffected. Copied into the session so they are in the tarball.
# shellcheck disable=SC2317  # run through `stage`
lock_images() {
  local k
  for k in dagmc-moab dagmc-dd; do
    "$bin" docker-lock "$k" || return 1
    cp "$repo/crates/sm-rt-bench/docker/locks/$k.linux-x86_64.txt" "$session/" || return 1
  done
}
stage docker-lock lock_images || stage_fail=1
# Linux hash lines for blessing (unblessed until an orchestrator ruling).
stage gen-print gen_print || stage_fail=1

rc=0
stage run taskset -c "$cores" "$bin" run --scale full --cases "$cases" \
  --trackers smrt,dagmc-moab,dagmc-dd,xdg --threads "1,$ncores" --reps 7 --warmup 3 \
  --out "$session/results" || rc=$?
[ "$rc" -eq 0 ] || stage_fail=1
stage report "$bin" report --results "$session/results" --out "$session/report.md" || stage_fail=1

echo "run exit code $rc (0 ok, 1 disagreements, 2 something unavailable); stages in $session/stages.txt"
exit "$stage_fail"

#!/usr/bin/env bash
# External conformance of the exporters (docs/design/w3/plan.md, W3-INT section 3).
# NOT part of tools/ci.sh. Our validator judges meshes; the external tools judge exporters only:
# a fixture whose mesh fails `smesh validate` is reported as OUR BUG and skips the tools.
#
#   SM_CONFORMANCE=1 tools/conformance.sh [--only openfoam|meshio|exodus] [--keep]
#
# Without SM_CONFORMANCE=1 it prints 'conformance skipped' and exits 0. A missing prerequisite
# (docker + opencfd/openfoam-default:2506, a Python with venv) exits 3 unless
# SM_CONFORMANCE_ALLOW_SKIP=1 (then those checks are reported as skipped). Exit 0 only when
# every mandatory check that ran passed; 1 otherwise. Work dir: target/conformance/<utc>-<pid>/
# with report.json (removed on success unless --keep; kept on failure).
set -uo pipefail
export PYTHONDONTWRITEBYTECODE=1

if [ "${SM_CONFORMANCE:-}" != "1" ]; then
    echo "conformance skipped (set SM_CONFORMANCE=1 to run tools/conformance.sh)"
    exit 0
fi

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TOOLS="$ROOT/tools/conformance"
IMAGE="${SM_CONFORMANCE_IMAGE:-opencfd/openfoam-default:2506}"
ONLY=""
KEEP=0
while [ $# -gt 0 ]; do
    case "$1" in
        --only) ONLY="${2:-}"; shift 2 ;;
        --keep) KEEP=1; shift ;;
        -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done
case "$ONLY" in
    ""|openfoam|meshio|exodus) ;;
    *) echo "--only takes openfoam, meshio or exodus" >&2; exit 2 ;;
esac
want() { [ -z "$ONLY" ] || [ "$ONLY" = "$1" ]; }

WORK="$ROOT/target/conformance/$(date -u +%Y%m%dT%H%M%SZ)-$$"
mkdir -p "$WORK"
LOG="$WORK/conformance.log"
echo "work dir: $WORK"
say() { echo "$*" | tee -a "$LOG"; }

# ---------------------------------------------------------------- prerequisites
SKIP_FOAM=""
SKIP_PY=""
missing() {
    if [ "${SM_CONFORMANCE_ALLOW_SKIP:-}" = "1" ]; then
        say "prerequisite missing (skipped): $1"
        return 0
    fi
    say "prerequisite missing: $1"
    exit 3
}
DOCKER_PLATFORM=()
if want openfoam; then
    if ! command -v docker >/dev/null 2>&1; then
        missing "docker"; SKIP_FOAM="no docker"
    elif ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
        missing "docker image $IMAGE (docker pull $IMAGE)"; SKIP_FOAM="no image $IMAGE"
    else
        arch="$(docker image inspect --format '{{.Architecture}}' "$IMAGE")"
        if [ "$arch" = "amd64" ] && [ "$(uname -m)" = "arm64" ]; then
            DOCKER_PLATFORM=(--platform linux/amd64)
        fi
    fi
fi
VENV="$ROOT/target/conformance/venv"
PY="$VENV/bin/python"
if want meshio || want exodus; then
    if [ ! -x "$PY" ]; then
        base="${SM_CONFORMANCE_PYTHON:-}"
        if [ -z "$base" ]; then
            for c in python3.13 python3.12 python3.11 python3; do
                if command -v "$c" >/dev/null 2>&1; then base="$c"; break; fi
            done
        fi
        if [ -z "$base" ] || ! "$base" -m venv "$VENV" >>"$LOG" 2>&1 \
            || ! "$VENV/bin/pip" install -q -r "$TOOLS/requirements.txt" >>"$LOG" 2>&1; then
            rm -rf "$VENV"
            missing "python venv from tools/conformance/requirements.txt"; SKIP_PY="no python venv"
        fi
    elif ! "$PY" -c "import meshio, netCDF4, numpy, vtk" >>"$LOG" 2>&1; then
        missing "python packages of tools/conformance/requirements.txt in $VENV"; SKIP_PY="venv incomplete"
    fi
fi

# ---------------------------------------------------------------- build and fixtures
say "building smesh (release)"
if ! (cd "$ROOT" && cargo build --release -p sm-cli --bin smesh) >>"$LOG" 2>&1; then
    say "FAIL: cargo build --release -p sm-cli"; exit 1
fi
SMESH="${CARGO_TARGET_DIR:-$ROOT/target}/release/smesh"
mkdir -p "$WORK/fixtures"
cp "$TOOLS"/fixtures/*.smesh "$WORK/fixtures/"
say "emitting the milestone fixtures w3a_hybrid and w3a_tet"
if ! (cd "$ROOT" && MILESTONE_W3A_EMIT="$WORK/fixtures" cargo test -q -p sm-cli --test milestone_w3a emit_conformance_scripts) >>"$LOG" 2>&1 \
    || [ ! -f "$WORK/fixtures/w3a_hybrid.smesh" ] || [ ! -f "$WORK/fixtures/w3a_tet.smesh" ]; then
    say "FAIL: could not emit w3a_hybrid / w3a_tet"; exit 1
fi

RESULTS="$WORK/results"
mkdir -p "$RESULTS"
result() { # result <file> <status> <message>
    printf '{"status": "%s", "problems": [%s], "facts": {}}\n' "$2" "$3" > "$1"
}

# ---------------------------------------------------------------- one mesh
check_mesh() { # check_mesh <fixture> <label> <mesh id> <project dir>
    local fx="$1" label="$2" mesh="$3" proj="$4"
    local d="$WORK/$label"
    mkdir -p "$d"
    printf '{"fixture": "%s", "label": "%s", "mesh": "%s"}\n' "$fx" "$label" "$mesh" > "$RESULTS/$label.meta.json"
    # (0) our validator first.
    "$SMESH" validate -p "$proj" --mesh "$mesh" --json > "$d/validate.json" 2>>"$LOG"
    local vrc=$?
    if [ $vrc -ne 0 ]; then
        say "  $label: OUR BUG (smesh validate exit $vrc: $(head -c 300 "$d/validate.json")); external tools skipped"
        result "$RESULTS/$label.validate.json" fail "\"OUR BUG: smesh validate exited $vrc\""
        return
    fi
    result "$RESULTS/$label.validate.json" pass ""
    "$SMESH" query mesh.info "{\"mesh\": \"$mesh\"}" -p "$proj" --json > "$d/info.json" 2>>"$LOG"
    "$SMESH" quality -p "$proj" --mesh "$mesh" --profile openfoam --json > "$d/quality.json" 2>>"$LOG"
    "$SMESH" export "$d/$label.exo" -p "$proj" --mesh "$mesh" --json > "$d/export_exo.json" 2>>"$LOG"
    "$SMESH" export "$d/$label.vtu" -p "$proj" --mesh "$mesh" --json > "$d/export_vtu.json" 2>>"$LOG"
    "$SMESH" export "$d/$label.smsh" -p "$proj" --mesh "$mesh" --json > "$d/export_smsh.json" 2>>"$LOG"
    "$SMESH" export "$d/case" --format openfoam -p "$proj" --mesh "$mesh" --json > "$d/export_foam.json" 2>>"$LOG"
    # (1) OpenFOAM checkMesh.
    if want openfoam; then
        if [ -n "$SKIP_FOAM" ]; then
            result "$RESULTS/$label.openfoam.json" skipped "\"$SKIP_FOAM\""
        else
            # checkMesh needs system/controlDict; the exporter writes constant/polyMesh only, so
            # the tool runs on a copy with a minimal system/ (the exported files are not touched).
            rm -rf "$d/foamcase"
            mkdir -p "$d/foamcase/system"
            cp -R "$d/case/constant" "$d/foamcase/"
            cp "$TOOLS"/foam_system/* "$d/foamcase/system/"
            docker run --rm ${DOCKER_PLATFORM[@]+"${DOCKER_PLATFORM[@]}"} --user "$(id -u):$(id -g)" -v "$d/foamcase:/case" -w /case "$IMAGE" \
                bash -lc '. $(ls -d /usr/lib/openfoam/openfoam*/etc/bashrc | head -1) && checkMesh -case /case -allGeometry -allTopology' \
                > "$d/checkMesh.log" 2>&1
            local mode=""
            [ "$fx" = "hexbox" ] && mode="exact0"
            if ! python3 "$TOOLS/check_openfoam.py" "$RESULTS/$label.openfoam.json" "$d/checkMesh.log" "$d/case" "$d/info.json" "$d/quality.json" $mode >>"$LOG" 2>&1; then
                result "$RESULTS/$label.openfoam.json" fail "\"check_openfoam.py crashed (see conformance.log)\""
            fi
        fi
    fi
    # (2) meshio / VTK and (3) netCDF4.
    for tool in meshio exodus; do
        want $tool || continue
        if [ -n "$SKIP_PY" ]; then
            result "$RESULTS/$label.$tool.json" skipped "\"$SKIP_PY\""
            continue
        fi
        if [ $tool = meshio ]; then
            "$PY" "$TOOLS/check_meshio.py" "$RESULTS/$label.meshio.json" "$d/$label.exo" "$d/$label.vtu" "$d/$label.smsh" >>"$LOG" 2>&1
        else
            "$PY" "$TOOLS/check_exodus.py" "$RESULTS/$label.exodus.json" "$d/$label.exo" "$d/$label.smsh" >>"$LOG" 2>&1
        fi
        if [ $? -ne 0 ]; then
            result "$RESULTS/$label.$tool.json" fail "\"check_$tool.py crashed (see conformance.log)\""
        fi
    done
}

# ---------------------------------------------------------------- fixtures
for fx in hexbox wedgeprism tetbox polydual w3a_hybrid w3a_tet; do
    say "fixture $fx"
    proj="$WORK/proj_$fx"
    "$SMESH" new "$proj" >>"$LOG" 2>&1
    if ! "$SMESH" run "$WORK/fixtures/$fx.smesh" -p "$proj" --json > "$WORK/run_$fx.json" 2>>"$LOG"; then
        say "  $fx: OUR BUG (the fixture script failed: $(head -c 400 "$WORK/run_$fx.json"))"
        printf '{"fixture": "%s", "label": "%s", "mesh": ""}\n' "$fx" "$fx" > "$RESULTS/$fx.meta.json"
        result "$RESULTS/$fx.validate.json" fail "\"OUR BUG: the fixture script failed\""
        continue
    fi
    python3 -c 'import json,sys; [print(l) for l in json.load(open(sys.argv[1]))["output"].splitlines() if l.startswith("check ")]' \
        "$WORK/run_$fx.json" > "$WORK/checks_$fx.txt"
    while read -r _ label mesh; do
        say "  check $label ($mesh)"
        check_mesh "$fx" "$label" "$mesh" "$proj"
    done < "$WORK/checks_$fx.txt"
done

# ---------------------------------------------------------------- report
python3 - "$RESULTS" "$WORK/report.json" "$IMAGE" "$ONLY" <<'EOF'
import glob, json, os, sys
res, out, image, only = sys.argv[1:5]
checks = {}
for p in sorted(glob.glob(os.path.join(res, "*.json"))):
    label, kind = os.path.basename(p)[:-5].rsplit(".", 1)
    checks.setdefault(label, {})[kind] = json.load(open(p))
rows, summary = [], {"pass": 0, "fail": 0, "skipped": 0}
for label, parts in checks.items():
    meta = parts.pop("meta", {})
    row = {"fixture": meta.get("fixture"), "label": label, "mesh": meta.get("mesh")}
    for kind, r in sorted(parts.items()):
        row[kind] = r
        summary[r["status"]] = summary.get(r["status"], 0) + 1
    rows.append(row)
report = {"image": image, "only": only or "all", "mandatory_ok": summary["fail"] == 0, "summary": summary, "checks": rows}
json.dump(report, open(out, "w"), indent=1)
for row in rows:
    parts = ", ".join(f"{k} {row[k]['status']}" for k in ("validate", "openfoam", "meshio", "exodus") if k in row)
    print(f"  {row['label']:12s} {parts}")
    for k in ("validate", "openfoam", "meshio", "exodus"):
        for p in row.get(k, {}).get("problems", []):
            print(f"      {k}: {p}")
print(f"summary: {summary}; mandatory_ok: {report['mandatory_ok']}")
EOF
ok=$(python3 -c 'import json,sys; print(1 if json.load(open(sys.argv[1]))["mandatory_ok"] else 0)' "$WORK/report.json")
say "report: $WORK/report.json"
if [ "$ok" = "1" ]; then
    say "conformance passed"
    if [ $KEEP -eq 0 ]; then
        cp "$WORK/report.json" "$ROOT/target/conformance/last-report.json"
        rm -rf "$WORK"
        echo "work dir removed (--keep keeps it); report copied to target/conformance/last-report.json"
    fi
    exit 0
fi
say "conformance FAILED (work dir kept: $WORK)"
exit 1

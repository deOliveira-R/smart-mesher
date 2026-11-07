#!/usr/bin/env bash
# External conformance of the exporters (docs/design/w3/plan.md, W3-INT section 3).
# NOT part of tools/ci.sh. Our validator judges meshes; the external tools judge exporters only:
# a fixture whose mesh fails `smesh validate` is reported as OUR BUG and skips the tools.
#
#   SM_CONFORMANCE=1 tools/conformance.sh [--only openfoam|meshio|exodus|dagmc|gmsh] [--keep]
#
# Without SM_CONFORMANCE=1 it prints 'conformance skipped' and exits 0. A missing prerequisite
# (docker + opencfd/openfoam-default:2506, docker + smesh-dagmc-conformance (Tier B of --only
# dagmc: docker build -t smesh-dagmc-conformance tools/conformance/dagmc), a Python with venv)
# exits 3 unless SM_CONFORMANCE_ALLOW_SKIP=1 (then those checks are reported as skipped).
# `--only dagmc` and `--only gmsh` check the W3b exporters: Tier A (h5py, gmsh, meshio in the
# venv) on the sm-meshio export_conformance cases and on tools/w3b_milestone.smesh, Tier B
# (MOAB / DAGMC in docker) on the two-volume file. Exit 0 only when
# every mandatory check that ran passed; 1 otherwise. Work dir: target/conformance/<utc>-<pid>/
# with report.json (removed on success unless --keep; kept on failure).
set -uo pipefail
# check_openfoam.py gets the label: polydual / w3a_poly record the INT-9 checkMesh warnings
# (report.json `openfoam_warnings`); INT-8 splits are read from smeshCellMap
# (`openfoam_int8_splits`).
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
        -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done
case "$ONLY" in
    ""|openfoam|meshio|exodus|dagmc|gmsh) ;;
    *) echo "--only takes openfoam, meshio, exodus, dagmc or gmsh" >&2; exit 2 ;;
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
IMAGE_DAGMC="${SM_CONFORMANCE_DAGMC_IMAGE:-smesh-dagmc-conformance}"
SKIP_DAGMC_DOCKER=""
DOCKER_PLATFORM_DAGMC=()
if want dagmc; then
    if ! command -v docker >/dev/null 2>&1; then
        missing "docker (Tier B of --only dagmc)"; SKIP_DAGMC_DOCKER="no docker"
    elif ! docker image inspect "$IMAGE_DAGMC" >/dev/null 2>&1; then
        missing "docker image $IMAGE_DAGMC (docker build -t $IMAGE_DAGMC tools/conformance/dagmc)"; SKIP_DAGMC_DOCKER="no image $IMAGE_DAGMC"
    else
        arch="$(docker image inspect --format '{{.Architecture}}' "$IMAGE_DAGMC")"
        if [ "$arch" = "amd64" ] && [ "$(uname -m)" = "arm64" ]; then
            DOCKER_PLATFORM_DAGMC=(--platform linux/amd64)
        fi
    fi
fi
VENV="$ROOT/target/conformance/venv"
PY="$VENV/bin/python"
# Python modules the wanted checks import.
PYMODS="numpy"
if want meshio || want exodus; then PYMODS="$PYMODS, meshio, netCDF4, vtk"; fi
if want dagmc; then PYMODS="$PYMODS, h5py"; fi
if want gmsh; then PYMODS="$PYMODS, gmsh, meshio"; fi
if want meshio || want exodus || want dagmc || want gmsh; then
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
    elif ! "$PY" -c "import $PYMODS" >>"$LOG" 2>&1; then
        # A venv from before the W3b pins: try once to complete it.
        "$VENV/bin/pip" install -q -r "$TOOLS/requirements.txt" >>"$LOG" 2>&1
        if ! "$PY" -c "import $PYMODS" >>"$LOG" 2>&1; then
            missing "python packages of tools/conformance/requirements.txt in $VENV"; SKIP_PY="venv incomplete"
        fi
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
LEGACY=0
if want openfoam || want meshio || want exodus; then LEGACY=1; fi
if [ $LEGACY = 1 ]; then
say "emitting the milestone fixtures w3a_hybrid and w3a_tet"
if ! (cd "$ROOT" && MILESTONE_W3A_EMIT="$WORK/fixtures" cargo test -q -p sm-cli --test milestone_w3a emit_conformance_scripts) >>"$LOG" 2>&1 \
    || [ ! -f "$WORK/fixtures/w3a_hybrid.smesh" ] || [ ! -f "$WORK/fixtures/w3a_tet.smesh" ]; then
    say "FAIL: could not emit w3a_hybrid / w3a_tet"; exit 1
fi
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
            if ! python3 "$TOOLS/check_openfoam.py" "$RESULTS/$label.openfoam.json" "$d/checkMesh.log" "$d/case" "$d/info.json" "$d/quality.json" "$label" $mode >>"$LOG" 2>&1; then
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
if [ $LEGACY = 1 ]; then
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
fi

# ---------------------------------------------------------------- W3b: DAGMC .h5m and Gmsh msh4
# Inputs: the sm-meshio `export_conformance` cases (analytic expectations; written twice for the
# byte-identity check) and tools/w3b_milestone.smesh (the lattice, plus the graveyard and binary
# variants through `smesh export --option`). Tier A is h5py / gmsh / meshio in the venv, Tier B
# is MOAB and DAGMC in the smesh-dagmc-conformance image. Our validator is not involved: these
# files are exporter output, judged by the external readers only.
w3b_check() { # w3b_check <label> <kind> <fixture> <command...>; the command writes $RESULTS/<label>.<kind>.json
    local label="$1" kind="$2" fx="$3"
    shift 3
    printf '{"fixture": "%s", "label": "%s", "mesh": ""}\n' "$fx" "$label" > "$RESULTS/$label.meta.json"
    if [ -n "$SKIP_PY" ]; then
        result "$RESULTS/$label.$kind.json" skipped "\"$SKIP_PY\""
        return
    fi
    "$@" >>"$LOG" 2>&1
    if [ ! -s "$RESULTS/$label.$kind.json" ]; then
        result "$RESULTS/$label.$kind.json" fail "\"check crashed before writing a result (see conformance.log)\""
    fi
}
if want dagmc || want gmsh; then
    IN="$WORK/export_in"
    IN2="$WORK/export_in2"
    say "writing the export_conformance cases (sm-meshio example), twice"
    if ! (cd "$ROOT" && cargo run -q --release -p sm-meshio --example export_conformance -- "$IN") >>"$LOG" 2>&1 \
        || ! (cd "$ROOT" && cargo run -q --release -p sm-meshio --example export_conformance -- "$IN2") >>"$LOG" 2>&1; then
        say "FAIL: cargo run -p sm-meshio --example export_conformance"; exit 1
    fi
    say "running tools/w3b_milestone.smesh"
    W3B="$WORK/w3b"
    mkdir -p "$W3B"
    proj="$WORK/proj_w3b"
    "$SMESH" new "$proj" >>"$LOG" 2>&1
    if ! "$SMESH" run "$ROOT/tools/w3b_milestone.smesh" -p "$proj" --json > "$WORK/run_w3b.json" 2>>"$LOG" \
        || ! "$SMESH" export "$W3B/milestone_gy.h5m" --format dagmc --option graveyard=true -p "$proj" >>"$LOG" 2>&1 \
        || ! "$SMESH" export "$W3B/milestone_bin.msh" --format gmsh --option binary=true -p "$proj" >>"$LOG" 2>&1; then
        say "  w3b_milestone: OUR BUG (the milestone script or its export failed: $(head -c 400 "$WORK/run_w3b.json"))"
        printf '{"fixture": "w3b_milestone", "label": "w3b_milestone", "mesh": ""}\n' > "$RESULTS/w3b_milestone.meta.json"
        result "$RESULTS/w3b_milestone.validate.json" fail "\"OUR BUG: the milestone script failed\""
    else
        cp "$proj/milestone.h5m" "$W3B/milestone.h5m"
        cp "$proj/milestone.msh" "$W3B/milestone.msh"
        REF="$TOOLS/fixtures/ref/dagmc_two_volume.h5m"
        if want dagmc; then
            say "dagmc Tier A (h5py)"
            w3b_check dagmc_two_volume h5m export_conformance "$PY" "$TOOLS/check_h5m.py" "$RESULTS/dagmc_two_volume.h5m.json" \
                "$IN/two_volume.h5m" "$REF" "$IN/two_volume.expect.json" --twin "$IN2/two_volume.h5m"
            w3b_check dagmc_hex_blocks h5m export_conformance "$PY" "$TOOLS/check_h5m.py" "$RESULTS/dagmc_hex_blocks.h5m.json" \
                "$IN/hex_blocks.h5m" "$REF" --twin "$IN2/hex_blocks.h5m"
            w3b_check w3b_milestone h5m w3b_milestone "$PY" "$TOOLS/check_h5m.py" "$RESULTS/w3b_milestone.h5m.json" "$W3B/milestone.h5m" "$REF"
            w3b_check w3b_milestone_graveyard h5m w3b_milestone "$PY" "$TOOLS/check_h5m.py" "$RESULTS/w3b_milestone_graveyard.h5m.json" "$W3B/milestone_gy.h5m" "$REF"
            say "dagmc Tier B (MOAB and DAGMC in $IMAGE_DAGMC)"
            printf '{"fixture": "export_conformance", "label": "dagmc_two_volume", "mesh": ""}\n' > "$RESULTS/dagmc_two_volume.meta.json"
            if [ -n "$SKIP_DAGMC_DOCKER" ]; then
                result "$RESULTS/dagmc_two_volume.dagmc.json" skipped "\"$SKIP_DAGMC_DOCKER\""
            else
                docker run --rm ${DOCKER_PLATFORM_DAGMC[@]+"${DOCKER_PLATFORM_DAGMC[@]}"} --user "$(id -u):$(id -g)" \
                    -v "$IN:/in:ro" -v "$RESULTS:/out" "$IMAGE_DAGMC" \
                    python /opt/check_dagmc.py /in/two_volume.h5m /in/two_volume.expect.json /out/dagmc_two_volume.dagmc.json >>"$LOG" 2>&1
                if [ ! -s "$RESULTS/dagmc_two_volume.dagmc.json" ]; then
                    result "$RESULTS/dagmc_two_volume.dagmc.json" fail "\"the DAGMC image wrote no result (see conformance.log)\""
                fi
            fi
        fi
        if want gmsh; then
            say "gmsh Tier A (gmsh, meshio)"
            for f in "$IN"/*.msh; do
                base="$(basename "$f" .msh)"
                exp="$IN/$base.expect.json"
                if [ -f "$exp" ]; then
                    w3b_check "gmsh_$base" msh4 export_conformance "$PY" "$TOOLS/check_msh4.py" "$RESULTS/gmsh_$base.msh4.json" "$f" "$exp"
                else
                    w3b_check "gmsh_$base" msh4 export_conformance "$PY" "$TOOLS/check_msh4.py" "$RESULTS/gmsh_$base.msh4.json" "$f"
                fi
            done
            w3b_check w3b_milestone_ascii msh4 w3b_milestone "$PY" "$TOOLS/check_msh4.py" "$RESULTS/w3b_milestone_ascii.msh4.json" "$W3B/milestone.msh"
            w3b_check w3b_milestone_binary msh4 w3b_milestone "$PY" "$TOOLS/check_msh4.py" "$RESULTS/w3b_milestone_binary.msh4.json" "$W3B/milestone_bin.msh"
        fi
    fi
fi

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
    parts = ", ".join(f"{k} {row[k]['status']}" for k in ("validate", "openfoam", "meshio", "exodus", "h5m", "dagmc", "msh4") if k in row)
    print(f"  {row['label']:26s} {parts}")
    for k in ("validate", "openfoam", "meshio", "exodus", "h5m", "dagmc", "msh4"):
        for p in row.get(k, {}).get("problems", []):
            print(f"      {k}: {p}")
    w = row.get("openfoam", {}).get("warnings")
    if w is not None:
        print(f"      openfoam warnings (INT-9, not gating): {w}")
    sp = row.get("openfoam", {}).get("facts", {}).get("int8_split")
    if sp:
        print(f"      openfoam INT-8 split: {sp['split_cells']} cells into {sp['sub_cells']} sub-cells, +{sp['added_internal_faces']} internal faces")
warnings = {r["label"]: r["openfoam"]["warnings"] for r in rows if "warnings" in r.get("openfoam", {})}
splits = {r["label"]: r["openfoam"]["facts"]["int8_split"]["split_cells"] for r in rows if "int8_split" in r.get("openfoam", {}).get("facts", {})}
report["openfoam_warnings"] = warnings
report["openfoam_int8_splits"] = splits
json.dump(report, open(out, "w"), indent=1)
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

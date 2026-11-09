#!/usr/bin/env python3
"""W3c round-pin Python driver (plan section 5.1): the pin milestone from the INSTALLED wheel.

Runs `tools/w3c_pin.smesh`, `w3c_pin_enclosed.smesh`, `w3c_pin_leftout.smesh` and
`w3c_warped.smesh` through `smartmesh.Session` in the given output directory, then exports through
the typed `Session.export` (no warning on the fully blocked pin, `ExportWarning` on the left-out
volume, OpenFOAM with and without `split_warped_faces`). Prints one `key value` line per fact; the
Rust harness (crates/sm-cli/tests/milestone_w3c_pin.rs) compares them with the CLI, MCP and
in-process front ends.

usage: w3c_driver.py <tools-dir> <out-dir>
"""

from __future__ import annotations

import hashlib
import os
import sys
import warnings

import smartmesh as sm

TIME = "2026-10-30T00:00:00Z"  # the Rust harness's fixed time


def fail(msg: str) -> None:
    print(f"FAIL {msg}")
    sys.exit(1)


def printed(output: str) -> dict[str, list[str]]:
    out: dict[str, list[str]] = {}
    for line in output.splitlines():
        key, _, rest = line.partition(" ")
        out.setdefault(key, []).append(rest.strip())
    return out


def file_sha(path: str) -> str:
    with open(path, "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def tree_sha(root: str) -> str:
    """sha256 over `relpath sha256(file)` lines of every file under `root`, sorted by path."""
    rows = []
    for d, _, names in os.walk(root):
        for n in names:
            p = os.path.join(d, n)
            rows.append(f"{os.path.relpath(p, root).replace(os.sep, '/')} {file_sha(p)}\n")
    return hashlib.sha256("".join(sorted(rows)).encode()).hexdigest()


def run(tools: str, name: str) -> tuple[sm.Session, dict[str, list[str]]]:
    s = sm.Session.memory(author="it", fixed_time=TIME)
    with open(os.path.join(tools, name), encoding="utf-8") as fh:
        r = s.run_script(fh.read())
    p = printed(r.output)
    if s.hash().state != p["h_mesh"][0] or r.state != p["h_mesh"][0]:
        fail(f"{name}: session hash differs from the script's h_mesh")
    return s, p


def silent_export(s: sm.Session, path: str, **kw: object) -> sm.ExportResult:
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        return s.export(path, **kw)  # type: ignore[arg-type]


def main() -> None:
    tools, out = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
    os.makedirs(out, exist_ok=True)
    os.chdir(out)
    print(f"smartmesh {sm.__version__} from {os.path.dirname(sm.__file__)}")

    # the through pin and its lattice
    s, p = run(tools, "w3c_pin.smesh")
    for key in ("h_model", "h_auto", "h_mesh", "cells", "shapes", "mesh_volume", "model_volume", "violations", "head"):
        print(f"{key} {p[key][0]}")
    print(f"state {s.hash().state}")
    print(f"file pin.smrt {file_sha('pin.smrt')}")
    print(f"tree pin_foam {tree_sha('pin_foam')}")
    r = silent_export(s, "py.smrt")
    if r.notes or r.warnings:
        fail(f"fully blocked .smrt export has notes {r.notes}")
    print(f"file py.smrt {file_sha('py.smrt')}")
    off = silent_export(s, "py_foam_off", format="openfoam")
    on = silent_export(s, "py_foam_on", format="openfoam", split_warped_faces=True)
    print(f"foam_notes off {sorted(n.code for n in off.notes)} on {sorted(n.code for n in on.notes)}")
    print(f"tree py_foam_off {tree_sha('py_foam_off')}")
    print(f"tree py_foam_on {tree_sha('py_foam_on')}")
    if s.hash().state != p["h_mesh"][0]:
        fail("exports moved the state hash")

    # the enclosed pin
    e, q = run(tools, "w3c_pin_enclosed.smesh")
    for key in ("h_model", "h_auto", "h_mesh", "cells", "shapes", "mesh_volume", "violations"):
        print(f"enclosed_{key} {q[key][0]}")
    print(f"file enclosed.smrt {file_sha('enclosed.smrt')}")
    silent_export(e, "py_enclosed.smrt")
    print(f"file py_enclosed.smrt {file_sha('py_enclosed.smrt')}")

    # a volume in no block: the typed result and a Python ExportWarning
    lo, lp = run(tools, "w3c_pin_leftout.smesh")
    print(f"leftout_h_mesh {lp['h_mesh'][0]}")
    with warnings.catch_warnings(record=True) as rec:
        warnings.simplefilter("always")
        res = lo.export("leftout.smrt")
    ew = [w for w in rec if issubclass(w.category, sm.ExportWarning)]
    if len(ew) != 1 or "smrt-region-names" not in str(ew[0].message):
        fail(f"expected one ExportWarning smrt-region-names, got {[str(w.message) for w in rec]}")
    if [n.code for n in res.warnings] != ["smrt-region-names"] or res.warnings[0].severity != "warning":
        fail(f"ExportResult.warnings: {res.warnings}")
    if "cells are in no block" not in res.warnings[0].detail:
        fail(f"warning detail: {res.warnings[0].detail}")
    print(f"leftout_warning {ew[0].message}")
    print(f"file leftout.smrt {file_sha('leftout.smrt')}")

    # OpenFOAM split_warped_faces on a warped mesh, and its refusal for another format
    w, wp = run(tools, "w3c_warped.smesh")
    print(f"warped_h_mesh {wp['h_mesh'][0]}")
    woff = silent_export(w, "w_off", format="openfoam")
    won = silent_export(w, "w_on", format="openfoam", split_warped_faces=True)
    print(f"warped_notes off {sorted(n.code for n in woff.notes)} on {sorted((n.code, n.count) for n in won.notes)}")
    print(f"tree w_off {tree_sha('w_off')}")
    print(f"tree w_on {tree_sha('w_on')}")
    try:
        w.export("x.msh", format="gmsh", split_warped_faces=True)
        fail("split_warped_faces on gmsh did not raise")
    except sm.SmartMeshError as x:
        if x.kind != "bad-request":
            fail(f"split_warped_faces on gmsh: kind {x.kind}")
        print(f"gmsh_refused {x.kind}")
    print("OK")


if __name__ == "__main__":
    main()

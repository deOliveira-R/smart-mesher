#!/usr/bin/env python3
"""W3b milestone Python driver (plan section 5.1): the milestone slice from the INSTALLED wheel.

Runs `tools/w3b_milestone.smesh` through `smartmesh.Session` in the given output directory (the
script writes its files relative to it), then replays the model and meshing part with typed ops
(`s.model.copy`, `s.mesh.auto`, ...) and requires the same state hashes. Prints one
`key value` line per fact; the Rust harness (crates/sm-cli/tests/milestone_w3b.rs) compares them
with the CLI, MCP and in-process front ends.

usage: w3b_driver.py <milestone.smesh> <out-dir>
"""

from __future__ import annotations

import hashlib
import os
import sys

import smartmesh as sm

FILES = ["milestone.step", "milestone.smsh", "milestone.msh", "milestone.h5m", "milestone.smrt"]


def fail(msg: str) -> None:
    print(f"FAIL {msg}")
    sys.exit(1)


def printed(output: str) -> dict[str, list[str]]:
    out: dict[str, list[str]] = {}
    for line in output.splitlines():
        key, _, rest = line.partition(" ")
        out.setdefault(key, []).append(rest.strip())
    return out


def typed_replay(s: sm.Session) -> tuple[str, str]:
    """The same ops as the script's main line, written as typed Python calls. Returns the state
    hashes after `mesh.size` (h_model) and after the volume run (h_mesh)."""

    def brick(**kw: object) -> sm.ApplyResult:
        return s.model.create_brick(**kw)

    pf = brick(x=0.8, y=0.8, z=1.0)
    wl = brick(x=0.23, y=1.26, z=1.0, center=[-0.515, 0.0, 0.0])
    wr = brick(x=0.23, y=1.26, z=1.0, center=[0.515, 0.0, 0.0])
    wb = brick(x=0.8, y=0.23, z=1.0, center=[0.0, -0.515, 0.0])
    wt = brick(x=0.8, y=0.23, z=1.0, center=[0.0, 0.515, 0.0])

    def all_bodies() -> list[sm.EntityId]:
        return [sm.EntityId.parse(b) for b in s.query("model.list", kind="TopoBody")]

    def all_volumes() -> list[sm.EntityId]:
        return [sm.EntityId.parse(v) for v in s.query("model.list", kind="TopoVolume")]

    s.model.imprint(bodies=all_bodies())
    s.model.merge()
    water = sorted(wl.outputs["volume"] + wr.outputs["volume"] + wb.outputs["volume"] + wt.outputs["volume"])
    move_x = {"kind": "move", "delta": [1.26, 0.0, 0.0]}
    move_y = {"kind": "move", "delta": [0.0, 1.26, 0.0]}
    fx = s.model.copy(entities=pf.outputs["volume"], count=2, xform=move_x)
    wx = s.model.copy(entities=water, count=2, xform=move_x)
    fy = s.model.copy(entities=sorted(pf.outputs["volume"] + fx.outputs["volumes"]), count=2, xform=move_y)
    wy = s.model.copy(entities=sorted(water + wx.outputs["volumes"]), count=2, xform=move_y)
    s.model.imprint(bodies=all_bodies())
    s.model.merge()
    if len(all_volumes()) != 45:
        fail(f"typed lattice has {len(all_volumes())} volumes")
    fuels = sorted(pf.outputs["volume"] + fx.outputs["volumes"] + fy.outputs["volumes"])
    waters = sorted(water + wx.outputs["volumes"] + wy.outputs["volumes"])
    s.fem.block_add(block=1, volumes=fuels, name="fuel")
    s.fem.block_add(block=2, volumes=waters, name="water")
    s.ops.mesh.size(targets=all_volumes(), size=0.3)
    h_model = s.hash().state
    s.ops.mesh.auto(strategy="default")
    m = s.ops.mesh.run(targets=all_volumes())
    mesh_id = m.outputs["mesh"][0]
    mesh = s.mesh(mesh_id)
    if mesh.n_cells != 675:
        fail(f"typed mesh has {mesh.n_cells} cells")
    return h_model, s.hash().state


def main() -> None:
    script, out = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
    text = open(script, encoding="utf-8").read()
    os.makedirs(out, exist_ok=True)
    os.chdir(out)

    print(f"smartmesh {sm.__version__} from {os.path.dirname(sm.__file__)}")
    s = sm.Session.memory(author="it", fixed_time="2026-11-05T00:00:00Z")
    r = s.run_script(text)
    p = printed(r.output)
    for key in ("h_model", "h_auto", "h_mesh", "cells", "model_volume", "step_volume", "face_cells", "step_bytes", "head"):
        print(f"{key} {p[key][0]}")
    for k in ("vol_a", "vol_b"):
        print(f"{k} {' '.join(p.get(k, []))}")
    if s.hash().state != p["h_mesh"][0] or r.state != p["h_mesh"][0]:
        fail("session hash differs from the script's h_mesh")
    print(f"state {s.hash().state}")
    st = s.status()
    print(f"branches {sorted(st.branches)} nodes {st.nodes}")
    for f in FILES:
        with open(f, "rb") as fh:
            print(f"file {f} {hashlib.sha256(fh.read()).hexdigest()}")

    # typed ops reproduce the script's main line
    typed = sm.Session.memory(author="it", fixed_time="2026-11-05T00:00:00Z")
    h_model, h_mesh = typed_replay(typed)
    if h_model != p["h_model"][0]:
        fail(f"typed h_model {h_model} != script {p['h_model'][0]}")
    if h_mesh != p["h_mesh"][0]:
        fail(f"typed h_mesh {h_mesh} != script {p['h_mesh'][0]}")
    print(f"typed {h_model} {h_mesh}")

    # the exported mesh read back through the Python .smsh reader
    back = sm.Mesh.from_smsh("milestone.smsh")
    if back.n_cells != 675:
        fail(f".smsh reader: {back.n_cells} cells")
    print(f"smsh_cells {back.n_cells}")
    print("OK")


if __name__ == "__main__":
    main()

"""Tier A check of a Gmsh MSH 4.1 file we wrote, with Gmsh itself and meshio.

usage: check_msh4.py OUT.json FILE.msh [EXPECT.json]

Gmsh (python API, pinned gmsh==4.15.2 in requirements.txt): `gmsh.open` accepts the file (ASCII
and binary), node count, element counts by type, the physical groups with their names, every
entity of a physical group exists, the Jacobian determinants of every volume element are
positive at a Gauss rule and their weighted sum is the analytic volume to 1e-10 (this fixes the
node order of tet, hex, wedge and pyramid), and a Gmsh re-save as msh4 round trips the counts.
meshio reads the same file and agrees on the cell blocks (hexahedron, tetra, wedge, pyramid).
EXPECT.json (from `cargo run -p sm-meshio --example export_conformance`): `nodes`, `volume`,
`cells` (meshio names), `physical_groups` as [dim, tag, name].
"""
import json
import os
import sys
import tempfile

import gmsh
import meshio
import numpy as np

# Gmsh element type -> meshio cell block name, for the volume types.
VOLUME = {4: "tetra", 5: "hexahedron", 6: "wedge", 7: "pyramid"}


def summary(path):
    gmsh.clear()
    gmsh.open(path)
    nodes = gmsh.model.mesh.getNodes()[0]
    types, tags, _ = gmsh.model.mesh.getElements()
    by_type = {int(t): int(len(x)) for t, x in zip(types, tags)}
    groups = []
    for dim, tag in gmsh.model.getPhysicalGroups():
        ents = sorted(int(e) for e in gmsh.model.getEntitiesForPhysicalGroup(dim, tag))
        groups.append([int(dim), int(tag), gmsh.model.getPhysicalName(dim, tag), ents])
    return {"nodes": int(len(nodes)), "by_type": by_type, "groups": groups}


def jacobians(problems, facts):
    total, worst = 0.0, np.inf
    for dim, tag in gmsh.model.getEntities(3):
        for ty in gmsh.model.mesh.getElementTypes(dim, tag):
            if ty not in VOLUME:
                continue
            local, weights = gmsh.model.mesh.getIntegrationPoints(ty, "Gauss4")
            _, dets, _ = gmsh.model.mesh.getJacobians(ty, local, tag)
            dets = np.array(dets).reshape(-1, len(weights))
            worst = min(worst, float(dets.min()))
            total += float((dets * np.array(weights)).sum())
    facts["min_jacobian_determinant"] = worst
    facts["jacobian_volume"] = total
    if not worst > 0:
        problems.append(f"non-positive Jacobian determinant {worst}")
    return total


def main(argv):
    out, path = argv[0], argv[1]
    expect = json.load(open(argv[2])) if len(argv) > 2 else {}
    problems, facts = [], {}
    physical_only = b""
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    try:
        s = summary(path)
        facts.update({"nodes": s["nodes"], "elements_by_gmsh_type": s["by_type"], "physical_groups": [g[:3] for g in s["groups"]]})
        if "nodes" in expect and expect["nodes"] != s["nodes"]:
            problems.append(f"{s['nodes']} nodes, expected {expect['nodes']}")
        for dim, tag, name, ents in s["groups"]:
            if not ents:
                problems.append(f"physical group {name!r} has no entity")
        if "physical_groups" in expect:
            want = sorted([d, t, n] for d, t, n in expect["physical_groups"])
            got = sorted(g[:3] for g in s["groups"])
            if got != want:
                problems.append(f"physical groups {got}, expected {want}")
        vol = jacobians(problems, facts)
        if "volume" in expect and abs(vol - expect["volume"]) > 1e-10 * expect["volume"]:
            problems.append(f"Jacobian volume {vol} vs analytic {expect['volume']}")
        with tempfile.TemporaryDirectory() as tmp:
            again = os.path.join(tmp, "again.msh")
            # Without SaveAll Gmsh writes only the elements of physical groups.
            gmsh.option.setNumber("Mesh.SaveAll", 1)
            gmsh.write(again)
            s2 = summary(again)
            gmsh.option.setNumber("Mesh.SaveAll", 0)
            gmsh.write(os.path.join(tmp, "physical_only.msh"))
            physical_only = open(os.path.join(tmp, "physical_only.msh"), "rb").read()
        if (s2["nodes"], s2["by_type"], [g[:3] for g in s2["groups"]]) != (s["nodes"], s["by_type"], [g[:3] for g in s["groups"]]):
            problems.append("a Gmsh re-save as msh4 changes the counts, types or physical groups")
        facts["resave_round_trips"] = (s2["nodes"], s2["by_type"]) == (s["nodes"], s["by_type"])
    except Exception as e:  # Gmsh raises on files it cannot read
        problems.append(f"gmsh could not read the file: {e}")
    finally:
        gmsh.finalize()
    try:
        try:
            m = meshio.read(path)
        except ValueError as e:
            if "gmsh:physical" not in str(e):
                raise
            # meshio 5.3.5 cannot read msh4 where only some entities carry a physical group
            # (Gmsh writes such files itself); read Gmsh's own physical-only re-save instead.
            facts["meshio_partial_physical_tags"] = "meshio rejects entities without a physical group; read Gmsh's physical-only re-save"
            with tempfile.TemporaryDirectory() as tmp:
                q = os.path.join(tmp, "physical_only.msh")
                open(q, "wb").write(physical_only)
                m = meshio.read(q)
        cells = {}
        for block in m.cells:
            if block.type in VOLUME.values():
                cells[block.type] = cells.get(block.type, 0) + len(block.data)
        facts["meshio_volume_cells"] = cells
        if "cells" in expect and cells != expect["cells"]:
            problems.append(f"meshio cell blocks {cells}, expected {expect['cells']}")
        if "nodes" in expect and len(m.points) != expect["nodes"] and "meshio_partial_physical_tags" not in facts:
            problems.append(f"meshio sees {len(m.points)} points, expected {expect['nodes']}")
    except Exception as e:
        problems.append(f"meshio could not read the file: {e}")
    json.dump({"status": "fail" if problems else "pass", "problems": problems, "facts": facts}, open(out, "w"), indent=1, default=str)
    if problems:
        print("\n".join(problems), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

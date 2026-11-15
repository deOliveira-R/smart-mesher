"""Reads an Abaqus .inp with meshio and prints per-type cell counts and volumes, set sizes and
point count as JSON. Used by tests/abaqus.rs (the oracle: meshio's Abaqus reader).

usage: python -I abaqus_meshio.py FILE.inp
Volumes: tets det/6; hexahedra as 6 tets around the 0-6 diagonal and wedges as 3 tets, both exact
for planar-faced cells (every fixture here), so they check the written node order independently
of the writer's own tables.
"""
import json
import sys

import meshio
import numpy as np


def tet(p, a, b, c, d):
    return float(np.linalg.det(np.stack([p[b] - p[a], p[c] - p[a], p[d] - p[a]]))) / 6.0


def volume(kind, p, c):
    if kind == "tetra":
        return tet(p, *c)
    if kind == "wedge":
        return tet(p, c[0], c[1], c[2], c[5]) + tet(p, c[0], c[1], c[5], c[4]) + tet(p, c[0], c[4], c[5], c[3])
    if kind == "hexahedron":
        return sum(tet(p, c[0], c[a], c[b], c[6]) for a, b in [(1, 2), (2, 3), (3, 7), (7, 4), (4, 5), (5, 1)])
    raise SystemExit(f"unexpected cell type {kind}")


m = meshio.read(sys.argv[1], file_format="abaqus")
out = {"points": len(m.points), "counts": {}, "volume": {}, "cell_sets": {}, "point_sets": {}}
for blk in m.cells:
    out["counts"][blk.type] = out["counts"].get(blk.type, 0) + len(blk.data)
    v = sum(volume(blk.type, m.points, c) for c in blk.data)
    out["volume"][blk.type] = out["volume"].get(blk.type, 0.0) + v
for k, v in m.cell_sets.items():
    out["cell_sets"][k] = int(sum(len(a) for a in v))
for k, v in m.point_sets.items():
    out["point_sets"][k] = int(len(v))
print(json.dumps(out))

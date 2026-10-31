"""netCDF4 structure check of an exported Exodus file against our `.smsh` export of the same mesh.

usage: check_exodus.py OUT.json FILE.exo FILE.smsh
Checks num_nodes, num_elem, num_el_blk, num_side_sets, connect<i> per block, coordinates, and
that every side set's (element, side) list reconstructs, under the Exodus II side numbering
(typed here from the manual), exactly the face node sets of our side set with the same id.
Polyhedral (NFACED) side sets number sides by face-list position; they are reconstructed from
facconn / fbconn.
"""
import json
import sys

import numpy as np
from netCDF4 import Dataset

from smsh import Smsh

SIDES = {
    "HEX8": [(1, 2, 6, 5), (2, 3, 7, 6), (3, 4, 8, 7), (1, 5, 8, 4), (1, 4, 3, 2), (5, 6, 7, 8)],
    "TET4": [(1, 2, 4), (2, 3, 4), (1, 4, 3), (1, 3, 2)],
    "WEDGE6": [(1, 2, 5, 4), (2, 3, 6, 5), (1, 4, 6, 3), (1, 3, 2), (4, 5, 6)],
    "PYRAMID5": [(1, 2, 5), (2, 3, 5), (3, 4, 5), (1, 5, 4), (1, 4, 3, 2)],
}
ALIASES = {"HEX": "HEX8", "HEXAHEDRON": "HEX8", "TETRA": "TET4", "TETRA4": "TET4", "TET": "TET4", "WEDGE": "WEDGE6", "PYRAMID": "PYRAMID5"}


def text(v):
    if isinstance(v, bytes):
        return v.decode()
    return str(v)


def main(out, exo, smsh_path):
    problems, facts = [], {}
    ref = Smsh(smsh_path)
    pts = ref.points()
    ds = Dataset(exo)
    dim = lambda n: len(ds.dimensions[n]) if n in ds.dimensions else 0
    facts.update({k: dim(k) for k in ("num_nodes", "num_elem", "num_el_blk", "num_side_sets")})
    if dim("num_nodes") != len(pts):
        problems.append(f"num_nodes {dim('num_nodes')} != {len(pts)} mesh vertices")
    ncell = len(ref.arrays["cell_shape"])
    if dim("num_elem") != ncell:
        problems.append(f"num_elem {dim('num_elem')} != {ncell} cells")
    want_ss = ref.side_sets()
    if dim("num_side_sets") != len(want_ss):
        problems.append(f"num_side_sets {dim('num_side_sets')} != {len(want_ss)} side sets")
    if dim("num_nodes") == len(pts):
        xyz = np.stack([ds.variables[c][:] for c in ("coordx", "coordy", "coordz")], axis=1)
        dev = float(np.max(np.abs(xyz - pts))) if len(pts) else 0.0
        facts["max_coordinate_deviation"] = dev
        if dev > 1e-12:
            problems.append(f"coordinates differ from the mesh by {dev}")
    # Elements in file order: (type, node tuple 1-based, faces for NFACED).
    elems = []
    fb = []
    for j in range(1, dim("num_fa_blk") + 1):
        conn = ds.variables[f"fbconn{j}"][:].tolist()
        cnt = ds.variables[f"fbepecnt{j}"][:].tolist()
        at = 0
        for c in cnt:
            fb.append(tuple(conn[at:at + c]))
            at += c
    blocks = []
    for i in range(1, dim("num_el_blk") + 1):
        # An NFACED block stores face ids in facconn<i> (file-global face numbering over the
        # face blocks in order) and has no connect<i>.
        name = f"facconn{i}" if f"facconn{i}" in ds.variables else f"connect{i}"
        if name not in ds.variables:
            problems.append(f"block {i}: no connect{i} or facconn{i}")
            continue
        var = ds.variables[name]
        ty = text(getattr(var, "elem_type", "")).upper()
        ty = ALIASES.get(ty, ty)
        n = dim(f"num_el_in_blk{i}")
        blocks.append((i, ty, n))
        if name.startswith("facconn") and ty != "NFACED":
            problems.append(f"block {i}: facconn{i} has elem_type {ty}, expected NFACED")
            continue
        if ty == "NFACED":
            fc = ds.variables[f"facconn{i}"][:].tolist()
            cnt = ds.variables[f"ebepecnt{i}"][:].tolist()
            at = 0
            for c in cnt:
                elems.append(("NFACED", None, [fb[f - 1] for f in fc[at:at + c]]))
                at += c
        else:
            conn = np.asarray(var[:]).reshape(n, -1)
            for row in conn:
                elems.append((ty, tuple(int(x) for x in row), None))
    facts["blocks"] = blocks
    if len(elems) != dim("num_elem"):
        problems.append(f"blocks hold {len(elems)} elements, num_elem {dim('num_elem')}")
    ids = ds.variables["ss_prop1"][:].tolist() if "ss_prop1" in ds.variables else []
    for k, sid in enumerate(ids, start=1):
        el = ds.variables[f"elem_ss{k}"][:].tolist()
        sd = ds.variables[f"side_ss{k}"][:].tolist()
        got = []
        for e, s in zip(el, sd):
            if not 1 <= e <= len(elems):
                problems.append(f"side set {sid}: element {e} out of range 1..{len(elems)}")
                break
            ty, nodes, faces = elems[e - 1]
            if ty == "NFACED" and not 1 <= s <= len(faces):
                problems.append(f"side set {sid}: element {e} side {s} out of range 1..{len(faces)}")
                break
            if ty == "NFACED":
                got.append(tuple(sorted(v - 1 for v in faces[s - 1])))
            elif ty in SIDES:
                got.append(tuple(sorted(nodes[i - 1] - 1 for i in SIDES[ty][s - 1])))
            else:
                problems.append(f"side set {sid}: element type {ty} has no side table")
                break
        got.sort()
        want = want_ss.get(int(sid))
        if want is None:
            problems.append(f"side set {sid} has no counterpart in the mesh")
        elif got != want:
            problems.append(f"side set {sid}: {len(got)} sides reconstruct a different face set than our {len(want)} sides")
    json.dump({"status": "fail" if problems else "pass", "problems": problems, "facts": facts}, open(out, "w"), indent=1)


if __name__ == "__main__":
    main(*sys.argv[1:4])

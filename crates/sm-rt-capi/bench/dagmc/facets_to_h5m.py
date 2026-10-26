#!/usr/bin/env python3
"""facets.bin ("smfacets v1", see bench/README.md) -> DAGMC h5m with pymoab.

Triangles with the same (forward, reverse) region pair that are connected through shared
vertices form one surface. Each region becomes a volume set; each surface set carries the
GEOM_SENSE_2 tag [forward volume, reverse volume] (0 = none, the implicit complement), where
"forward" is the volume the triangle normals point OUT of (the same rule as the smfacets files).
Then `check_watertight` (DAGMC's make_watertight tool chain) is run on the file and its verdict
is stored in the --report JSON; the script exits non-zero if the model is not watertight.
"""
import argparse
import json
import struct
import subprocess
import sys

import numpy as np
from pymoab import core, types


def read_facets(path):
    with open(path, "rb") as f:
        nv, nt = struct.unpack("<II", f.read(8))
        verts = np.frombuffer(f.read(nv * 24), dtype="<f8").reshape(nv, 3)
        tris = np.frombuffer(f.read(nt * 12), dtype="<u4").reshape(nt, 3)
        sides = np.frombuffer(f.read(nt * 8), dtype="<i4").reshape(nt, 2)
        assert f.read(1) == b"", "trailing bytes in facets.bin"
    return verts, tris, sides


def components(tris, keys):
    """Group triangles by (key, connected component over shared vertices)."""
    parent = {}

    def find(x):
        while parent.setdefault(x, x) != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for t in tris:
        a, b, c = (find(int(v)) for v in t)
        parent[b] = a
        parent[find(c)] = find(a)
    groups = {}
    for i, t in enumerate(tris):
        groups.setdefault((keys[i], find(int(t[0]))), []).append(i)
    return [(k[0], idx) for k, idx in sorted(groups.items(), key=lambda kv: kv[1][0])]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("facets")
    ap.add_argument("h5m")
    ap.add_argument("--report", default=None)
    ap.add_argument("--swap-sense", action="store_true", help="debug: swap forward/reverse volumes")
    a = ap.parse_args()

    verts, tris, sides = read_facets(a.facets)
    mb = core.Core()
    root = mb.get_root_set()

    def tag(name, size, kind, storage):
        return mb.tag_get_handle(name, size, kind, storage, create_if_missing=True)

    t_geom = tag("GEOM_DIMENSION", 1, types.MB_TYPE_INTEGER, types.MB_TAG_SPARSE)
    t_gid = tag("GLOBAL_ID", 1, types.MB_TYPE_INTEGER, types.MB_TAG_DENSE)
    t_cat = tag("CATEGORY", 32, types.MB_TYPE_OPAQUE, types.MB_TAG_SPARSE)
    t_name = tag("NAME", 32, types.MB_TYPE_OPAQUE, types.MB_TAG_SPARSE)
    t_sense = tag("GEOM_SENSE_2", 2, types.MB_TYPE_HANDLE, types.MB_TAG_SPARSE)
    t_tol = tag("FACETING_TOL", 1, types.MB_TYPE_DOUBLE, types.MB_TAG_SPARSE)
    mb.tag_set_data(t_tol, root, np.array([1e-4]))

    vh = mb.create_vertices(verts.astype("<f8").ravel())
    vh = list(vh)
    tri_h = [mb.create_element(types.MBTRI, [vh[int(i)] for i in t]) for t in tris]

    regions = sorted({int(r) for pair in sides for r in pair if r >= 0})
    vols = {}
    for r in regions:
        s = mb.create_meshset()
        mb.tag_set_data(t_geom, s, np.array([3], dtype="i4"))
        mb.tag_set_data(t_gid, s, np.array([r + 1], dtype="i4"))
        mb.tag_set_data(t_cat, s, np.array(["Volume"], dtype="S32"))
        mb.tag_set_data(t_name, s, np.array([f"region{r}"], dtype="S32"))
        vols[r] = s

    keys = [(int(f), int(b)) for f, b in sides]
    surfaces = components(tris, keys)
    for gid, ((fwd, rev), idx) in enumerate(surfaces, start=1):
        s = mb.create_meshset()
        mb.tag_set_data(t_geom, s, np.array([2], dtype="i4"))
        mb.tag_set_data(t_gid, s, np.array([gid], dtype="i4"))
        mb.tag_set_data(t_cat, s, np.array(["Surface"], dtype="S32"))
        mb.add_entities(s, [tri_h[i] for i in idx])
        f_h = vols[fwd] if fwd >= 0 else 0
        r_h = vols[rev] if rev >= 0 else 0
        if a.swap_sense:
            f_h, r_h = r_h, f_h
        mb.tag_set_data(t_sense, s, np.array([[f_h, r_h]], dtype="u8"))
        for v in {fwd, rev}:
            if v >= 0:
                mb.add_parent_child(vols[v], s)
    mb.write_file(a.h5m)

    # DAGMC's own watertightness check (the make_watertight tool chain).
    proc = subprocess.run(["check_watertight", a.h5m], capture_output=True, text=True)
    text = proc.stdout + proc.stderr
    report = {
        "facets": a.facets,
        "vertices": int(len(verts)),
        "triangles": int(len(tris)),
        "volumes": len(regions),
        "surfaces": len(surfaces),
        "check_watertight_exit": proc.returncode,
        "check_watertight_tail": text.strip().splitlines()[-6:],
    }
    report["watertight"] = proc.returncode == 0 and "not watertight" not in text.lower()
    if a.report:
        with open(a.report, "w") as f:
            json.dump(report, f, indent=2, sort_keys=True)
            f.write("\n")
    print(json.dumps(report, sort_keys=True))
    return 0 if report["watertight"] else 3


if __name__ == "__main__":
    sys.exit(main())

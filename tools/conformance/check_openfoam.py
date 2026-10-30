"""Compare an OpenFOAM `checkMesh -allGeometry -allTopology` log with our mesh.

usage: check_openfoam.py OUT.json CHECKMESH.log CASE_DIR INFO.json QUALITY.json [exact0]
- `Mesh OK.` must be printed;
- the cell census (hexahedra, prisms, pyramids, tetrahedra, polyhedra) equals ours exactly
  (checkMesh recognises a 2-tri + 3-quad prism from its faces: polyMesh stores no shapes);
- points, faces, cells equal our mesh, internal faces and patches equal what we wrote;
- Max non-orthogonality within 1e-3 degree and Max skewness within 1e-4 of our `mesh.quality`
  (openfoam profile); with `exact0` (hexbox) both must also be below 1e-6.
INFO.json is `mesh.info`, QUALITY.json is `mesh.quality` with profile openfoam.
"""
import json
import os
import re
import sys

NUM = r"([-+0-9.eE]+|nan|inf)"


def first(rx, text, cast=float):
    m = re.search(rx, text, re.MULTILINE)
    return cast(m.group(1)) if m else None


def foam_count(path):
    """The list length of an OpenFOAM ASCII file: the first bare integer line after the header."""
    body = open(path, encoding="latin-1").read()
    body = re.sub(r"/\*.*?\*/", "", body, flags=re.S)
    body = re.sub(r"FoamFile\s*\{.*?\}", "", body, flags=re.S)
    m = re.search(r"^\s*(\d+)\s*$", body, re.MULTILINE)
    return int(m.group(1)) if m else None


def main(out, log, case, info_path, quality_path, mode=""):
    problems, facts = [], {}
    text = open(log, encoding="latin-1").read()
    info = json.load(open(info_path))
    quality = json.load(open(quality_path))
    if "Mesh OK." not in text:
        failed = [l.strip() for l in text.splitlines() if l.strip().startswith("***")]
        problems.append(f"checkMesh did not print 'Mesh OK.' ({failed[:8]})")
    shapes = info.get("cell_shapes", {})
    ours = {
        "hexahedra": shapes.get("Hex", 0),
        "prisms": shapes.get("Wedge", 0),
        "pyramids": shapes.get("Pyramid", 0),
        "tetrahedra": shapes.get("Tet", 0),
        "polyhedra": shapes.get("Polyhedron", 0),
    }
    theirs = {k: first(rf"^\s*{k}:\s+(\d+)", text, int) for k in ours}
    facts["census_checkmesh"] = theirs
    facts["census_ours"] = ours
    if theirs != ours:
        problems.append(f"cell census {theirs} != ours {ours}")
    poly = os.path.join(case, "constant", "polyMesh")
    counts = {
        "points": (first(r"^\s*points:\s+(\d+)", text, int), info.get("vertices")),
        "faces": (first(r"^\s*faces:\s+(\d+)", text, int), info.get("faces")),
        "internal faces": (first(r"^\s*internal faces:\s+(\d+)", text, int), foam_count(os.path.join(poly, "neighbour"))),
        "cells": (first(r"^\s*cells:\s+(\d+)", text, int), info.get("cells")),
        "boundary patches": (first(r"^\s*boundary patches:\s+(\d+)", text, int), foam_count(os.path.join(poly, "boundary"))),
    }
    facts["counts"] = counts
    for k, (a, b) in counts.items():
        if a != b:
            problems.append(f"{k}: checkMesh {a}, ours {b}")
    metric = {m["metric"]: m["summary"] for m in quality.get("metrics", [])}
    nonortho = first(rf"Mesh non-orthogonality Max:\s*{NUM}", text)
    skew = first(rf"Max skewness\s*=\s*{NUM}", text)
    our_no = metric.get("non_orthogonality", {}).get("max")
    our_sk = metric.get("skewness", {}).get("max")
    facts["non_orthogonality"] = {"checkmesh": nonortho, "ours": our_no}
    facts["skewness"] = {"checkmesh": skew, "ours": our_sk}
    if nonortho is None or our_no is None or abs(nonortho - our_no) > 1e-3:
        problems.append(f"max non-orthogonality: checkMesh {nonortho}, ours {our_no} (tol 1e-3 degree)")
    if skew is None or our_sk is None or abs(skew - our_sk) > 1e-4:
        problems.append(f"max skewness: checkMesh {skew}, ours {our_sk} (tol 1e-4)")
    if mode == "exact0":
        for k, v in (("non-orthogonality", nonortho), ("skewness", skew)):
            if v is None or abs(v) >= 1e-6:
                problems.append(f"hexbox {k} {v} is not below 1e-6")
    json.dump({"status": "fail" if problems else "pass", "problems": problems, "facts": facts}, open(out, "w"), indent=1)


if __name__ == "__main__":
    main(*sys.argv[1:7])

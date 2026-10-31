"""Compare an OpenFOAM `checkMesh -allGeometry -allTopology` log with our mesh.

usage: check_openfoam.py OUT.json CHECKMESH.log CASE_DIR INFO.json QUALITY.json LABEL [MODE]
- `Mesh OK.` must be printed, except for the polyhedral duals (LABEL polydual, w3a_poly; ruling
  INT-9): there 'concave cells (using face planes)', 'small interpolation weight' and 'small
  volume ratio' are recorded under `warnings` with their counts and do not gate; any other
  `***` finding (determinant included) still fails;
- the cell census (hexahedra, prisms, pyramids, tetrahedra, polyhedra) equals ours exactly
  (checkMesh recognises a 2-tri + 3-quad prism from its faces: polyMesh stores no shapes);
- points, faces, cells equal our mesh, internal faces and patches equal what we wrote;
- Max non-orthogonality within 1e-3 degree and Max skewness within 1e-4 of our `mesh.quality`
  (openfoam profile); with MODE `exact0` (hexbox) both must also be below 1e-6.
Ruling INT-8: when the exporter split small-determinant cells (`constant/polyMesh/smeshCellMap`,
exported cell -> internal cell), the expected exported mesh is ours plus the split: each split
parent becomes one face pyramid per face (a tet: 4 tets), cells = ours + extra sub-cells,
points = ours + one centroid per parent, faces = ours + one new internal face per parent edge.
The census is ours with each parent replaced by its sub-cells (shapes from the exported files:
triangle base -> tet, quad base -> pyramid). The split changes non-orthogonality and skewness, so
for a split case checkMesh's maxima are compared with the same OpenFOAM formulas evaluated here
on the exported polyMesh (validated against `mesh.quality` on every unsplit case, recorded as
`recomputed`); ours are recorded for reference.
When the export notes are available (`export_foam.json` next to CASE_DIR) they must report the
same split count; `smesh export --json` does not carry them yet (recorded in `int8_split.note`).
INFO.json is `mesh.info`, QUALITY.json is `mesh.quality` with profile openfoam.
"""
import json
import math
import os
import re
import sys

NUM = r"([-+0-9.eE]+|nan|inf)"
# INT-9: checkMesh findings intrinsic to the exact median dual (warnings for these labels only).
POLY_LABELS = {"polydual", "w3a_poly"}
POLY_WARNINGS = {
    "concave_cells": re.compile(r"concave cells \(using face planes\)", re.I),
    "small_interpolation_weight": re.compile(r"small interpolation weight", re.I),
    "small_volume_ratio": re.compile(r"small volume ratio", re.I),
}
VSMALL = 1e-300
ROOTVSMALL = 1e-150


def first(rx, text, cast=float):
    m = re.search(rx, text, re.MULTILINE)
    return cast(m.group(1)) if m else None


def body_of(path):
    body = open(path, encoding="latin-1").read()
    body = re.sub(r"/\*.*?\*/", "", body, flags=re.S)
    body = re.sub(r"//[^\n]*", "", body)
    return re.sub(r"FoamFile\s*\{.*?\}", "", body, flags=re.S)


def foam_count(path):
    """The list length of an OpenFOAM ASCII file: the first bare integer line after the header."""
    m = re.search(r"^\s*(\d+)\s*$", body_of(path), re.MULTILINE)
    return int(m.group(1)) if m else None


def label_list(path):
    body = body_of(path)
    m = re.search(r"^\s*(\d+)\s*$", body, re.MULTILINE)
    if not m:
        return None
    n = int(m.group(1))
    rest = body[m.end():]
    open_ = rest.index("(")
    vals = [int(x) for x in rest[open_ + 1:rest.index(")", open_)].split()]
    if len(vals) != n:
        raise ValueError(f"{path}: {len(vals)} labels for size {n}")
    return vals


def face_list(path):
    body = body_of(path)
    m = re.search(r"^\s*(\d+)\s*$", body, re.MULTILINE)
    n = int(m.group(1))
    faces = [[int(x) for x in g.split()] for g in re.findall(r"\d+\(([^()]*)\)", body[m.end():])]
    if len(faces) != n:
        raise ValueError(f"{path}: {len(faces)} faces for size {n}")
    return faces


def point_list(path):
    body = body_of(path)
    m = re.search(r"^\s*(\d+)\s*$", body, re.MULTILINE)
    n = int(m.group(1))
    rest = body[m.end():]
    pts = [tuple(float(x) for x in g.split()) for g in re.findall(r"\(([^()]+)\)", rest[rest.index("(") + 1:])]
    if len(pts) != n:
        raise ValueError(f"{path}: {len(pts)} points for size {n}")
    return pts


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def mag(a):
    return math.sqrt(dot(a, a))


def read_poly(poly):
    pts = point_list(os.path.join(poly, "points"))
    faces = face_list(os.path.join(poly, "faces"))
    own = label_list(os.path.join(poly, "owner"))
    nei = label_list(os.path.join(poly, "neighbour"))
    return pts, faces, own, nei


def foam_quality(pts, faces, own, nei):
    """Max non-orthogonality (degrees, internal faces) and max skewness (all faces), by OpenFOAM's
    primitiveMeshTools formulas (face centres/areas by triangle fan, cell centres by pyramids)."""
    fc, fa = [], []
    for f in faces:
        p = [pts[i] for i in f]
        if len(p) == 3:
            fc.append(tuple((p[0][k] + p[1][k] + p[2][k]) / 3.0 for k in range(3)))
            fa.append(tuple(0.5 * x for x in cross(sub(p[1], p[0]), sub(p[2], p[0]))))
            continue
        # OpenFOAM primitiveMeshFaceCentresAndAreas: triangle fan about the point average,
        # centre weighted by the triangle area projected on the face normal.
        c0 = tuple(sum(q[k] for q in p) / len(p) for k in range(3))
        tris = [cross(sub(p[(i + 1) % len(p)], p[i]), sub(c0, p[i])) for i in range(len(p))]
        sn = tuple(sum(t[k] for t in tris) for k in range(3))
        ms = mag(sn)
        hat = tuple(x / ms for x in sn) if ms > VSMALL else (0.0, 0.0, 0.0)
        san, sanc = 0.0, [0.0] * 3
        for i, t in enumerate(tris):
            an = dot(t, hat)
            san += an
            for k in range(3):
                sanc[k] += an * (p[i][k] + p[(i + 1) % len(p)][k] + c0[k])
        fc.append(tuple(sanc[k] / (3.0 * san) for k in range(3)) if san > ROOTVSMALL else c0)
        fa.append(tuple(0.5 * x for x in sn))
    nc = max(max(own), max(nei, default=-1)) + 1
    cest, cnt = [[0.0] * 3 for _ in range(nc)], [0] * nc
    for f, o in enumerate(own):
        cells = (o, nei[f]) if f < len(nei) else (o,)
        for c in cells:
            for k in range(3):
                cest[c][k] += fc[f][k]
            cnt[c] += 1
    cest = [tuple(x / max(n, 1) for x in c) for c, n in zip(cest, cnt)]
    cc, cv = [[0.0] * 3 for _ in range(nc)], [0.0] * nc
    for f, o in enumerate(own):
        cells = ((o, 1.0), (nei[f], -1.0)) if f < len(nei) else ((o, 1.0),)
        for c, s in cells:
            v = s * dot(fa[f], sub(fc[f], cest[c]))
            for k in range(3):
                cc[c][k] += v * (0.75 * fc[f][k] + 0.25 * cest[c][k])
            cv[c] += v
    cc = [tuple(x / cv[c] for x in cc[c]) if abs(cv[c]) > VSMALL else cest[c] for c in range(nc)]

    def skew(f, d, cpf):
        sv_ = dot(fa[f], d)
        sv = tuple(cpf[k] - (dot(fa[f], cpf) / (sv_ + ROOTVSMALL)) * d[k] for k in range(3))
        msv = mag(sv)
        hat = tuple(x / (msv + ROOTVSMALL) for x in sv)
        fd = 0.2 * mag(d) + ROOTVSMALL
        for i in faces[f]:
            fd = max(fd, abs(dot(hat, sub(pts[i], fc[f]))))
        return msv / fd

    max_no, max_sk = 0.0, 0.0
    for f, o in enumerate(own):
        cpf = sub(fc[f], cc[o])
        if f < len(nei):
            d = sub(cc[nei[f]], cc[o])
            c = dot(d, fa[f]) / (mag(d) * mag(fa[f]) + VSMALL)
            max_no = max(max_no, math.degrees(math.acos(max(-1.0, min(1.0, c)))))
        else:
            nrm = mag(fa[f])
            n = tuple(x / nrm for x in fa[f]) if nrm > VSMALL else (0.0, 0.0, 0.0)
            d = tuple(n[k] * dot(n, cpf) for k in range(3))
        max_sk = max(max_sk, skew(f, d, cpf))
    return max_no, max_sk


SHAPE_OF_SIZES = {
    (3, 3, 3, 3): "tetrahedra",
    (3, 3, 3, 3, 4): "pyramids",
    (3, 3, 4, 4, 4): "prisms",
    (4, 4, 4, 4, 4, 4): "hexahedra",
}


def split_census(cell_map, faces, own, nei):
    """Parents split by the exporter and the census delta: (parents, sub-cells, added internal
    faces, {shape: delta}, problems)."""
    subs = {}
    for e, c in enumerate(cell_map):
        subs.setdefault(c, []).append(e)
    parents = {c: s for c, s in subs.items() if len(s) > 1}
    of = {}
    for c, s in parents.items():
        for e in s:
            of[e] = c
    cell_faces = {}
    added = 0
    for f, o in enumerate(own):
        n = nei[f] if f < len(nei) else None
        if n is not None and o in of and of.get(n) == of[o]:
            added += 1
            continue
        for e in (o, n):
            if e is not None and e in of:
                cell_faces.setdefault(e, []).append(len(faces[f]))
    delta, problems = {}, []
    for c, s in sorted(parents.items()):
        bases = []
        for e in s:
            b = cell_faces.get(e, [])
            if len(b) != 1:
                problems.append(f"split parent {c}: sub-cell {e} has {len(b)} base faces (1 expected)")
                continue
            bases.append(b[0])
            kind = "tetrahedra" if b[0] == 3 else "pyramids" if b[0] == 4 else "polyhedra"
            delta[kind] = delta.get(kind, 0) + 1
        parent = SHAPE_OF_SIZES.get(tuple(sorted(bases)), "polyhedra")
        delta[parent] = delta.get(parent, 0) - 1
    want_added = 0
    for c, s in parents.items():
        want_added += sum(cell_faces.get(e, [0])[0] for e in s) // 2
    if added != want_added:
        problems.append(f"split adds {added} internal faces, parent edges give {want_added}")
    return len(parents), sum(len(s) for s in parents.values()), added, delta, problems


def export_notes(case):
    path = os.path.join(os.path.dirname(case.rstrip("/")), "export_foam.json")
    try:
        v = json.load(open(path))
    except (OSError, ValueError):
        return None
    for k in ("notes",):
        if isinstance(v.get(k), list):
            return v[k]
        if isinstance(v.get("result"), dict) and isinstance(v["result"].get(k), list):
            return v["result"][k]
    return None


def main(out, log, case, info_path, quality_path, label="", mode=""):
    problems, facts, warnings = [], {}, {}
    text = open(log, encoding="latin-1").read()
    info = json.load(open(info_path))
    quality = json.load(open(quality_path))
    poly_label = label in POLY_LABELS
    starred = [l.strip() for l in text.splitlines() if l.strip().startswith("***")]
    if poly_label:
        # INT-9: three findings are warnings with counts; any other '***' line gates.
        other = []
        for l in starred:
            key = next((k for k, rx in POLY_WARNINGS.items() if rx.search(l)), None)
            if key is None:
                other.append(l)
                continue
            n = re.findall(r"(\d+)\s*$", l)
            warnings[key] = warnings.get(key, 0) + (int(n[0]) if n else 1)
        if other:
            problems.append(f"checkMesh findings other than the INT-9 warnings: {other[:8]}")
        facts["mesh_ok"] = "Mesh OK." in text
        if not facts["mesh_ok"] and not other and not warnings:
            problems.append("checkMesh printed neither 'Mesh OK.' nor a finding")
    elif "Mesh OK." not in text:
        problems.append(f"checkMesh did not print 'Mesh OK.' ({starred[:8]})")
    poly = os.path.join(case, "constant", "polyMesh")
    pts, faces, own, nei = read_poly(poly)
    map_path = os.path.join(poly, "smeshCellMap")
    cell_map = label_list(map_path) if os.path.exists(map_path) else []
    split = None
    n_cells = max(max(own), max(nei, default=-1)) + 1
    if cell_map:
        if len(cell_map) != n_cells:
            problems.append(f"smeshCellMap has {len(cell_map)} entries for {n_cells} cells")
        if sorted(set(cell_map)) != list(range(info.get("cells", -1))):
            problems.append("smeshCellMap is not onto our cells 0..cells-1")
        parents, sub_cells, added, delta, sp = split_census(cell_map, faces, own, nei)
        problems += sp
        split = {"split_cells": parents, "sub_cells": sub_cells, "added_internal_faces": added, "census_delta": delta}
        notes = export_notes(case)
        line = next((n for n in notes or [] if n.startswith("foam.small_det_split")), None)
        split["note"] = line
        m = re.search(r"into (\d+) sub-cells.* x(\d+)$", line or "")
        if notes is None:
            # `smesh export --json` prints sm-protocol's ExportResult, which has no notes field.
            split["note"] = "export notes not available from `smesh export --json` (ExportResult has no notes)"
        elif not m:
            problems.append(f"export notes do not report the INT-8 split: {notes}")
        elif (int(m.group(2)), int(m.group(1))) != (parents, sub_cells):
            problems.append(f"export note reports {m.group(2)} split cells / {m.group(1)} sub-cells, smeshCellMap gives {parents} / {sub_cells}")
        facts["int8_split"] = split
    shapes = info.get("cell_shapes", {})
    ours = {
        "hexahedra": shapes.get("Hex", 0),
        "prisms": shapes.get("Wedge", 0),
        "pyramids": shapes.get("Pyramid", 0),
        "tetrahedra": shapes.get("Tet", 0),
        "polyhedra": shapes.get("Polyhedron", 0),
    }
    want = dict(ours)
    if split:
        for k, d in split["census_delta"].items():
            want[k] += d
        facts["census_exported"] = want
    theirs = {k: first(rf"^\s*{k}:\s+(\d+)", text, int) for k in ours}
    facts["census_checkmesh"] = theirs
    facts["census_ours"] = ours
    if theirs != want:
        problems.append(f"cell census {theirs} != {'exported (ours + INT-8 split)' if split else 'ours'} {want}")
    extra_cells = split["sub_cells"] - split["split_cells"] if split else 0
    extra_points = split["split_cells"] if split else 0
    extra_faces = split["added_internal_faces"] if split else 0
    counts = {
        "points": (first(r"^\s*points:\s+(\d+)", text, int), info.get("vertices", 0) + extra_points),
        "faces": (first(r"^\s*faces:\s+(\d+)", text, int), info.get("faces", 0) + extra_faces),
        "internal faces": (first(r"^\s*internal faces:\s+(\d+)", text, int), foam_count(os.path.join(poly, "neighbour"))),
        "cells": (first(r"^\s*cells:\s+(\d+)", text, int), info.get("cells", 0) + extra_cells),
        "boundary patches": (first(r"^\s*boundary patches:\s+(\d+)", text, int), foam_count(os.path.join(poly, "boundary"))),
    }
    facts["counts"] = counts
    for k, (a, b) in counts.items():
        if a != b:
            problems.append(f"{k}: checkMesh {a}, expected {b}")
    metric = {m["metric"]: m["summary"] for m in quality.get("metrics", [])}
    nonortho = first(rf"Mesh non-orthogonality Max:\s*{NUM}", text)
    skew = first(rf"Max skewness\s*=\s*{NUM}", text)
    our_no = metric.get("non_orthogonality", {}).get("max")
    our_sk = metric.get("skewness", {}).get("max")
    re_no, re_sk = foam_quality(pts, faces, own, nei)
    facts["non_orthogonality"] = {"checkmesh": nonortho, "ours": our_no, "recomputed": re_no}
    facts["skewness"] = {"checkmesh": skew, "ours": our_sk, "recomputed": re_sk}
    if split:
        # The split changes both maxima: compare with the exported mesh.
        ref_no, ref_sk, what = re_no, re_sk, "exported mesh (recomputed)"
    else:
        ref_no, ref_sk, what = our_no, our_sk, "ours"
        # The recomputation must agree with mesh.quality wherever nothing was split.
        if our_no is None or abs(re_no - our_no) > 1e-3 or our_sk is None or abs(re_sk - our_sk) > 1e-4:
            problems.append(f"recomputed non-orthogonality / skewness {re_no} / {re_sk} disagree with mesh.quality {our_no} / {our_sk}")
    facts["quality_compared_to"] = what
    if nonortho is None or ref_no is None or abs(nonortho - ref_no) > 1e-3:
        problems.append(f"max non-orthogonality: checkMesh {nonortho}, {what} {ref_no} (tol 1e-3 degree)")
    if skew is None or ref_sk is None or abs(skew - ref_sk) > 1e-4:
        problems.append(f"max skewness: checkMesh {skew}, {what} {ref_sk} (tol 1e-4)")
    if mode == "exact0":
        for k, v in (("non-orthogonality", nonortho), ("skewness", skew)):
            if v is None or abs(v) >= 1e-6:
                problems.append(f"hexbox {k} {v} is not below 1e-6")
    res = {"status": "fail" if problems else "pass", "problems": problems, "facts": facts}
    if poly_label:
        res["warnings"] = warnings
    json.dump(res, open(out, "w"), indent=1)


if __name__ == "__main__":
    main(*sys.argv[1:8])

"""meshio and VTK read the exported `.exo` and `.vtu` files; compared with our `.smsh` export.

usage: check_meshio.py OUT.json FILE.exo FILE.vtu FILE.smsh
- meshio: cell counts per type equal our census, points within 1e-12 of our coordinates.
  meshio has no NFACED reader (it raises or returns no cells); for a polyhedral Exodus file that
  is recorded as a tool limit (VTK below is the judge there), not as a failure. meshio 5.3.5 also
  cannot read a polyhedral .vtu with cell data: it groups the cells by node count in first-seen
  order but the cell data by connectivity size in sorted order, and raises "Incompatible cell
  data"; that is a recorded tool limit too, and VTK checks the .vtu points instead.
- VTK: vtkExodusIIReader and vtkXMLUnstructuredGridReader load the files; cell counts per type
  equal our census (VTK_POLYHEDRON count = our polyhedra). vtkExodusIIReader runs in a child
  process, so a reader crash is reported as a finding instead of killing this check.
"""
import json
import subprocess
import sys

import meshio
import numpy as np
import vtk

from smsh import Smsh

MESHIO = {"tetra": "tet", "hexahedron": "hex", "wedge": "wedge", "pyramid": "pyramid"}
VTK = {10: "tet", 12: "hex", 13: "wedge", 14: "pyramid", 42: "polyhedron"}


def meshio_census(m):
    out = {}
    for b in m.cells:
        k = MESHIO.get(b.type, "polyhedron" if b.type.startswith("polyhedron") else b.type)
        out[k] = out.get(k, 0) + len(b.data)
    return out


def vtk_census(grid):
    out = {}
    for i in range(grid.GetNumberOfCells()):
        k = VTK.get(grid.GetCellType(i), str(grid.GetCellType(i)))
        out[k] = out.get(k, 0) + 1
    return out


EXO_CHILD = """
import json, sys, vtk
VTK = {10: "tet", 12: "hex", 13: "wedge", 14: "pyramid", 42: "polyhedron"}
e = vtk.vtkExodusIIReader()
e.SetFileName(sys.argv[1])
e.UpdateInformation()
e.SetAllArrayStatus(vtk.vtkExodusIIReader.ELEM_BLOCK, 1)
e.Update()
it = e.GetOutput().NewIterator()
it.InitTraversal()
got = {}
while not it.IsDoneWithTraversal():
    ds = it.GetCurrentDataObject()
    for i in range(ds.GetNumberOfCells()):
        k = VTK.get(ds.GetCellType(i), str(ds.GetCellType(i)))
        got[k] = got.get(k, 0) + 1
    it.GoToNextItem()
print(json.dumps(got))
"""


def vtk_face_block_hazard(exo):
    """Face blocks (1-based, after the first) whose first face an NFACED element references."""
    from netCDF4 import Dataset

    ds = Dataset(exo)
    if "num_fa_blk" not in ds.dimensions:
        return {}
    firsts, at = {}, 0
    for j in range(1, len(ds.dimensions["num_fa_blk"]) + 1):
        if j > 1:
            firsts[at + 1] = j
        at += len(ds.dimensions[f"num_fa_in_blk{j}"])
    hits = {}
    for i in range(1, len(ds.dimensions["num_el_blk"]) + 1):
        if f"facconn{i}" not in ds.variables:
            continue
        ids = set(int(x) for x in ds.variables[f"facconn{i}"][:].tolist())
        for f, j in firsts.items():
            if f in ids:
                hits[j] = hits.get(j, 0) + 1
    return hits


def main(out, exo, vtu, smsh_path):
    problems, notes, facts = [], [], {}
    ref = Smsh(smsh_path)
    census = ref.census()
    pts = ref.points()
    facts["census"] = census
    poly = census.get("polyhedron", 0) > 0
    for label, path in (("exo", exo), ("vtu", vtu)):
        try:
            m = meshio.read(path)
        except Exception as e:  # noqa: BLE001 - any reader failure is a finding
            if label == "exo" and poly:
                notes.append(f"meshio cannot read the NFACED Exodus file ({type(e).__name__}: {e}); VTK judges it")
                continue
            if label == "vtu" and poly and "Incompatible cell data" in str(e):
                notes.append(f"meshio 5.3.5 cannot read a polyhedral .vtu with cell data ({e}); VTK judges it")
                continue
            problems.append(f"meshio.read({label}): {type(e).__name__}: {e}")
            continue
        got = meshio_census(m)
        facts[f"meshio_{label}"] = got
        if label == "exo" and poly and not got:
            notes.append("meshio read no cells from the NFACED Exodus file (no NFACED reader); VTK judges it")
            continue
        if got != census:
            problems.append(f"meshio {label} census {got} != ours {census}")
        if m.points.shape != pts.shape:
            problems.append(f"meshio {label}: {m.points.shape[0]} points, ours {len(pts)}")
        else:
            dev = float(np.max(np.abs(m.points - pts))) if len(pts) else 0.0
            facts[f"meshio_{label}_max_point_dev"] = dev
            if dev > 1e-12:
                problems.append(f"meshio {label}: points differ by {dev}")
    # VTK readers.
    r = vtk.vtkXMLUnstructuredGridReader()
    r.SetFileName(vtu)
    r.Update()
    g = r.GetOutput()
    got = vtk_census(g) if g is not None else {}
    facts["vtk_vtu"] = got
    if got != census:
        problems.append(f"vtkXMLUnstructuredGridReader census {got} != ours {census}")
    if g is not None and g.GetNumberOfPoints() == len(pts):
        vp = np.array([g.GetPoint(i) for i in range(g.GetNumberOfPoints())])
        dev = float(np.max(np.abs(vp - pts))) if len(pts) else 0.0
        facts["vtk_vtu_max_point_dev"] = dev
        if dev > 1e-12:
            problems.append(f"vtk vtu: points differ by {dev}")
    elif g is not None:
        problems.append(f"vtk vtu: {g.GetNumberOfPoints()} points, ours {len(pts)}")
    # vtkExodusIIReader (source of 9.4.0 and master, 2026-10; crash seen with the pinned 9.7.1) looks a file-global face id up with
    # `blockLocalFaceId > Size` instead of `>=`, so the first face of every face block after the
    # first is read one past the end of the previous block (undefined behaviour: it aborts with
    # bad_alloc / length_error on some runs and not others). Report the hazard deterministically.
    hazard = vtk_face_block_hazard(exo)
    facts["vtk_exo_first_face_refs"] = hazard
    if hazard:
        problems.append(f"vtkExodusIIReader reads out of bounds on this file: NFACED elements reference the first face of face block(s) {sorted(hazard)} after the first (VTK off-by-one in GetPolyhedronFaceConnectivity; one face block for all NFACED blocks avoids it)")
    # vtkExodusIIReader in a child process (a C++ abort would otherwise end this check).
    child = subprocess.run([sys.executable, "-I", "-c", EXO_CHILD, exo], capture_output=True, text=True)
    if child.returncode != 0:
        tail = (child.stderr.strip().splitlines() or ["no output"])[-1]
        problems.append(f"vtkExodusIIReader crashed (exit {child.returncode}): {tail}")
    else:
        got = json.loads(child.stdout.strip().splitlines()[-1])
        facts["vtk_exo"] = got
        if got != census:
            problems.append(f"vtkExodusIIReader census {got} != ours {census}")
    json.dump({"status": "fail" if problems else "pass", "problems": problems, "notes": notes, "facts": facts}, open(out, "w"), indent=1)


if __name__ == "__main__":
    main(*sys.argv[1:5])

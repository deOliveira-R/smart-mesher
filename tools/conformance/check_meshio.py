"""meshio and VTK read the exported `.exo` and `.vtu` files; compared with our `.smsh` export.

usage: check_meshio.py OUT.json FILE.exo FILE.vtu FILE.smsh
- meshio: cell counts per type equal our census, points within 1e-12 of our coordinates.
  meshio has no NFACED reader; for a polyhedral Exodus file that is recorded as a tool limit
  (VTK below is the judge there), not as a failure.
- VTK: vtkExodusIIReader and vtkXMLUnstructuredGridReader load the files; cell counts per type
  equal our census (VTK_POLYHEDRON count = our polyhedra).
"""
import json
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
            problems.append(f"meshio.read({label}): {type(e).__name__}: {e}")
            continue
        got = meshio_census(m)
        facts[f"meshio_{label}"] = got
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
    e = vtk.vtkExodusIIReader()
    e.SetFileName(exo)
    e.UpdateInformation()
    e.SetAllArrayStatus(vtk.vtkExodusIIReader.ELEM_BLOCK, 1)
    e.Update()
    mb = e.GetOutput()
    it = mb.NewIterator()
    it.InitTraversal()
    got = {}
    while not it.IsDoneWithTraversal():
        ds = it.GetCurrentDataObject()
        for k, v in vtk_census(ds).items():
            got[k] = got.get(k, 0) + v
        it.GoToNextItem()
    facts["vtk_exo"] = got
    if got != census:
        problems.append(f"vtkExodusIIReader census {got} != ours {census}")
    json.dump({"status": "fail" if problems else "pass", "problems": problems, "notes": notes, "facts": facts}, open(out, "w"), indent=1)


if __name__ == "__main__":
    main(*sys.argv[1:5])

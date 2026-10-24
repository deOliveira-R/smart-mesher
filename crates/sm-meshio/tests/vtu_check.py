"""Independent decoder for VTU files written by sm-meshio; prints a JSON summary.

usage: python3 -I vtu_check.py FILE
Decodes the appended data (raw or base64) with the standard library only. If the `vtk`
python modules are importable, also loads the file with VTK's XML reader and reports
facts VTK derives (cell types, per-cell face counts, outward-face check, volumes).
"""
import base64, json, re, struct, sys
import xml.etree.ElementTree as ET

CODE = {"Float64": ("d", 8), "Int32": ("i", 4), "Int64": ("q", 8), "UInt8": ("B", 1)}


def decode(path):
    data = open(path, "rb").read()
    k = data.index(b"<AppendedData")
    head = data[:k] + b"</VTKFile>"
    enc = re.search(rb'<AppendedData encoding="(\w+)"', data[k:]).group(1).decode()
    body_start = data.index(b"_", k) + 1
    body_end = data.rindex(b"\n  </AppendedData>")
    body = data[body_start:body_end]
    root = ET.fromstring(head)
    piece = root.find("UnstructuredGrid/Piece")
    out = {"np": int(piece.get("NumberOfPoints")), "nc": int(piece.get("NumberOfCells")), "encoding": enc, "sections": {}}
    for sec in piece:
        arrays = {}
        for da in sec.findall("DataArray"):
            fmt, size = CODE[da.get("type")]
            off = int(da.get("offset"))
            if enc == "raw":
                (n,) = struct.unpack_from("<Q", body, off)
                raw = body[off + 8:off + 8 + n]
            else:
                (n,) = struct.unpack("<Q", base64.b64decode(body[off:off + 12]))
                raw = base64.b64decode(body[off + 12:off + 12 + (n + 2) // 3 * 4])
            assert len(raw) == n, (da.get("Name"), len(raw), n)
            arrays[da.get("Name")] = {"type": da.get("type"), "ncomp": int(da.get("NumberOfComponents", "1")),
                                     "values": list(struct.unpack("<%d%s" % (n // size, fmt), raw))}
        out["sections"][sec.tag] = arrays
    return out


def vtk_facts(path):
    try:
        from vtkmodules.vtkIOXML import vtkXMLUnstructuredGridReader
        from vtkmodules.vtkFiltersVerdict import vtkCellSizeFilter
        from vtkmodules.vtkCommonDataModel import vtkPolygon
        from vtkmodules.util.numpy_support import vtk_to_numpy
        import numpy as np
    except Exception:
        return None
    r = vtkXMLUnstructuredGridReader()
    r.SetFileName(path)
    r.Update()
    g = r.GetOutput()
    s = vtkCellSizeFilter()
    s.SetInputData(g)
    s.ComputeVolumeOn(); s.ComputeAreaOn(); s.ComputeLengthOff(); s.ComputeVertexCountOff()
    s.Update()
    o = s.GetOutput()
    vol = vtk_to_numpy(o.GetCellData().GetArray("Volume")).tolist()
    area = vtk_to_numpy(o.GetCellData().GetArray("Area")).tolist()
    pts = vtk_to_numpy(g.GetPoints().GetData())
    types, nfaces, outward, nverts = [], [], [], []
    for c in range(g.GetNumberOfCells()):
        cell = g.GetCell(c)
        types.append(g.GetCellType(c))
        nverts.append(cell.GetNumberOfPoints())
        ids = [cell.GetPointId(i) for i in range(cell.GetNumberOfPoints())]
        ctr = pts[ids].mean(axis=0)
        nf = cell.GetNumberOfFaces()
        nfaces.append(nf)
        ok = True
        for i in range(nf):
            f = cell.GetFace(i)
            fp = np.array([f.GetPoints().GetPoint(j) for j in range(f.GetNumberOfPoints())])
            n = np.zeros(3)
            for j in range(len(fp)):  # Newell normal
                a, b = fp[j], fp[(j + 1) % len(fp)]
                n += np.cross(a, b)
            if np.dot(n, fp.mean(axis=0) - ctr) <= 0:
                ok = False
        outward.append(ok if nf else None)
    return {"n_cells": g.GetNumberOfCells(), "n_points": g.GetNumberOfPoints(), "types": types, "nverts": nverts,
            "nfaces": nfaces, "outward": outward, "volume": vol, "area": area,
            "cell_arrays": [g.GetCellData().GetArrayName(i) for i in range(g.GetCellData().GetNumberOfArrays())],
            "point_arrays": [g.GetPointData().GetArrayName(i) for i in range(g.GetPointData().GetNumberOfArrays())]}


if __name__ == "__main__":
    d = decode(sys.argv[1])
    d["vtk"] = vtk_facts(sys.argv[1])
    print(json.dumps(d))

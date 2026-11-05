"""The ARCHITECTURE section 8 slice in its three spellings, and table helpers."""

from __future__ import annotations

import hashlib

import numpy as np
import numpy.typing as npt

import smartmesh as sm

#: One statement per line, ids written out (each `cmd` is its own script scope).
SLICE_LINES = [
    "model.create_brick(x: 10.0, y: 10.0, z: 10.0)",
    "mesh.size(targets: TopoVolume#1, size: 1.0)",
    'mesh.scheme(targets: TopoVolume#1, scheme: "map")',
    "mesh.run(targets: TopoVolume#1)",
    "fem.block_add(block: 1, volumes: TopoVolume#1)",
    'fem.block_element_type(block: 1, element_type: "HEX8")',
]

#: The same slice as one script with variables (also the file given to `smesh run`).
SLICE_SCRIPT = """\
let b = model.create_brick(x: 10.0, y: 10.0, z: 10.0)
mesh.size(targets: b.volume, size: 1.0)
mesh.scheme(targets: b.volume, scheme: "map")
let m = mesh.run(targets: b.volume)
fem.block_add(block: 1, volumes: b.volume)
fem.block_element_type(block: 1, element_type: "HEX8")
"""


def slice_typed(s: sm.Session) -> str:
    """Run the slice as typed ops; returns the state hash."""
    b = s.model.create_brick(x=10.0, y=10.0, z=10.0)
    vol = b.outputs["volume"]
    s.ops.mesh.size(targets=vol, size=1.0)
    s.ops.mesh.scheme(targets=vol, scheme="map")
    s.ops.mesh.run(targets=vol)
    s.fem.block_add(block=1, volumes=vol)
    last = s.fem.block_element_type(block=1, element_type="HEX8")
    return last.state


def tables_digest(m: sm.Mesh) -> str:
    """sha256 over every always-present table (name, dtype, bytes)."""
    h = hashlib.sha256()
    for name, a in m.tables().items():
        h.update(name.encode())
        h.update(str(a.dtype).encode())
        h.update(np.ascontiguousarray(a).tobytes())
    return h.hexdigest()


def edge_count(m: sm.Mesh) -> int:
    """Number of distinct edges, from the face loops."""
    off = m.face_offsets.astype(np.int64)
    v = m.face_vertices.astype(np.int64)
    counts = np.diff(off)
    face_of = np.repeat(np.arange(counts.size), counts)
    pos = np.arange(v.size)
    nxt = np.where(pos + 1 == off[face_of + 1], off[face_of], pos + 1)
    a, b = v, v[nxt]
    lo, hi = np.minimum(a, b), np.maximum(a, b)
    return int(np.unique(lo * (int(v.max()) + 1) + hi).size)


def cell_volumes(m: sm.Mesh) -> npt.NDArray[np.float64]:
    """Volume of every cell by the divergence formula `V = 1/3 sum_f s_f (p_f . A_f)`."""
    pts = m.points
    off = m.face_offsets.astype(np.int64)
    v = m.face_vertices.astype(np.int64)
    counts = np.diff(off)
    face_of = np.repeat(np.arange(counts.size), counts)
    pos = np.arange(v.size)
    nxt = np.where(pos + 1 == off[face_of + 1], off[face_of], pos + 1)
    cross = np.cross(pts[v], pts[v[nxt]])
    area = 0.5 * np.add.reduceat(cross, off[:-1], axis=0)
    anchor = pts[v[off[:-1]]]
    face_dot = (anchor * area).sum(axis=1)
    fidx, sign = m.cell_faces_split()
    cell_of = np.repeat(np.arange(m.n_cells), np.diff(m.cell_face_offsets.astype(np.int64)))
    result: npt.NDArray[np.float64] = np.bincount(cell_of, weights=sign * face_dot[fidx] / 3.0, minlength=m.n_cells)
    return result

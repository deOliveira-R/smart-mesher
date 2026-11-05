"""`Mesh`: the polyhedral CSR tables of a mesh as read-only numpy arrays, named as in `.smsh`."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Literal, Mapping

import numpy as np
import numpy.typing as npt

from .errors import invalid
from .ids import EntityId

if TYPE_CHECKING:
    pass

#: `cell_shape` codes of `SMSH_FORMAT.md` 3.1.
SHAPE_NAMES: dict[int, str] = {0: "tet", 1: "hex", 2: "wedge", 3: "pyramid", 4: "polyhedron", 5: "tri", 6: "quad", 7: "polygon"}
#: Corner count of the template shapes (polyhedra and polygons have none fixed).
SHAPE_CORNERS: dict[str, int] = {"tet": 4, "hex": 8, "wedge": 6, "pyramid": 5, "tri": 3, "quad": 4}
#: The always-present tables, in `.smsh` order.
TABLE_NAMES: tuple[str, ...] = (
    "x", "y", "z", "vertex_ids", "face_ids", "cell_ids", "face_offsets", "face_vertices", "face_left", "face_right",
    "face_kind", "cell_face_offsets", "cell_faces", "cell_shape", "cell_corner_offsets", "cell_corners", "cell_parent",
    "cell_active", "class_entities", "vertex_class", "face_class", "cell_class",
)  # fmt: skip
#: `face_right` value of a face with one cell.
NO_CELL = 0xFFFFFFFF

_U8 = npt.NDArray[np.uint8]
_U32 = npt.NDArray[np.uint32]
_U64 = npt.NDArray[np.uint64]
_F64 = npt.NDArray[np.float64]


class Mesh:
    """Read-only mesh tables. A session mesh is a snapshot copy; a file mesh maps the `.smsh`.

    Both kinds have the same names, dtypes and meanings (`SMSH_FORMAT.md` 3.1), so they are
    interchangeable.
    """

    x: _F64
    y: _F64
    z: _F64
    vertex_ids: _U64
    face_ids: _U64
    cell_ids: _U64
    face_offsets: _U64
    face_vertices: _U32
    face_left: _U32
    face_right: _U32
    face_kind: _U8
    cell_face_offsets: _U64
    cell_faces: _U32
    cell_shape: _U8
    cell_corner_offsets: _U64
    cell_corners: _U32
    cell_parent: _U32
    cell_active: _U8
    class_entities: _U64
    vertex_class: _U32
    face_class: _U32
    cell_class: _U32

    def __init__(self, tables: Mapping[str, npt.NDArray[Any]], *, mesh_id: EntityId | None = None, source: str = "session") -> None:
        missing = [n for n in TABLE_NAMES if n not in tables]
        if missing:
            raise invalid(f"mesh tables missing: {missing}")
        for name in TABLE_NAMES:
            a = tables[name]
            a.flags.writeable = False
            object.__setattr__(self, name, a)
        self.mesh_id = mesh_id
        self.source = source

    def __setattr__(self, name: str, value: Any) -> None:
        if name in TABLE_NAMES:
            raise AttributeError(f"{name} is read-only")
        object.__setattr__(self, name, value)

    @classmethod
    def from_smsh(cls, path: str) -> Mesh:
        """Map a `.smsh` file (zero copy) and wrap its tables."""
        from . import smsh

        return smsh.open(path).mesh()

    def tables(self) -> dict[str, npt.NDArray[Any]]:
        """The always-present tables by name."""
        return {n: getattr(self, n) for n in TABLE_NAMES}

    @property
    def n_vertices(self) -> int:
        """Number of vertices."""
        return int(self.vertex_ids.shape[0])

    @property
    def n_faces(self) -> int:
        """Number of faces."""
        return int(self.face_ids.shape[0])

    @property
    def n_cells(self) -> int:
        """Number of cells."""
        return int(self.cell_ids.shape[0])

    @property
    def points(self) -> _F64:
        """Vertex coordinates, shape `(N, 3)` (high-order extra nodes follow the vertices)."""
        return np.stack([self.x, self.y, self.z], axis=1)

    def cells(self) -> dict[str, tuple[npt.NDArray[np.int64], _U32]]:
        """Cells by shape name as `(cell_indices, corners[n, k])` (meshio-style blocks).

        Polyhedra and polygons carry no fixed corner count: their corners array is `(n, 0)`.
        """
        out: dict[str, tuple[npt.NDArray[np.int64], _U32]] = {}
        off = self.cell_corner_offsets.astype(np.int64)
        for code, name in SHAPE_NAMES.items():
            idx = np.flatnonzero(self.cell_shape == code).astype(np.int64)
            if idx.size == 0:
                continue
            k = SHAPE_CORNERS.get(name)
            if k is None:
                out[name] = (idx, np.empty((idx.size, 0), dtype=np.uint32))
            else:
                cols = off[idx][:, None] + np.arange(k, dtype=np.int64)[None, :]
                out[name] = (idx, self.cell_corners[cols])
        return out

    def cell_faces_split(self) -> tuple[_U32, npt.NDArray[np.int8]]:
        """`cell_faces` as `(face_index, sign)`: sign +1 = the stored loop is outward for the cell."""
        w = self.cell_faces
        return (w >> np.uint32(1)).astype(np.uint32), np.where(w & np.uint32(1), -1, 1).astype(np.int8)

    def class_entity_ids(self, kind: Literal["vertex", "face", "cell"]) -> list[EntityId]:
        """The model entity each vertex, face or cell is classified on, one id per element."""
        if kind not in ("vertex", "face", "cell"):
            raise invalid(f"kind must be vertex, face or cell, not {kind!r}")
        col: npt.NDArray[np.uint32] = getattr(self, f"{kind}_class")
        raw = self.class_entities[col]
        cache: dict[int, EntityId] = {}
        out: list[EntityId] = []
        for r in raw.tolist():
            e = cache.get(r)
            if e is None:
                e = cache[r] = EntityId.from_raw(r)
            out.append(e)
        return out

    def __repr__(self) -> str:
        return f"Mesh({self.mesh_id}, vertices={self.n_vertices}, faces={self.n_faces}, cells={self.n_cells}, source={self.source!r})"

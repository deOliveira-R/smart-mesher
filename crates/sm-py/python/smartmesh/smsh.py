"""Independent numpy reader for `.smsh` 1.x: `mmap`, header checks, read-only zero-copy views.

Second reader next to `crates/sm-meshio/readers/smsh.py` (stdlib only); it also checks the writer.
"""

from __future__ import annotations

import builtins
import json
import math
import mmap
import struct
import zlib
from typing import Any

import numpy as np
import numpy.typing as npt

from .errors import SmartMeshError
from .mesh import Mesh

MAGIC = b"SMSH\r\n\x1a\n"
_DTYPES: dict[str, np.dtype[Any]] = {"u8": np.dtype("u1"), "u32": np.dtype("<u4"), "u64": np.dtype("<u8"), "i64": np.dtype("<i8"), "f64": np.dtype("<f8")}


class SmshError(SmartMeshError, ValueError):
    """The file is not a valid `.smsh` container."""

    default_kind = "smsh"


class SmshFile:
    """An opened `.smsh`: `.header` (JSON) and `.arrays` (name to read-only ndarray over the map)."""

    def __init__(self, path: str, *, verify: bool = False) -> None:
        with builtins.open(path, "rb") as f:
            self._map = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        size = len(self._map)
        if size < 16 or self._map[:8] != MAGIC:
            raise SmshError("not a .smsh file (bad magic)")
        (hl,) = struct.unpack_from("<Q", self._map, 8)
        if 16 + hl > size or (16 + hl) % 64 != 0:
            raise SmshError("bad header length")
        try:
            self.header: dict[str, Any] = json.loads(bytes(self._map[16 : 16 + hl]))
        except ValueError as e:
            raise SmshError(f"header is not JSON: {e}") from e
        if self.header.get("format") != "smsh":
            raise SmshError("format is not smsh")
        if (self.header.get("version") or {}).get("major") != 1:
            raise SmshError("unsupported .smsh major version")
        start = 16 + hl
        self.arrays: dict[str, npt.NDArray[Any]] = {}
        for a in self.header["arrays"]:
            name, dtype = a["name"], _DTYPES.get(a["dtype"])
            if dtype is None:
                raise SmshError(f"array {name}: unknown dtype {a['dtype']}")
            if name in self.arrays:
                raise SmshError(f"duplicate array {name}")
            shape = tuple(int(n) for n in a["shape"])
            n = math.prod(shape)
            if a["nbytes"] != n * dtype.itemsize:
                raise SmshError(f"array {name}: nbytes != prod(shape) * sizeof(dtype)")
            if a["offset"] % 64 != 0 or a["offset"] + a["nbytes"] > size - start:
                raise SmshError(f"array {name}: bad offset or length")
            if verify and zlib.crc32(self._map[start + a["offset"] : start + a["offset"] + a["nbytes"]]) != a["crc32"]:
                raise SmshError(f"array {name}: checksum mismatch")
            if n == 0:
                arr: npt.NDArray[Any] = np.empty(shape, dtype=dtype)
            else:
                arr = np.frombuffer(self._map, dtype=dtype, count=n, offset=start + a["offset"]).reshape(shape)
            arr.flags.writeable = False
            self.arrays[name] = arr

    def mesh(self) -> Mesh:
        """The tables as a `Mesh` (views stay mapped to the file)."""
        mid = (self.header.get("mesh") or {}).get("id")
        from .ids import EntityId

        return Mesh(self.arrays, mesh_id=EntityId.parse(mid) if mid else None, source="smsh")

    def close(self) -> None:
        """Unmap the file if no view is alive (otherwise the map lives until the views die)."""
        try:
            self._map.close()
        except BufferError:
            pass


def open(path: str, *, verify: bool = False) -> SmshFile:  # noqa: A001
    """Open a `.smsh` file; `verify=True` checks every array's CRC-32."""
    return SmshFile(path, verify=verify)

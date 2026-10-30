"""Minimal reader of our native `.smsh` container (the reference the external checks compare to).

Only what the conformance checks need: coordinates, face loops, cells' shapes and the named sets.
Layout: 8-byte magic, u64 LE header length, JSON header, then the arrays described by
`header["arrays"]` (offsets relative to the payload start; every array is CRC-32 checked).
"""
import json
import struct
import zlib

import numpy as np

MAGIC = b"SMSH\r\n\x1a\n"
DTYPES = {"f64": "<f8", "u64": "<u8", "i64": "<i8", "u32": "<u4", "i32": "<i4", "u8": "u1", "u16": "<u2"}
# sm_mesh_api::CellShape order (the `.smsh` cell_shape codes).
SHAPES = ["tet", "hex", "wedge", "pyramid", "polyhedron", "tri", "quad", "polygon"]


class Smsh:
    def __init__(self, path):
        b = open(path, "rb").read()
        if b[:8] != MAGIC:
            raise ValueError(f"{path}: not a .smsh file")
        n = struct.unpack("<Q", b[8:16])[0]
        self.header = json.loads(b[16:16 + n])
        arrays = self.header["arrays"]
        base = None
        for cand in (16 + n, (16 + n + 7) // 8 * 8, (16 + n + 63) // 64 * 64):
            a = arrays[0]
            if zlib.crc32(b[cand + a["offset"]:cand + a["offset"] + a["nbytes"]]) == a["crc32"]:
                base = cand
                break
        if base is None:
            raise ValueError(f"{path}: cannot locate the payload")
        self.arrays = {}
        for a in arrays:
            raw = b[base + a["offset"]:base + a["offset"] + a["nbytes"]]
            if zlib.crc32(raw) != a["crc32"]:
                raise ValueError(f"{path}: array {a['name']} fails its CRC")
            self.arrays[a["name"]] = np.frombuffer(raw, dtype=DTYPES[a["dtype"]])

    def points(self):
        return np.stack([self.arrays["x"], self.arrays["y"], self.arrays["z"]], axis=1)

    def face(self, f):
        o = self.arrays["face_offsets"]
        return self.arrays["face_vertices"][int(o[f]):int(o[f + 1])]

    def census(self):
        """Cell count per shape name."""
        out = {}
        for c in self.arrays["cell_shape"]:
            k = SHAPES[int(c)]
            out[k] = out.get(k, 0) + 1
        return out

    def side_sets(self):
        """label -> sorted list of sorted face vertex tuples (one per side)."""
        out = {}
        for s in self.header.get("sets", []):
            if s.get("role") != "side_set":
                continue
            words = self.arrays[s["array"]]
            out[int(s["label"])] = sorted(tuple(sorted(int(v) for v in self.face(int(w) >> 1))) for w in words)
        return out

"""Reference reader for the smart-mesh `.smsh` container (format v1.0 and v1.1, see SMSH_FORMAT.md).

Standard library only. Arrays are zero-copy memoryviews over an mmap (little-endian host).
Usage as a script: `python3 -I smsh.py FILE` verifies checksums and prints a JSON summary.
"""
import json, mmap, struct, sys, zlib

MAGIC = b"SMSH\r\n\x1a\n"
_FMT = {"u8": "B", "u32": "I", "u64": "Q", "i64": "q", "f64": "d"}


class Smsh:
    def __init__(self, path):
        with open(path, "rb") as f:
            self._map = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        if self._map[:8] != MAGIC:
            raise ValueError("not a .smsh file")
        (hl,) = struct.unpack_from("<Q", self._map, 8)
        self.header = json.loads(self._map[16:16 + hl])
        if self.header["version"]["major"] != 1:
            raise ValueError("unsupported .smsh major version")
        self._start = 16 + hl  # start of the data region, a multiple of 64
        self._arrays = {a["name"]: a for a in self.header["arrays"]}

    def has(self, name):
        return name in self._arrays

    def _bytes(self, a):
        s = self._start + a["offset"]
        return memoryview(self._map)[s:s + a["nbytes"]]

    def array(self, name):
        """Zero-copy 1-D memoryview of the array (shape is in header['arrays'])."""
        a = self._arrays[name]
        return self._bytes(a).cast(_FMT[a["dtype"]])

    def verify(self):
        """Raise ValueError if any array fails its CRC-32."""
        for a in self.header["arrays"]:
            if zlib.crc32(self._bytes(a)) != a["crc32"]:
                raise ValueError("checksum mismatch in " + a["name"])

    def face_loop(self, f):
        off, v = self.array("face_offsets"), self.array("face_vertices")
        return list(v[off[f]:off[f + 1]])

    def cell_faces(self, c):
        """[(face, sign)] with sign +1 = the stored loop is outward for this cell."""
        off, cf = self.array("cell_face_offsets"), self.array("cell_faces")
        return [(w >> 1, -1 if w & 1 else 1) for w in cf[off[c]:off[c + 1]]]

    def cell_vertices(self, c):
        """Sorted distinct vertices of cell c (works for polyhedra and template cells)."""
        return sorted({v for f, _ in self.cell_faces(c) for v in self.face_loop(f)})

    def cell_class(self, c):
        """Model entity id (raw u64: kind << 56 | serial) the cell is classified on."""
        return self.array("class_entities")[self.array("cell_class")[c]]

    def sets(self):
        return {s["name"]: (s["label"], s["role"], list(self.array(s["array"]))) for s in self.header["sets"]}

    def element_types(self):
        """[(set name, element type)] declared by the file (v1.1; [] for v1.0 files)."""
        return [(e["set"], e["element_type"]) for e in self.header.get("element_types", [])]

    def stats(self):
        """The v1.1 `mesh.stats` dict, or None for a v1.0 file."""
        return self.header["mesh"].get("stats")


def main(path):
    m = Smsh(path)
    m.verify()
    mi = m.header["mesh"]
    n = mi["n_cells"]
    print(json.dumps({
        "mesh": mi,
        "cell_vertex_counts": [len(m.cell_vertices(c)) for c in range(n)],
        "cell_ids": list(m.array("cell_ids")),
        "cell_class": [m.cell_class(c) for c in range(n)],
        "x_last": m.array("x")[mi["n_points"] - 1] if mi["n_points"] else None,
        "sets": {k: [v[0], v[1], len(v[2])] for k, v in m.sets().items()},
        "arrays": [a["name"] for a in m.header["arrays"]],
        "version": m.header["version"],
        "element_types": m.element_types(),
        "stats": m.stats(),
    }))


if __name__ == "__main__":
    main(sys.argv[1])

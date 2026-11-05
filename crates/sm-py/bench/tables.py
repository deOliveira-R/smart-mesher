"""Record (not gate) the cost of the table snapshot on a large hex mesh.

    python bench/tables.py [N]      # N cells per axis, default 100 (10^6 hexes)

Reports the meshing time, the snapshot time (`Session.mesh()`: one copy per table), the bytes
copied (sum of `nbytes`), and the `.smsh` export and `smsh.open` map times. Use a release build
(`maturin develop --release`) for numbers worth recording; they are machine-dependent.
"""

from __future__ import annotations

import sys
import tempfile
import time
from pathlib import Path

import smartmesh as sm


def main(n: int) -> None:
    s = sm.Session.memory()
    t = time.perf_counter()
    vol = s.model.create_brick(x=float(n), y=float(n), z=float(n)).outputs["volume"]
    s.ops.mesh.size(targets=vol, size=1.0)
    s.ops.mesh.scheme(targets=vol, scheme="map")
    s.ops.mesh.run(targets=vol)
    t_mesh = time.perf_counter() - t

    t = time.perf_counter()
    m = s.mesh()
    t_snap = time.perf_counter() - t
    nbytes = sum(a.nbytes for a in m.tables().values())

    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "m.smsh"
        t = time.perf_counter()
        s.export(p, format="smsh")
        t_export = time.perf_counter() - t
        t = time.perf_counter()
        f = sm.smsh.open(str(p))
        t_open = time.perf_counter() - t
        assert f.header["mesh"]["n_cells"] == m.n_cells
        f.close()

    print(f"cells={m.n_cells} vertices={m.n_vertices} faces={m.n_faces}")
    print(f"mesh_s={t_mesh:.3f} snapshot_s={t_snap:.4f} bytes_copied={nbytes} smsh_export_s={t_export:.3f} smsh_open_s={t_open:.5f}")


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 100)

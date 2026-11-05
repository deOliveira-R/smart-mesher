"""Run from a fresh venv holding only numpy and the installed wheel (never the source tree)."""

import os
import sys

import numpy as np

import smartmesh as sm

assert "site-packages" in sm.__file__, f"imported from the source tree: {sm.__file__}"
assert sm.__version__ == os.environ["EXPECT_VERSION"], (sm.__version__, os.environ["EXPECT_VERSION"])
assert "pytest" not in sys.modules and "mypy" not in sys.modules
assert not hasattr(sm._native, "_panic"), "the release wheel must not carry test hooks"  # type: ignore[attr-defined]

s = sm.Session.memory()
b = s.model.create_brick(x=10.0, y=10.0, z=10.0)
vol = b.outputs["volume"]
s.ops.mesh.size(targets=vol, size=1.0)
s.ops.mesh.scheme(targets=vol, scheme="map")
s.ops.mesh.run(targets=vol)
s.fem.block_add(block=1, volumes=vol)
s.fem.block_element_type(block=1, element_type="HEX8")
assert s.hash().state == os.environ["EXPECT_HASH"], "slice hash differs from `smesh run`"
m = s.mesh()
assert (m.n_vertices, m.n_faces, m.n_cells) == (1331, 3300, 1000)
print("wheel ok", sm.__version__, s.hash().state[:12], "numpy", np.__version__)

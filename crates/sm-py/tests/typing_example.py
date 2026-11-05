"""Checked with `mypy --strict`: typed ops come from the committed `ops.pyi`."""

from __future__ import annotations

import numpy as np
import numpy.typing as npt

import smartmesh as sm


def build(s: sm.Session) -> sm.Mesh:
    brick: sm.ApplyResult = s.model.create_brick(x=10.0, y=10.0, z=10.0)
    vol: list[sm.EntityId] = brick.outputs["volume"]
    s.ops.mesh.size(targets=vol, size=1.0)
    s.ops.mesh.scheme(targets=["TopoVolume#1"], scheme="map", firmness="hard")
    s.ops.mesh.run(targets=vol)
    s.fem.block_add(block=1, volumes=vol)
    s.fem.block_element_type(block=1, element_type="HEX8")
    info: sm.MeshInfo = s.meshes()[0]
    assert info.cells == 1000
    return s.mesh()


def centroid(m: sm.Mesh) -> npt.NDArray[np.float64]:
    return np.asarray(m.points.mean(axis=0), dtype=np.float64)


if __name__ == "__main__":
    print(centroid(build(sm.Session.memory())))

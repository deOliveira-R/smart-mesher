"""Acceptance 2, 3, 4: analytic tables of the 10-cube, session vs file, polyhedra by divergence."""

from __future__ import annotations

import pathlib

import numpy as np

import smartmesh as sm
from helpers import cell_volumes, edge_count


def test_ten_cube_tables_are_analytic(cube: sm.Session) -> None:
    m = cube.mesh()
    assert (m.n_vertices, m.n_faces, m.n_cells) == (1331, 3300, 1000)
    assert m.points.shape == (1331, 3)
    lo, hi = m.points.min(axis=0), m.points.max(axis=0)
    assert lo.tolist() == [-5.0] * 3 and hi.tolist() == [5.0] * 3  # the brick is centred on the origin
    idx, corners = m.cells()["hex"]
    assert corners.shape == (1000, 8) and idx.tolist() == list(range(1000))
    assert set(m.cells()) == {"hex"}
    assert (m.cell_shape == 1).all()
    assert (np.diff(m.cell_face_offsets.astype(np.int64)) == 6).all()
    assert edge_count(m) == 3630
    assert m.n_vertices - edge_count(m) + m.n_faces - m.n_cells == 1
    both = m.face_right != 0xFFFFFFFF
    assert (m.face_left[both] < m.face_right[both]).all()
    assert int((~both).sum()) == 600  # 6 sides of 10x10 boundary quads
    assert np.isclose(cell_volumes(m), 1.0, rtol=1e-12, atol=0).all()
    ids = m.class_entity_ids("cell")
    assert len(ids) == 1000 and {i.kind for i in ids} == {"TopoVolume"}
    info = cube.meshes()
    assert len(info) == 1 and info[0].cells == 1000 and info[0].vertices == 1331 and info[0].cell_shapes == {"Hex": 1000}


def test_session_tables_equal_the_smsh_file_and_are_read_only_snapshots(cube: sm.Session, tmp_path: pathlib.Path) -> None:
    path = tmp_path / "out.smsh"
    cube.export(path, format="smsh")
    m = cube.mesh()
    f = sm.smsh.open(str(path), verify=True)
    fm = f.mesh()
    for name, a in m.tables().items():
        b = fm.tables()[name]
        assert a.dtype == b.dtype and a.shape == b.shape, name
        assert a.tobytes() == b.tobytes(), name
    # file arrays are views over the map and read-only
    x = fm.x
    assert not x.flags.writeable and not x.flags.owndata
    assert x.base is not None
    # session arrays own their bytes (or a bytes-owning base) and are read-only
    for name, a in m.tables().items():
        assert not a.flags.writeable, name
        assert a.flags.owndata or a.base is not None, name
    try:
        m.x[0] = 1.0
    except ValueError:
        pass
    else:
        raise AssertionError("session array must be read-only")
    # snapshot semantics: a later delete leaves the earlier Mesh unchanged
    before = m.x.copy()
    cube.ops.mesh.delete(meshes=[m_id for m_id in [cube.meshes()[0].mesh]])
    assert cube.meshes() == []
    assert np.array_equal(m.x, before) and m.n_cells == 1000
    assert sm.Mesh.from_smsh(str(path)).n_cells == 1000
    f.close()


def test_polyhedra_volumes_by_divergence(s: sm.Session) -> None:
    b = s.model.create_brick(x=2.0)
    vol = b.outputs["volume"]
    s.ops.mesh.size(targets=vol, size=1.0)
    s.ops.mesh.scheme(targets=vol, scheme="tetmesh")
    tet = s.ops.mesh.run(targets=vol).outputs["mesh"][0]
    poly = s.ops.mesh.to_poly(mesh=tet).outputs["mesh"][0]
    tm, pm = s.mesh(tet), s.mesh(poly)
    assert (tm.cell_shape == 0).all() and tm.cell_corners.size == 4 * tm.n_cells
    assert (pm.cell_shape == 4).all()
    assert pm.cell_corners.size == 0 and (np.diff(pm.cell_corner_offsets.astype(np.int64)) == 0).all()
    assert set(pm.cells()) == {"polyhedron"} and pm.cells()["polyhedron"][1].shape == (pm.n_cells, 0)
    v = cell_volumes(pm)
    assert (v > 0).all()
    assert abs(v.sum() - 8.0) <= 8.0 * 1e-12
    assert abs(cell_volumes(tm).sum() - 8.0) <= 8.0 * 1e-12
    assert len(s.meshes()) == 2
    try:
        s.mesh()
    except sm.SmartMeshError as e:
        assert e.kind == "ambiguous-mesh" and e.data["meshes"] == [str(tet), str(poly)]
    else:
        raise AssertionError("two meshes need a name")


def test_no_mesh_error(s: sm.Session) -> None:
    try:
        s.mesh()
    except sm.ApplicationError as e:
        assert e.kind == "no-mesh"
    else:
        raise AssertionError("empty session has no mesh")

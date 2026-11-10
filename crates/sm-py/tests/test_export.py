"""`Session.export` options, `ExportNote.severity` and `ExportWarning` (W3c-EXPORT-C)."""

from __future__ import annotations

import pathlib
import subprocess
import warnings
from typing import Any

import pytest

import smartmesh as sm
from conftest import CRATE, smesh_bin
from helpers import slice_typed
from smartmesh.session import Session


def _poly(s: sm.Session) -> sm.EntityId:
    b = s.model.create_brick(x=2.0)
    vol = b.outputs["volume"]
    s.ops.mesh.size(targets=vol, size=1.0)
    s.ops.mesh.scheme(targets=vol, scheme="tetmesh")
    tet = s.ops.mesh.run(targets=vol).outputs["mesh"][0]
    poly: sm.EntityId = s.ops.mesh.to_poly(mesh=tet).outputs["mesh"][0]
    return poly


# A brick cut by three oblique planes into a general hexahedron, map-meshed: its interior faces
# are warped. (The dual POLY from `_poly` has no face above the warp tolerance.)
WARPED = """
let a = model.create_brick(x: 4.0, y: 4.0, z: 4.0, center: [2.0, 2.0, 2.0])
let c1 = model.webcut_plane(targets: a.body, plane: {origin: [3.0, 0.0, 0.0], normal: [1.0, 0.4, 0.0]})
let c2 = model.webcut_plane(targets: c1.negative, plane: {origin: [0.0, 0.0, 3.0], normal: [0.3, 0.1, 1.0]})
let c3 = model.webcut_plane(targets: c2.negative, plane: {origin: [0.0, 3.0, 0.0], normal: [0.2, 1.0, 0.3]})
let v = topo.related(kind: "TopoVolume", within: c3.negative)
mesh.size(targets: v, size: 0.5)
mesh.scheme(targets: v, scheme: "map")
let m = mesh.run(targets: v)
"""


def test_dagmc_graveyard_sugar_equals_options(cube: sm.Session, tmp_path: pathlib.Path) -> None:
    plain, a, b = (tmp_path / n for n in ("plain.h5m", "a.h5m", "b.h5m"))
    cube.export(plain, format="dagmc")
    cube.export(a, format="dagmc", graveyard=True)
    cube.export(b, format="dagmc", options={"graveyard": True})
    assert a.read_bytes() == b.read_bytes()
    assert a.read_bytes() != plain.read_bytes()


def test_graveyard_mapping_is_passed_through(cube: sm.Session, tmp_path: pathlib.Path) -> None:
    a, b = tmp_path / "a.h5m", tmp_path / "b.h5m"
    cube.export(a, format="dagmc", graveyard={"scale": 1.5})
    cube.export(b, format="dagmc", options={"graveyard": {"scale": 1.5}})
    assert a.read_bytes() == b.read_bytes()


def test_gmsh_binary_matches_cli(s: sm.Session, tmp_path: pathlib.Path) -> None:
    """The Python `binary=True` export is byte-identical to `smesh export --option binary=true`."""
    slice_typed(s)
    py_ascii, py_bin = tmp_path / "py_a.msh", tmp_path / "py_b.msh"
    s.export(py_ascii, format="gmsh")
    s.export(py_bin, format="gmsh", binary=True)
    exe = str(smesh_bin())
    proj = tmp_path / "proj"
    for args in (
        ["new", str(proj)],
        ["-p", str(proj), "run", str(CRATE / "tests" / "slice.smesh")],
        ["-p", str(proj), "export", str(tmp_path / "cli_a.msh"), "--format", "gmsh"],
        ["-p", str(proj), "export", str(tmp_path / "cli_b.msh"), "--format", "gmsh", "--option", "binary=true"],
    ):
        r = subprocess.run([exe, *args], capture_output=True, text=True, cwd=tmp_path)
        assert r.returncode == 0, (args, r.stdout, r.stderr)
    assert py_bin.read_bytes().startswith(b"$MeshFormat\n4.1 1 8\n")
    assert py_bin.read_bytes() == (tmp_path / "cli_b.msh").read_bytes()
    assert py_ascii.read_bytes() == (tmp_path / "cli_a.msh").read_bytes()


def test_gmsh_binary_and_polyhedra(s: sm.Session, tmp_path: pathlib.Path) -> None:
    slice_typed(s)
    txt, binf = tmp_path / "t.msh", tmp_path / "b.msh"
    s.export(txt, format="gmsh")
    s.export(binf, format="gmsh", binary=True)
    assert txt.read_bytes() != binf.read_bytes()
    # msh4 header `$MeshFormat` / `4.1 <file-type> 8`: file-type 1 is binary.
    assert binf.read_bytes().split(b"\n")[1].split()[1] == b"1"
    assert txt.read_bytes().split(b"\n")[1].split()[1] == b"0"
    other = tmp_path / "o.msh"
    s.export(other, format="gmsh", options={"binary": True})
    assert other.read_bytes() == binf.read_bytes()

    p = sm.Session.memory()
    pm = _poly(p)
    out = tmp_path / "poly.msh"
    with pytest.raises(sm.SmartMeshError) as ei:
        p.export(out, format="gmsh", mesh=pm)
    assert ei.value.kind == "polyhedra-unsupported"
    r = p.export(out, format="gmsh", polyhedra="drop", mesh=pm)
    assert out.read_bytes() and any(n.code == "msh.poly_dropped" for n in r.notes)


def test_option_given_twice_is_a_value_error_with_no_request(cube: sm.Session, tmp_path: pathlib.Path) -> None:
    before = (cube.status().nodes, cube.hash().state)
    out = tmp_path / "x.h5m"
    cases: list[dict[str, Any]] = [
        {"graveyard": True, "options": {"graveyard": False}},
        {"binary": True, "options": {"binary": True}},
        {"polyhedra": "drop", "options": {"polyhedra": "error"}},
    ]
    for kw in cases:
        with pytest.raises(ValueError):
            cube.export(out, format="dagmc", **kw)
    assert not out.exists()
    assert (cube.status().nodes, cube.hash().state) == before


def test_server_rejects_unknown_option_as_smartmesh_error(cube: sm.Session, tmp_path: pathlib.Path) -> None:
    with pytest.raises(sm.SmartMeshError) as ei:
        cube.export(tmp_path / "x.msh", format="gmsh", options={"nope": 1})
    assert ei.value.kind == "bad-request"
    with pytest.raises(sm.SmartMeshError) as ei:
        cube.export(tmp_path / "y.msh", format="gmsh", graveyard=True)
    assert ei.value.kind == "bad-request"


class _FakeTransport:
    """Returns a canned `session.export` result and records the request."""

    def __init__(self, result: dict[str, Any]) -> None:
        self.result = result
        self.calls: list[tuple[str, Any]] = []

    def request(self, method: str, params: Any) -> Any:
        self.calls.append((method, params))
        return self.result


def _fake(notes: list[dict[str, Any]]) -> tuple[Session, _FakeTransport]:
    t = _FakeTransport({"format": "smrt", "bytes": 1, "sha256": "00", "notes": notes})
    return Session(t), t


def test_severity_parsing_and_export_warning_with_a_canned_result() -> None:
    notes = [
        {"code": "a.info", "count": 2, "detail": "d1"},
        {"code": "smrt-region-names", "count": 1, "detail": "regions are named by volume id (no blocks)", "severity": "warning"},
    ]
    s, _ = _fake(notes)
    with pytest.warns(sm.ExportWarning, match="smrt-region-names: regions are named") as rec:
        r = s.export("a.smrt")
    assert len(rec) == 1 and issubclass(sm.ExportWarning, UserWarning)
    assert [n.severity for n in r.notes] == ["info", "warning"]
    assert [n.code for n in r.warnings] == ["smrt-region-names"]
    assert rec[0].filename == __file__  # stacklevel points at the caller

    clean, _ = _fake([notes[0]])
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        assert clean.export("a.smrt").warnings == []
    none, _ = _fake([])
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        none.export("a.smrt")


def test_request_shape_for_the_new_keywords() -> None:
    s, t = _fake([])
    s.export("a.case", format="openfoam", split_small_determinant=False, split_warped_faces=True, empty_patches=["e"], wedge_patches=("w",))
    assert t.calls[-1] == (
        "session.export",
        {
            "path": "a.case",
            "format": "openfoam",
            "split_small_determinant": False,
            "split_warped_faces": True,
            "empty_patches": ["e"],
            "wedge_patches": ["w"],
        },
    )
    s.export("a.h5m", format="dagmc", graveyard=True)
    assert t.calls[-1][1] == {"path": "a.h5m", "format": "dagmc", "options": {"graveyard": True}}
    s.export("a.smsh")
    assert t.calls[-1][1] == {"path": "a.smsh"}  # defaults send nothing new
    s.export("m.h5m", source="model", chord_tol=0.001)
    assert t.calls[-1][1] == {"path": "m.h5m", "options": {"chord_tol": 0.001}, "source": "model"}
    s.export("m.h5m", source="mesh")
    assert t.calls[-1][1] == {"path": "m.h5m", "source": "mesh"}
    with pytest.raises(ValueError, match="chord_tol"):
        s.export("m.h5m", source="model", chord_tol=0.1, options={"chord_tol": 0.2})


def test_smrt_no_blocks_warns(s: sm.Session, tmp_path: pathlib.Path) -> None:
    b = s.model.create_brick(x=2.0)
    vol = b.outputs["volume"]
    s.ops.mesh.size(targets=vol, size=1.0)
    s.ops.mesh.scheme(targets=vol, scheme="map")
    s.ops.mesh.run(targets=vol)
    with pytest.warns(sm.ExportWarning, match="smrt-region-names"):
        r = s.export(tmp_path / "a.smrt")
    assert r.warnings[0].severity == "warning"


def test_smrt_fully_blocked_is_silent(cube: sm.Session, tmp_path: pathlib.Path) -> None:
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        r = cube.export(tmp_path / "a.smrt")
    assert r.warnings == []


def test_split_warped_faces_flag(s: sm.Session, tmp_path: pathlib.Path) -> None:
    s.run_script(WARPED)
    r = s.export(tmp_path / "case", format="openfoam", split_warped_faces=True)
    assert any(n.code == "foam.warped_faces_split" for n in r.notes)
    with pytest.raises(sm.SmartMeshError) as ei:
        s.export(tmp_path / "x.msh", format="gmsh", split_warped_faces=True)
    assert ei.value.kind == "bad-request"


def test_mypy_rejects_bad_export_keywords(tmp_path: pathlib.Path) -> None:
    import subprocess
    import sys

    src = tmp_path / "bad.py"
    src.write_text(
        "import smartmesh as sm\n\n\ndef f(s: sm.Session) -> None:\n"
        '    s.export("a.msh", polyhedra="fix")\n'
        '    s.export("a.msh", binary="yes")\n'
    )
    r = subprocess.run([sys.executable, "-m", "mypy", "--strict", str(src)], capture_output=True, text=True)
    assert r.returncode == 1, r.stdout
    assert r.stdout.count("error:") == 2 and "polyhedra" in r.stdout and "binary" in r.stdout, r.stdout


def test_export_is_deterministic_between_calls(cube: sm.Session, tmp_path: pathlib.Path) -> None:
    a, b = tmp_path / "a.h5m", tmp_path / "b.h5m"
    ra = cube.export(a, format="dagmc", graveyard=True)
    rb = cube.export(b, format="dagmc", graveyard=True)
    assert ra.sha256 == rb.sha256 and a.read_bytes() == b.read_bytes()


PIN = """
let br = model.create_brick(x: 1.26, y: 1.26, z: 1.0)
let fu = model.create_cylinder(height: 1.0, radius: 0.4)
let tl = model.create_cylinder(height: 1.0, radius: 0.4)
let mo = model.subtract(targets: br.body, tools: tl.body)
model.imprint(bodies: all bodies)
model.merge()
fem.block_add(block: 1, volumes: fu.volume, name: "fuel")
fem.block_add(block: 2, volumes: mo.volumes, name: "moderator")
"""


def test_source_model_exports_dagmc_without_a_mesh(s: sm.Session, tmp_path: pathlib.Path) -> None:
    """`source="model"` faces the B-rep: no mesh in the session, deterministic bytes, typed errors."""
    s.run_script(PIN)
    a, b = tmp_path / "a.h5m", tmp_path / "b.h5m"
    ra = s.export(a, source="model", chord_tol=1e-3)
    rb = s.export(b, source="model", options={"chord_tol": 1e-3})
    assert a.read_bytes() == b.read_bytes() and ra.sha256 == rb.sha256
    assert a.read_bytes()[:8] == b"\x89HDF\r\n\x1a\n"
    coarse = s.export(tmp_path / "c.h5m", source="model", chord_tol=1e-2)
    assert coarse.bytes < ra.bytes, "a finer chord tolerance writes more triangles"
    with pytest.raises(sm.SmartMeshError) as ei:  # the mesh path has no mesh
        s.export(tmp_path / "d.h5m")
    assert ei.value.kind == "no-mesh"
    with pytest.raises(sm.SmartMeshError) as ei:
        s.export(tmp_path / "e.msh", format="gmsh", source="model")
    assert ei.value.kind == "unsupported-source"
    with pytest.raises(sm.SmartMeshError) as ei:
        s.export(tmp_path / "f.h5m", chord_tol=1e-3)  # a mesh is not re-faceted
    assert ei.value.kind == "bad-request"

"""Project sessions: create, close (saves), reopen with a strict replay, staged STEP inputs."""

from __future__ import annotations

import pathlib

import pytest

import smartmesh as sm
from conftest import step_path
from helpers import slice_typed


def test_create_close_reopen_replays_to_the_same_hash(tmp_path: pathlib.Path) -> None:
    proj = tmp_path / "proj"
    with sm.Session.open(proj, create=True) as s:
        want = slice_typed(s)
        assert s.status().project == "proj"
    with sm.Session.open(proj) as again:
        assert again.hash().state == want
        assert again.status().depth == 6
        assert again.mesh().n_cells == 1000


def test_closed_session_refuses_calls(tmp_path: pathlib.Path) -> None:
    s = sm.Session.create(tmp_path / "p", "named")
    s.close()
    s.close()  # idempotent
    with pytest.raises(sm.SmartMeshError) as e:
        s.status()
    assert e.value.kind == "closed"


def test_project_replays_a_staged_step_import_from_its_directory_alone(tmp_path: pathlib.Path) -> None:
    proj = tmp_path / "step"
    with sm.Session.create(proj, "step") as s:
        r = s.import_step(step_path())
        want = s.hash().state
        assert len(r.outputs["bodies"]) == 3
    with sm.Session.open(proj) as again:
        assert again.hash().state == want


def test_opening_a_missing_project_is_an_application_error(tmp_path: pathlib.Path) -> None:
    with pytest.raises(sm.SmartMeshError):
        sm.Session.open(tmp_path / "nope")

"""Acceptance 8: STEP import through the staged-input path, equal to the inline form."""

from __future__ import annotations

import pytest

import smartmesh as sm
from conftest import step_path

VOLUMES = [1200.0, 200.0, 600.0]


def _volumes(s: sm.Session, r: sm.ApplyResult) -> list[float]:
    return [float(s.query("model.measure", entity=v)["volume"]) for v in r.outputs["volumes"]]


def test_import_step_three_bodies_and_the_three_spellings_agree() -> None:
    a = sm.Session.memory()
    ra = a.import_step(step_path())
    assert len(ra.outputs["bodies"]) == 3
    assert _volumes(a, ra) == pytest.approx(VOLUMES, rel=1e-12)

    b = sm.Session.memory()
    rb = b.import_step_inline(step_path())
    assert _volumes(b, rb) == pytest.approx(VOLUMES, rel=1e-12)

    c = sm.Session.memory()
    ref = c.stage_file(step_path())
    rc = c.apply("model.import_step", source={"sha256": ref.sha256})
    assert ref.bytes == step_path().stat().st_size

    assert a.hash().state == b.hash().state == c.hash().state
    assert ra.state == rb.state == rc.state


def test_staging_is_per_session() -> None:
    a, b = sm.Session.memory(), sm.Session.memory()
    ref = a.stage_file(step_path())
    with pytest.raises(sm.ApplicationError):
        b.apply("model.import_step", source={"sha256": ref.sha256})  # b never staged it
    assert b.status().depth == 0

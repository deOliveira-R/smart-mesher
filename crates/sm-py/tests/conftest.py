"""Shared fixtures. Paths come from environment variables set by `ci/py.sh`, with in-repo defaults."""

from __future__ import annotations

import os
import pathlib
import sys

import pytest

CRATE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import smartmesh as sm  # noqa: E402


def smesh_bin() -> pathlib.Path:
    """The `smesh` binary (`SMESH_BIN`, else the workspace debug build). A missing binary fails."""
    p = pathlib.Path(os.environ.get("SMESH_BIN", CRATE.parents[1] / "target" / "debug" / "smesh"))
    if not p.is_file():
        pytest.fail(f"smesh binary not found at {p}; run `cargo build -p sm-cli` or set SMESH_BIN")
    return p


def step_path() -> pathlib.Path:
    """The W3a STEP part (`SM_TESTKIT_STEP`, else the testkit data directory)."""
    p = pathlib.Path(os.environ.get("SM_TESTKIT_STEP", CRATE.parent / "sm-testkit" / "data" / "step" / "w3a_part.step"))
    if not p.is_file():
        pytest.fail(f"STEP file not found at {p}")
    return p


@pytest.fixture
def s() -> sm.Session:
    return sm.Session.memory()


@pytest.fixture
def cube(s: sm.Session) -> sm.Session:
    """A session holding the meshed 10-cube at size 1 (the ARCHITECTURE section 8 slice)."""
    from helpers import slice_typed

    slice_typed(s)
    return s

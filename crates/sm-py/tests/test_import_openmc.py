"""W4-CSG-CLI: `Session.import_openmc` gives the state hash of `smesh import-openmc` (the Python leg
of the four-front-end row; the Rust leg is `crates/sm-cli/tests/csg_cli.rs`)."""

from __future__ import annotations

import json
import pathlib
import subprocess

import pytest

import smartmesh as sm
from conftest import CRATE, smesh_bin

PIN = CRATE.parent / "sm-csgio" / "tests" / "fixtures" / "openmc" / "pin"

TWO_BOXES = (
    '<?xml version="1.0"?><geometry>'
    '<cell id="1" material="1" region="1 -2 3 -4 5 -6" universe="1"/>'
    '<cell id="2" material="2" region="7 -8 3 -4 5 -6" universe="1"/>'
    '<surface id="1" type="x-plane" boundary="vacuum" coeffs="0"/><surface id="2" type="x-plane" coeffs="5.5"/>'
    '<surface id="7" type="x-plane" coeffs="5"/><surface id="8" type="x-plane" boundary="vacuum" coeffs="10"/>'
    '<surface id="3" type="y-plane" boundary="vacuum" coeffs="0"/><surface id="4" type="y-plane" boundary="vacuum" coeffs="10"/>'
    '<surface id="5" type="z-plane" boundary="vacuum" coeffs="0"/><surface id="6" type="z-plane" boundary="vacuum" coeffs="10"/>'
    "</geometry>"
)


def _cli(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run([str(smesh_bin()), "--memory", "--json", *args], capture_output=True, text=True, check=False)


def test_pin_deck_state_hash_equals_the_cli() -> None:
    s = sm.Session.memory()
    r = s.import_openmc(PIN / "geometry.xml", PIN / "materials.xml")
    assert len(r.outputs["deck_volumes"]) == 4
    cli = _cli("import-openmc", str(PIN / "geometry.xml"), "--materials", str(PIN / "materials.xml"))
    assert cli.returncode == 0, cli.stderr
    assert json.loads(cli.stdout)["state"] == r.state == s.hash().state


def test_overlap_raises_by_default_and_reports_on_request(tmp_path: pathlib.Path) -> None:
    deck = tmp_path / "geometry.xml"
    deck.write_text(TWO_BOXES)
    with pytest.raises(sm.ApplicationError, match="csg.overlap"):
        sm.Session.memory().import_openmc(deck)
    s = sm.Session.memory()
    r = s.import_openmc(deck, overlap="report")
    assert [d.code for d in r.diagnostics].count("import.overlap") == 1
    # The CLI with the same policy reaches the same state.
    cli = _cli("import-openmc", str(deck), "--overlap", "report")
    assert cli.returncode == 0, cli.stderr
    assert json.loads(cli.stdout)["state"] == r.state


def test_export_regions_is_sugar_for_the_option() -> None:
    s = sm.Session.memory()
    s.import_openmc(PIN / "geometry.xml")
    with pytest.raises(ValueError, match="regions"):
        s.export(format="smsh", inline=True, options={"regions": {"split": []}}, regions=["lattice"])
    with pytest.raises(sm.SmartMeshError, match="unknown part"):
        s.export(format="smsh", inline=True, regions=["bogus"])  # type: ignore[list-item]

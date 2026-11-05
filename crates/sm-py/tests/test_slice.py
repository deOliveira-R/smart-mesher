"""Acceptance 1: slice parity across cmd, typed ops and run_script, `smesh run` and MCP; timeline."""

from __future__ import annotations

import json
import pathlib
import subprocess

import pytest

import smartmesh as sm
from conftest import smesh_bin
from helpers import SLICE_LINES, SLICE_SCRIPT, slice_typed


def _cmd_hash() -> str:
    s = sm.Session.memory()
    for line in SLICE_LINES:
        s.cmd(line)
    return s.hash().state


def _script_hash() -> str:
    s = sm.Session.memory()
    r = s.run_script(SLICE_SCRIPT)
    assert r.state == s.hash().state
    return r.state


def _typed_hash() -> str:
    s = sm.Session.memory()
    h = slice_typed(s)
    assert h == s.hash().state
    return h


def _smesh_run_hash(tmp: pathlib.Path) -> str:
    f = tmp / "slice.smesh"
    f.write_text(SLICE_SCRIPT)
    out = subprocess.run([str(smesh_bin()), "run", str(f), "--memory", "--print-hash"], capture_output=True, text=True, check=True)
    return out.stdout.strip()


def _mcp_hash() -> str:
    def call(i: int, name: str, args: dict[str, object]) -> dict[str, object]:
        return {"jsonrpc": "2.0", "id": i, "method": "tools/call", "params": {"name": name, "arguments": args}}

    lines = [
        {"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "py", "version": "0"}}},
        {"jsonrpc": "2.0", "method": "notifications/initialized"},
        call(3, "model__create_brick", {"x": 10, "y": 10, "z": 10}),
        call(4, "mesh__size", {"targets": ["TopoVolume#1"], "size": 1.0}),
        call(5, "mesh__scheme", {"targets": ["TopoVolume#1"], "scheme": "map"}),
        call(6, "mesh__run", {"targets": ["TopoVolume#1"]}),
        call(7, "fem__block_add", {"block": 1, "volumes": ["TopoVolume#1"]}),
        call(8, "fem__block_element_type", {"block": 1, "element_type": "HEX8"}),
        call(11, "session_hash", {}),
    ]
    out = subprocess.run(
        [str(smesh_bin()), "mcp", "--memory"], input="".join(json.dumps(x) + "\n" for x in lines), capture_output=True, text=True, check=True
    )
    replies = {json.loads(ln)["id"]: json.loads(ln) for ln in out.stdout.splitlines() if ln.strip()}
    res = replies[11]["result"]
    assert res["isError"] is False
    state = res["structuredContent"]["state"]
    assert isinstance(state, str)
    return state


def test_three_spellings_give_one_hash_equal_to_cli_and_mcp(tmp_path: pathlib.Path) -> None:
    h_cmd, h_typed, h_script = _cmd_hash(), _typed_hash(), _script_hash()
    assert h_cmd == h_typed == h_script
    assert len(h_cmd) == 64
    assert _smesh_run_hash(tmp_path) == h_cmd
    assert _mcp_hash() == h_cmd


def test_batch_is_one_node_with_the_same_state(s: sm.Session) -> None:
    with s.batch() as b:
        brick = b.model.create_brick(x=10.0, y=10.0, z=10.0)
        vol = brick.ref("volume", 0)
        b.ops.mesh.size(targets=[vol], size=1.0)
        b.ops.mesh.scheme(targets=[vol], scheme="map")
        b.ops.mesh.run(targets=[vol])
        b.fem.block_add(block=1, volumes=[vol])
        b.fem.block_element_type(block=1, element_type="HEX8")
    assert b.result is not None
    assert s.status().depth == 1
    assert s.hash().state == _typed_hash()


def test_undo_redo_checkout_and_branch_restore_hashes(s: sm.Session) -> None:
    s.cmd(SLICE_LINES[0])
    base = s.status()
    assert base.position is not None
    for line in SLICE_LINES[1:]:
        s.cmd(line)
    full = s.hash().state
    # undo one node, redo it
    s.undo()
    assert s.hash().state != full
    s.redo()
    assert s.hash().state == full
    # branch at the brick, rebuild the same ops, equal hash; main still reachable
    s.branch("alt", at=base.position)
    assert s.hash().state == base.state
    for line in SLICE_LINES[1:]:
        s.cmd(line)
    assert s.hash().state == full
    s.checkout("main")
    assert s.hash().state == full
    # checkout by node hash and to the empty state
    s.checkout(base.position)
    assert s.hash().state == base.state
    s.checkout("empty")
    assert s.status().depth == 0
    s.checkout("main")
    assert s.hash().state == full
    assert [e.op for e in s.log(3)][0] == "fem.block_element_type"
    assert s.hash(at="alt").state == full


def test_two_sessions_are_independent() -> None:
    a, b = sm.Session.memory(), sm.Session.memory()
    a.cmd(SLICE_LINES[0])
    assert b.status().depth == 0
    assert a.status().depth == 1


@pytest.mark.parametrize("fixed", [("py", "2026-10-24T00:00:00Z")])
def test_fixed_time_makes_node_hashes_reproducible(fixed: tuple[str, str]) -> None:
    nodes = []
    for _ in range(2):
        s = sm.Session.memory(author=fixed[0], fixed_time=fixed[1])
        nodes.append(s.model.create_brick(x=10.0).node)
    assert nodes[0] == nodes[1]

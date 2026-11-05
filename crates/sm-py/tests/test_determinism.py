"""Acceptance 7: same hashes across runs, thread counts and concurrent use; plus a spawn smoke test."""

from __future__ import annotations

import json
import multiprocessing
import os
import pathlib
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import smartmesh as sm
from helpers import SLICE_LINES, slice_typed, tables_digest

HERE = pathlib.Path(__file__).resolve().parent


def _run(env_extra: dict[str, str]) -> dict[str, str]:
    env = {k: v for k, v in os.environ.items() if k != "RAYON_NUM_THREADS"}
    env.update(env_extra)
    env["PYTHONPATH"] = str(HERE) + os.pathsep + env.get("PYTHONPATH", "")
    out = subprocess.run([sys.executable, str(HERE / "_det_script.py")], capture_output=True, text=True, env=env, check=True, cwd=HERE)
    result: dict[str, str] = json.loads(out.stdout.strip().splitlines()[-1])
    return result


def test_two_runs_and_one_thread_versus_default_agree() -> None:
    a, b = _run({}), _run({})
    one = _run({"RAYON_NUM_THREADS": "1"})
    many = _run({"RAYON_NUM_THREADS": "8"})
    assert a == b == one == many
    assert len(a["state"]) == 64 and len(a["tables"]) == 64


def test_in_process_repeat_gives_identical_tables() -> None:
    digests = []
    for _ in range(2):
        s = sm.Session.memory()
        slice_typed(s)
        digests.append(tables_digest(s.mesh()))
    assert digests[0] == digests[1]


def test_eight_threads_of_queries_and_one_writer_match_the_serial_run() -> None:
    serial = sm.Session.memory()
    for line in SLICE_LINES:
        serial.cmd(line)
    want = serial.hash().state

    s = sm.Session.memory()
    stop = False

    def reader(_: int) -> int:
        n = 0
        while not stop:
            s.hash()
            s.status()
            s.catalog()
            n += 1
        return n

    def writer() -> str:
        nonlocal stop
        try:
            for line in SLICE_LINES:
                s.cmd(line)
        finally:
            stop = True
        return s.hash().state

    with ThreadPoolExecutor(max_workers=8) as pool:
        readers = [pool.submit(reader, i) for i in range(7)]
        w = pool.submit(writer)
        got = w.result(timeout=300)
        for r in readers:
            assert r.result(timeout=300) >= 0
    assert got == want == s.hash().state


def _spawned(q: "multiprocessing.Queue[str]") -> None:
    import smartmesh as sm2

    q.put(sm2.Session.memory().model.create_brick(x=2.0).state)


def test_spawned_process_gives_the_same_hash() -> None:
    ctx = multiprocessing.get_context("spawn")
    q: multiprocessing.Queue[str] = ctx.Queue()
    p = ctx.Process(target=_spawned, args=(q,))
    p.start()
    child = q.get(timeout=300)
    p.join(timeout=60)
    assert child == sm.Session.memory().model.create_brick(x=2.0).state

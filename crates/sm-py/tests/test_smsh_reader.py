"""The numpy `.smsh` reader checks what `SMSH_FORMAT.md` section 1 asks a reader to check."""

from __future__ import annotations

import json
import pathlib
import struct

import numpy as np
import pytest

import smartmesh as sm


@pytest.fixture
def smsh_file(cube: sm.Session, tmp_path: pathlib.Path) -> pathlib.Path:
    p = tmp_path / "m.smsh"
    cube.export(p, format="smsh")
    return p


def _rewrite_header(src: pathlib.Path, dst: pathlib.Path, edit) -> None:  # type: ignore[no-untyped-def]
    raw = src.read_bytes()
    (hl,) = struct.unpack_from("<Q", raw, 8)
    hdr = json.loads(raw[16 : 16 + hl])
    edit(hdr)
    body = json.dumps(hdr, separators=(",", ":")).encode()
    body += b" " * ((-(16 + len(body))) % 64)
    # keep the data region where it was by padding to the original header length
    assert len(body) <= hl, "edited header must not grow"
    body += b" " * (hl - len(body))
    dst.write_bytes(raw[:8] + struct.pack("<Q", hl) + body + raw[16 + hl :])


def test_reads_every_always_present_array_with_crc(smsh_file: pathlib.Path) -> None:
    f = sm.smsh.open(str(smsh_file), verify=True)
    for name in sm.mesh.TABLE_NAMES:  # type: ignore[attr-defined]
        assert name in f.arrays
    assert f.arrays["x"].dtype == np.dtype("<f8") and f.arrays["cell_faces"].dtype == np.dtype("<u4")
    assert f.header["mesh"]["n_cells"] == 1000


def test_rejects_bad_magic_major_and_sizes(smsh_file: pathlib.Path, tmp_path: pathlib.Path) -> None:
    bad = tmp_path / "bad.smsh"
    bad.write_bytes(b"NOTSMSH!" + smsh_file.read_bytes()[8:])
    with pytest.raises(sm.smsh.SmshError):
        sm.smsh.open(str(bad))

    def major(h: dict) -> None:  # type: ignore[type-arg]
        h["version"]["major"] = 2

    _rewrite_header(smsh_file, bad, major)
    with pytest.raises(sm.smsh.SmshError, match="major"):
        sm.smsh.open(str(bad))

    def nbytes(h: dict) -> None:  # type: ignore[type-arg]
        h["arrays"][0]["nbytes"] += 8

    _rewrite_header(smsh_file, bad, nbytes)
    with pytest.raises(sm.smsh.SmshError, match="nbytes"):
        sm.smsh.open(str(bad))

    trunc = tmp_path / "trunc.smsh"
    trunc.write_bytes(smsh_file.read_bytes()[:-1024])
    with pytest.raises(sm.smsh.SmshError):
        sm.smsh.open(str(trunc))


def test_crc_failure_is_detected_only_when_asked(smsh_file: pathlib.Path, tmp_path: pathlib.Path) -> None:
    raw = bytearray(smsh_file.read_bytes())
    raw[-3] ^= 0xFF
    bad = tmp_path / "flip.smsh"
    bad.write_bytes(bytes(raw))
    sm.smsh.open(str(bad))
    with pytest.raises(sm.smsh.SmshError, match="checksum"):
        sm.smsh.open(str(bad), verify=True)

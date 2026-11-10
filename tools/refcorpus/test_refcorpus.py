"""Tests for build.py that need no OpenMC, FreeCAD or network.

python3 -I -B -m unittest discover -s tools/refcorpus -p 'test_*.py'
"""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build


class ManifestTests(unittest.TestCase):
    def setUp(self) -> None:
        self.m = build.load_manifest()

    def test_roundtrip_is_byte_identical(self) -> None:
        self.assertEqual(build.dump_manifest(self.m), build.MANIFEST.read_text())

    def test_acceptance_counts(self) -> None:
        cats: dict[str, list[str]] = {}
        for e in self.m["entry"]:
            cats.setdefault(e["category"], []).append(e["id"])
        self.assertGreaterEqual(len(cats["openmc"]), 6)
        self.assertIn("openmc-hex-x-7ring", cats["openmc"])
        self.assertIn("openmc-hex-y-7ring", cats["openmc"])
        self.assertGreaterEqual(len(cats["beavrs"]), 3)
        self.assertGreaterEqual(len(cats["vtb"]), 3)
        self.assertGreaterEqual(len(cats["step-clean"]), 3)
        self.assertGreaterEqual(len(cats["step-dirty"]), 7)

    def test_every_entry_is_complete(self) -> None:
        for e in self.m["entry"]:
            for k in ("id", "category", "recipe", "source", "licence", "outputs"):
                self.assertIn(k, e, e["id"])
            if e["category"] != "moose":
                self.assertTrue(all(len(o["sha256"]) == 64 for o in e["outputs"]), e["id"])
            if "deck" in e:
                self.assertTrue(
                    e["expected"] == "accepted" or e["expected"].split(":")[0] in ("unsupported", "error", "parse", "invalid"), e["id"]
                )
            if e["category"] == "step-dirty":
                self.assertTrue(e["expected"], e["id"])  # the heal_check codes

    def test_committed_files_are_small(self) -> None:
        for p in build.HERE.iterdir():
            if p.is_file():
                self.assertLess(p.stat().st_size, 100_000, p.name)


class CheckTests(unittest.TestCase):
    def test_out_dir_in_repo_is_refused(self) -> None:
        for p in (build.REPO, build.REPO / "target" / "x", build.HERE / "out"):
            with self.assertRaises(build.CorpusError):
                build.check_out_dir(p)
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(build.check_out_dir(Path(d)), Path(d).resolve())

    def test_corrupted_file_is_named(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            (root / "a").mkdir()
            f = root / "a" / "x.xml"
            f.write_bytes(b"<geometry/>")
            m = {"entry": [{"id": "a", "outputs": [{"path": "a/x.xml", "sha256": build.sha256_file(f)}]}]}
            self.assertEqual(build.check_outputs(m, root), [])
            f.write_bytes(b"<geometry />")
            bad = build.check_outputs(m, root)
            self.assertEqual(len(bad), 1)
            self.assertTrue(bad[0].startswith("a/x.xml"))
            f.unlink()
            self.assertIn("missing", build.check_outputs(m, root)[0])


class RefusalTests(unittest.TestCase):
    def test_codes_drop_lines_ids_and_free_text(self) -> None:
        c = build.refusal_code
        self.assertEqual(c("unsupported (line 9): hex_lattice: <hex_lattice> is not supported (rectangular)"), "unsupported:hex_lattice")
        self.assertEqual(c("unsupported (line 77): hex_lattice: <hex_lattice> is not supported"), "unsupported:hex_lattice")
        self.assertEqual(
            c('unsupported (line 5): cell 20733: a material list ("10 11"); one material per cell'), "unsupported:material-list"
        )
        self.assertEqual(c("parse error at 3:4: unexpected end"), "parse")
        self.assertEqual(c("the flat model has more than 2097152 cells"), "error:flat-model-has-more-than")


if __name__ == "__main__":
    unittest.main()

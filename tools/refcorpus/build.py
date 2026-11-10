"""Reference corpus builder (W4-REFCORPUS, ruling D8). See docs/design/w4/refcorpus.md.

    python3 -I tools/refcorpus/build.py                 # fetch (unless --offline), then build
    python3 -I tools/refcorpus/build.py fetch           # the only step that uses the network
    python3 -I tools/refcorpus/build.py build --offline # build from the cache
    python3 -I tools/refcorpus/build.py --check         # recompute sha256 of every output
    python3 -I tools/refcorpus/build.py expected --smcsg PATH [--write]
    python3 -I tools/refcorpus/build.py bless [--smcsg PATH]   # record sha256 (and expected)
    python3 -I tools/refcorpus/build.py selftest        # --check on a corrupted copy exits 1

The models are written under ``${SM_REFCORPUS_DIR:-~/.cache/smart-mesh/refcorpus}``, never into
the repository (the script refuses). Standard library only; OpenMC and FreeCAD run as
subprocesses (``SM_OPENMC_PYTHON``, ``SM_FREECADCMD``). Exit codes: 0 ok, 1 a check failed,
2 usage or environment error.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tomllib
import urllib.request
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
MANIFEST = HERE / "manifest.toml"
Entry = dict[str, Any]
Manifest = dict[str, Any]


class CorpusError(Exception):
    """A usage or environment problem (exit code 2)."""


# ---- manifest ---------------------------------------------------------------------------------

HEADER = (
    "# Reference corpus manifest (W4-REFCORPUS). Written by `build.py bless`; edit metadata by hand,\n"
    "# sha256 values only through bless. See docs/design/w4/refcorpus.md.\n"
)
ENTRY_KEYS = [
    "id", "category", "recipe", "source", "licence", "tool", "optional", "deck", "needs",
    "fetch_paths", "expected", "expected_basis", "note",
]  # fmt: skip


def load_manifest(path: Path = MANIFEST) -> Manifest:
    with path.open("rb") as f:
        return tomllib.load(f)


def toml_value(v: Any) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, int):
        return str(v)
    if isinstance(v, str):
        return json.dumps(v)
    if isinstance(v, list):
        return "[" + ", ".join(toml_value(x) for x in v) + "]"
    raise TypeError(f"cannot write {v!r} as TOML")


def dump_manifest(m: Manifest) -> str:
    """Deterministic TOML: fixed key order, entries in file order."""
    out = [HEADER, "[meta]"]
    for k, v in m["meta"].items():
        out.append(f"{k} = {toml_value(v)}")
    for s in m.get("support", []):
        out += ["", "[[support]]"] + [f"{k} = {toml_value(s[k])}" for k in ("source", "repo", "commit", "path", "sha256")]
    for e in m["entry"]:
        out += ["", "[[entry]]"]
        extra = sorted(k for k in e if k not in ENTRY_KEYS and k != "outputs")
        assert not extra, f"{e['id']}: unknown keys {extra}"
        out += [f"{k} = {toml_value(e[k])}" for k in ENTRY_KEYS if k in e]
        for o in e["outputs"]:
            out += ["", "[[entry.outputs]]", f"path = {toml_value(o['path'])}", f"sha256 = {toml_value(o['sha256'])}"]
    return "\n".join(out) + "\n"


# ---- paths and tools --------------------------------------------------------------------------


def default_out() -> Path:
    env = os.environ.get("SM_REFCORPUS_DIR")
    return Path(env) if env else Path.home() / ".cache" / "smart-mesh" / "refcorpus"


def check_out_dir(out: Path) -> Path:
    """Resolve ``out`` and refuse anything inside the repository."""
    r = out.expanduser().resolve()
    if r == REPO or REPO in r.parents:
        raise CorpusError(f"refusing to write into the repository: {r} (set SM_REFCORPUS_DIR)")
    return r


def find_tool(env: str, candidates: list[str], what: str) -> Path:
    paths = [os.environ[env]] if env in os.environ else []
    paths += [str(Path(c).expanduser()) for c in candidates]
    for p in paths:
        if Path(p).is_file():
            return Path(p)
    which = shutil.which(Path(candidates[-1]).name) if candidates else None
    if which:
        return Path(which)
    raise CorpusError(f"{what} not found; set {env} (tried {', '.join(paths)})")


def openmc_python() -> Path:
    home = ["~/opt/miniforge/envs/openmc", "~/miniforge3/envs/openmc", "~/mambaforge/envs/openmc", "~/miniconda3/envs/openmc"]
    return find_tool("SM_OPENMC_PYTHON", [f"{h}/bin/python" for h in home], "the OpenMC conda env python")


def freecadcmd() -> Path:
    return find_tool("SM_FREECADCMD", ["/Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd", "freecadcmd"], "freecadcmd")


def sha256_file(p: Path) -> str:
    h = hashlib.sha256()
    with p.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def run(cmd: list[str], cwd: Path, env_extra: dict[str, str] | None = None) -> str:
    env = {k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", "PYTHONHOME")}
    env.update({"PYTHONDONTWRITEBYTECODE": "1", "PYTHONHASHSEED": "0"})
    env.update(env_extra or {})
    cwd.mkdir(parents=True, exist_ok=True)
    r = subprocess.run(cmd, cwd=cwd, env=env, capture_output=True, text=True, check=False)
    if r.returncode != 0:
        raise CorpusError(f"{' '.join(cmd[:3])} ... failed (rc {r.returncode}):\n{r.stdout[-2000:]}\n{r.stderr[-2000:]}")
    return r.stdout


# ---- fetch ------------------------------------------------------------------------------------


def raw_url(repo: str, commit: str, path: str) -> str:
    return f"https://raw.githubusercontent.com/{repo}/{commit}/{path}"


def cache_path(out: Path, repo: str, commit: str, path: str) -> Path:
    return out / ".cache" / repo.replace("/", "__") / commit / path


def split_source(e: Entry) -> tuple[str, str]:
    """(``owner/repo``, commit) of an entry whose source is ``https://github.com/owner/repo@commit``."""
    url, commit = e["source"].split("@")
    return url.removeprefix("https://github.com/"), commit


def source_files(m: Manifest, entries: list[Entry]) -> list[tuple[str, str, str, str]]:
    """(repo, commit, path, sha256) of every file the selected entries need."""
    files: list[tuple[str, str, str, str]] = []
    needs = {n for e in entries for n in e.get("needs", [])}
    for s in m.get("support", []):
        if s["source"] in needs:
            files.append((s["repo"], s["commit"], s["path"], s["sha256"]))
    pinned = {  # a downloaded file is pinned by the copy entry that publishes it
        (*split_source(e), p): o["sha256"]
        for e in m["entry"]
        if e["recipe"] == "copy"
        for p in e["fetch_paths"]
        for o in e["outputs"]
        if o["path"] == f"{e['id']}/{p}"
    }
    for e in entries:
        if e["recipe"] in ("copy", "moose"):
            repo, commit = split_source(e)
            files += [(repo, commit, p, pinned.get((repo, commit, p), "")) for p in e["fetch_paths"]]
    return files


def fetch(m: Manifest, entries: list[Entry], out: Path, allow_unpinned: bool) -> None:
    for repo, commit, path, sha in source_files(m, entries):
        dest = cache_path(out, repo, commit, path)
        if dest.is_file() and (not sha or sha256_file(dest) == sha):
            continue
        if not sha and not allow_unpinned:
            raise CorpusError(f"{repo}:{path} has no pinned sha256 (run `bless`)")
        dest.parent.mkdir(parents=True, exist_ok=True)
        url = raw_url(repo, commit, path)  # fixed https host
        req = urllib.request.Request(url, headers={"User-Agent": "smart-mesh-refcorpus"})  # noqa: S310
        with urllib.request.urlopen(req, timeout=120) as r:  # noqa: S310
            data = r.read()
        got = hashlib.sha256(data).hexdigest()
        if sha and got != sha:
            raise CorpusError(f"{repo}@{commit}:{path}: sha256 {got} differs from the pinned {sha}")
        dest.write_bytes(data)
        print(f"fetched {repo}@{commit[:8]}:{path}")


# ---- build ------------------------------------------------------------------------------------


def build_entry(m: Manifest, e: Entry, out: Path) -> bool:
    """Build one entry into ``out/<id>``. Returns False when an optional entry is skipped."""
    d = out / e["id"]
    if d.exists():
        shutil.rmtree(d)
    d.mkdir(parents=True)
    recipe = e["recipe"]
    if recipe == "openmc":
        run([str(openmc_python()), "-I", "-B", str(HERE / "gen_openmc.py"), e["id"], str(d)], out)
    elif recipe == "step":
        # FreeCAD's embedded Python ignores PYTHONDONTWRITEBYTECODE, so run a copy under out/ to keep
        # bytecode out of the repository.
        sd = out / ".scripts"
        sd.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(HERE / "gen_step.py", sd / "gen_step.py")
        run([str(freecadcmd()), str(sd / "gen_step.py")], out, {"SM_RC_STEP_OUT": str(d), "SM_RC_STEP_IDS": e["id"]})
    elif recipe == "beavrs":
        sup = next(s for s in m["support"] if s["source"] == "beavrs")
        src = cache_path(out, sup["repo"], sup["commit"], "models/openmc")
        run([str(openmc_python()), "-I", "-B", str(HERE / "gen_beavrs.py"), e["id"], str(d)], out, {"SM_RC_BEAVRS_SRC": str(src)})
    elif recipe == "copy":
        repo, commit = split_source(e)
        for p in e["fetch_paths"]:
            dest = d / p
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(cache_path(out, repo, commit, p), dest)
    elif recipe == "moose":
        exe = os.environ.get("SM_MOOSE_EXE")
        if not exe:
            shutil.rmtree(d)
            print(f"skipped {e['id']}: SM_MOOSE_EXE is not set (optional)")
            return False
        repo, commit = split_source(e)
        inp = cache_path(out, repo, commit, e["fetch_paths"][0])
        run([exe, "-i", str(inp), "--mesh-only", str(d / "mesh.e")], inp.parent)
    else:
        raise CorpusError(f"{e['id']}: unknown recipe {recipe!r}")
    return True


def select(m: Manifest, only: str | None, include_optional: bool) -> list[Entry]:
    es = [e for e in m["entry"] if include_optional or not e.get("optional", False)]
    if only:
        es = [e for e in m["entry"] if e["id"] == only or e["category"] == only]
        if not es:
            raise CorpusError(f"no entry or category {only!r}")
    return es


def cmd_build(m: Manifest, out: Path, offline: bool, only: str | None, include_optional: bool, bless_mode: bool = False) -> list[Entry]:
    es = select(m, only, include_optional)
    if not offline:
        fetch(m, es, out, bless_mode)
    built = []
    for e in es:
        if build_entry(m, e, out):
            built.append(e)
            print(f"built {e['id']}")
    return built


# ---- check ------------------------------------------------------------------------------------


def check_outputs(m: Manifest, root: Path, entries: list[Entry] | None = None) -> list[str]:
    """Problems found (empty when every output exists and matches its sha256)."""
    bad = []
    for e in entries if entries is not None else m["entry"]:
        for o in e["outputs"]:
            p = root / o["path"]
            if not o["sha256"]:
                if not e.get("optional", False):
                    bad.append(f"{o['path']}: no sha256 recorded")
                continue
            if not p.is_file():
                if not e.get("optional", False):
                    bad.append(f"{o['path']}: missing")
                continue
            got = sha256_file(p)
            if got != o["sha256"]:
                bad.append(f"{o['path']}: sha256 {got} != manifest {o['sha256']}")
    return bad


def cmd_check(m: Manifest, out: Path) -> int:
    bad = check_outputs(m, out)
    for b in bad:
        print(f"MISMATCH {b}", file=sys.stderr)
    n = sum(len(e["outputs"]) for e in m["entry"])
    print(f"checked {n} outputs: {'ok' if not bad else f'{len(bad)} problem(s)'}")
    return 1 if bad else 0


def cmd_selftest(m: Manifest, out: Path) -> int:
    """Copy one small existing output, flip a byte, and require that --check fails naming it."""
    pick = next((e for e in m["entry"] if e["outputs"] and (out / e["outputs"][0]["path"]).is_file() and e["outputs"][0]["sha256"]), None)
    if pick is None:
        raise CorpusError("selftest needs a built entry (run build first)")
    work = out / ".selftest"
    if work.exists():
        shutil.rmtree(work)
    try:
        rel = pick["outputs"][0]["path"]
        dst = work / rel
        dst.parent.mkdir(parents=True)
        shutil.copyfile(out / rel, dst)
        copy = {"entry": [{**pick, "outputs": [pick["outputs"][0]]}]}
        if check_outputs(copy, work):
            print("selftest: the clean copy fails the check", file=sys.stderr)
            return 1
        data = bytearray(dst.read_bytes())
        data[len(data) // 2] ^= 0x01
        dst.write_bytes(bytes(data))
        bad = check_outputs(copy, work)
        if len(bad) == 1 and bad[0].startswith(rel):
            print(f"selftest ok: corrupted {rel} is reported ({bad[0].split(':')[0]})")
            return 0
        print(f"selftest: corruption not reported as expected: {bad}", file=sys.stderr)
        return 1
    finally:
        shutil.rmtree(work, ignore_errors=True)


# ---- expected (what sm-csgio does today) ------------------------------------------------------

KIND = re.compile(r"^(unsupported|parse error|invalid)(?: \(line \d+\)| at \d+:\d+)?: (.*)$", re.S)


def slug(text: str, words: int = 4) -> str:
    """First few words of ``text`` up to a parenthesis, quote or semicolon, as ``a-b-c``."""
    head = re.split(r"[(\";]", text, maxsplit=1)[0]
    ws = [w for w in re.findall(r"[a-z0-9_]+", head.lower()) if w not in ("a", "an", "the")]
    return "-".join(ws[:words])


def refusal_code(message: str) -> str:
    """A stable code from an import error message, without line numbers, ids or free text:
    ``unsupported:hex_lattice``, ``unsupported:material-list``, ``parse``, ..."""
    m = KIND.match(message)
    if not m:
        return f"error:{slug(message, 5)}".rstrip(":")  # e.g. "error:flat-model-has-more-than"
    if m.group(1) == "parse error":
        return "parse"
    kind = m.group(1)
    what = m.group(2)
    with_id = re.match(r"^\w+ \d+: (.*)$", what, re.S)  # "cell 20733: a material list ..."
    named = re.match(r"^(\w+): ", what)  # "hex_lattice: <hex_lattice> is not supported"
    detail = slug(with_id.group(1)) if with_id else (named.group(1) if named else slug(what))
    return f"{kind}:{detail}" if detail else kind


def smcsg_expected(smcsg: Path, geometry: Path) -> str:
    r = subprocess.run([str(smcsg), "info", str(geometry), "--json"], capture_output=True, text=True, check=False)
    if r.returncode == 0:
        return "accepted"
    try:
        msg = json.loads(r.stdout)["error"]
    except (ValueError, KeyError):
        raise CorpusError(f"smcsg gave no JSON error for {geometry} (rc {r.returncode}): {r.stderr[:300]}") from None
    return refusal_code(str(msg))


def deck_entries(m: Manifest) -> list[Entry]:
    return [e for e in m["entry"] if "deck" in e]


def cmd_expected(m: Manifest, out: Path, smcsg: Path, write: bool, ids: set[str] | None = None) -> int:
    diffs = 0
    for e in deck_entries(m):
        if ids is not None and e["id"] not in ids:
            continue
        deck = out / e["deck"]
        if not deck.is_file():
            if e.get("optional", False):
                continue
            print(f"{e['id']}: deck {deck} missing (build first)", file=sys.stderr)
            return 2
        got = smcsg_expected(smcsg, deck)
        mark = "ok" if got == e.get("expected") else "DIFF"
        print(f"{e['id']}: {got} (manifest: {e.get('expected')}) {mark}")
        if got != e.get("expected"):
            diffs += 1
            if write:
                e["expected"] = got
    if write:
        MANIFEST.write_text(dump_manifest(m))
        return 0
    return 1 if diffs else 0


# ---- bless ------------------------------------------------------------------------------------


def cmd_bless(m: Manifest, out: Path, smcsg: Path | None, only: str | None, include_optional: bool) -> int:
    """Rebuild everything from a wiped directory twice and record the sha256 only when both agree."""
    es = select(m, only, include_optional)
    fetch(m, es, out, allow_unpinned=True)
    for s in m.get("support", []):
        p = cache_path(out, s["repo"], s["commit"], s["path"])
        if p.is_file():
            s["sha256"] = sha256_file(p)
    for e in es:
        if e["recipe"] == "copy":  # pinned downloads: the sha256 of the cached file
            repo, commit = split_source(e)
            for o in e["outputs"]:
                o["sha256"] = sha256_file(cache_path(out, repo, commit, o["path"].split("/", 1)[1]))
    first: dict[str, str] = {}
    for rnd in (1, 2):
        for e in es:
            if not build_entry(m, e, out):
                continue
            for o in e["outputs"]:
                p = out / o["path"]
                if not p.is_file():
                    raise CorpusError(f"{e['id']}: expected output {o['path']} was not produced")
                h = sha256_file(p)
                if rnd == 1:
                    first[o["path"]] = h
                elif first[o["path"]] != h:
                    raise CorpusError(f"{o['path']}: two builds differ ({first[o['path']]} vs {h}); not reproducible")
                o["sha256"] = h
    MANIFEST.write_text(dump_manifest(m))
    if smcsg is not None:
        m2 = load_manifest()
        return cmd_expected(m2, out, smcsg, write=True, ids={e["id"] for e in es})
    return 0


# ---- main -------------------------------------------------------------------------------------


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description="Build the smart-mesh reference corpus.")
    ap.add_argument("command", nargs="?", default="build", choices=["build", "fetch", "expected", "bless", "selftest", "list"])
    ap.add_argument("--out", type=Path, default=None, help="output directory (default ${SM_REFCORPUS_DIR:-~/.cache/smart-mesh/refcorpus})")
    ap.add_argument("--check", action="store_true", help="recompute sha256 of every output and compare with the manifest")
    ap.add_argument("--offline", action="store_true", help="build from the cache, no network")
    ap.add_argument("--only", default=None, help="one entry id or category")
    ap.add_argument("--include-optional", action="store_true")
    ap.add_argument("--smcsg", type=Path, default=REPO / "target" / "debug" / "smcsg")
    ap.add_argument("--write", action="store_true", help="expected: write the recorded values into the manifest")
    a = ap.parse_args(argv)
    try:
        out = check_out_dir(a.out or default_out())
        m = load_manifest()
        if a.check:
            rc = cmd_check(m, out)
            if rc == 0 and a.smcsg.is_file() and "--smcsg" in argv:
                rc = cmd_expected(m, out, a.smcsg, write=False)
            return rc
        if a.command == "list":
            for e in m["entry"]:
                print(f"{e['id']:32} {e['category']:11} {e.get('expected', '')}")
            return 0
        out.mkdir(parents=True, exist_ok=True)
        if a.command == "fetch":
            fetch(m, select(m, a.only, a.include_optional), out, allow_unpinned=False)
        elif a.command == "build":
            cmd_build(m, out, a.offline, a.only, a.include_optional)
        elif a.command == "expected":
            return cmd_expected(m, out, a.smcsg, a.write)
        elif a.command == "bless":
            return cmd_bless(m, out, a.smcsg if a.smcsg.is_file() else None, a.only, a.include_optional)
        elif a.command == "selftest":
            return cmd_selftest(m, out)
        return 0
    except CorpusError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

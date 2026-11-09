#!/usr/bin/env python3
"""Helper of tools/census/census.sh (run as `python3 -I census.py <command> ...`).

Every command prints `group|key|os-arch-or-any|value` rows on stdout; `finish` sorts and writes
the file, `compare` diffs two files. Values come from running code (the CLI, tests, the probe) or
from sha256 of committed files, never from test source.
"""

import glob
import hashlib
import json
import os
import platform
import re
import subprocess
import sys
import tempfile

PIN = ["--author", "it", "--fixed-time", "2026-10-24T00:00:00Z"]
LINUX = "linux-x86_64"


def osarch():
    """`macos-aarch64`, `linux-x86_64`: the os-arch tag of this machine."""
    system = {"darwin": "macos"}.get(
        platform.system().lower(), platform.system().lower()
    )
    machine = {"arm64": "aarch64", "amd64": "x86_64"}.get(
        platform.machine().lower(), platform.machine().lower()
    )
    return f"{system}-{machine}"


def row(group, key, plat, value):
    """One output row."""
    value = str(value).replace("\n", " ").replace("|", "/")
    print(f"{group}|{key}|{plat}|{value}")


def sha256_file(path):
    """Hex sha256 of a file."""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def read_rows(path):
    """`script|key|value` lines of a hashes.sh output as a dict."""
    out = {}
    with open(path, encoding="utf-8") as f:
        for ln in f:
            ln = ln.rstrip("\n")
            if ln:
                s, k, v = ln.split("|", 2)
                out[(s, k)] = v
    return out


def cmd_platform(expected, hashes):
    """The 62 data lines of expected.txt recomputed, plus the sha256 of expected.txt itself."""
    got = read_rows(hashes)
    n = 0
    with open(expected, encoding="utf-8") as f:
        for ln in f:
            ln = ln.rstrip("\n")
            if not ln or ln.startswith("#"):
                continue
            s, k, _ = ln.split("|", 2)
            row(
                "platform",
                f"{s}.{k}",
                "any",
                got.get((s, k), "missing: not produced by hashes.sh"),
            )
            n += 1
    row("platform", "expected.txt.sha256", "any", sha256_file(expected))
    print(f"platform rows: {n} + 1", file=sys.stderr)
    # SUBMAP_GRADED: topology, geometry and state hash of the fixture, as the test asserts them.
    for k in ("print.topology_hash", "print.geometry_hash", "print.cells", "state"):
        row("submap_graded", k, "any", got.get(("w3b_submap_graded", k), "missing"))
    row("w3c_pin", "pin.hexes", "any", got.get(("w3c_pin", "print.cells"), "missing"))


def run(args, cwd, check=True):
    """Run a command, capture text."""
    r = subprocess.run(args, cwd=cwd, capture_output=True, text=True, check=False)
    if check and r.returncode != 0:
        sys.exit(
            f"census: {args[0]} failed ({r.returncode}): {r.stderr.strip()[-300:] or r.stdout.strip()[-300:]}"
        )
    return r


def cmd_w3c(smesh, root):
    """Enclosed pin (733 tets, mesh.auto_tet warnings) and the other W3c fixture scripts."""
    collect = os.path.join(root, "tools", "platform", "collect.py")
    with tempfile.TemporaryDirectory(prefix="census-w3c-") as tmp:
        for name, script in (
            ("enclosed", "tools/w3c_pin_enclosed.smesh"),
            ("leftout", "tools/w3c_pin_leftout.smesh"),
            ("warped", "tools/w3c_warped.smesh"),
        ):
            scratch = os.path.join(tmp, name)
            r = run(
                [
                    "python3",
                    "-I",
                    collect,
                    smesh,
                    name,
                    os.path.join(root, script),
                    scratch,
                ],
                tmp,
            )
            for ln in r.stdout.splitlines():
                _, k, v = ln.split("|", 2)
                if k.startswith("error."):
                    sys.exit(f"census: {name}: step failed: {k} {v}")
                if k.startswith("print.") or k == "state":
                    row(
                        "w3c_pin",
                        f"{name}.{k}",
                        "any" if k != "print.mesh_volume" else osarch(),
                        v,
                    )
        cwd = os.path.join(tmp, "warn")
        os.makedirs(cwd)
        r = run(
            [
                smesh,
                "run",
                os.path.join(root, "tools/w3c_pin_enclosed.smesh"),
                "--memory",
                "--json",
                *PIN,
            ],
            cwd,
        )
        events = json.loads(r.stdout)["events"]
        diags = [
            d
            for e in events
            if e.get("op") == "mesh.auto"
            for d in e.get("delta", {}).get("diagnostics", [])
            if d.get("code") == "mesh.auto_tet"
        ]
        row("w3c_pin", "enclosed.mesh.auto_tet_warnings", "any", len(diags))
        row(
            "w3c_pin",
            "enclosed.mesh.auto_tet_entities",
            "any",
            ",".join(sorted(e for d in diags for e in d["entities"])),
        )


P0_HEAD = "let b = model.create_brick(x: 10.0, y: 10.0, z: 10.0)\n"
TAIL = "let m = mesh.run(targets: b.volume)\nlet i = mesh.info(mesh: m.mesh[0])\n"


def p0_cases():
    """The scripts of crates/sm-cli/tests/p0_rows.rs rows 1 to 6 (same ops, run through the CLI)."""
    cases = {
        "row1": "let b = model.create_brick(x: 4.0, y: 2.0, z: 1.0)\n"
        "mesh.intervals(targets: b.volume, count: 3)\n"
        'mesh.scheme(targets: b.volume, scheme: "map")\n' + TAIL,
    }
    for k, (dims, f) in enumerate(
        [
            ([10.0] * 3, 1),
            ([10.0] * 3, 5),
            ([10.0] * 3, 9),
            ([10.0] * 3, 10),
            ([4.0, 2.0, 1.0], 10),
            ([10.0] * 3, 7),
        ]
    ):
        cases[f"row2_{k}"] = (
            f"let b = model.create_brick(x: {dims[0]!r}, y: {dims[1]!r}, z: {dims[2]!r})\n"
            f"mesh.size(targets: b.volume, auto: {f})\n"
            'mesh.scheme(targets: b.volume, scheme: "map")\n' + TAIL
        )
    cases["row3"] = P0_HEAD + "mesh.default_size(size: 2.0)\n" + TAIL
    cases["row4"] = (
        P0_HEAD + "mesh.size(targets: b.volume, size: 2.5)\n"
        'mesh.scheme(targets: b.edges, scheme: "equal")\n'
        'mesh.scheme(targets: b.faces, scheme: "map")\n'
        'mesh.scheme(targets: b.volume, scheme: "map")\n' + TAIL
    )
    cases["row4_plain"] = P0_HEAD + "mesh.size(targets: b.volume, size: 2.5)\n" + TAIL
    cases["row6a"] = (
        P0_HEAD
        + "mesh.intervals(targets: b.volume, count: 3)\nmesh.size(targets: b.volume, size: 2.0)\n"
        + TAIL
    )
    cases["row6b"] = (
        P0_HEAD
        + "mesh.size(targets: b.volume, size: 2.0)\nmesh.intervals(targets: b.volume, count: 3)\n"
        + TAIL
    )
    cases["row6c"] = (
        P0_HEAD
        + "mesh.intervals(targets: b.edges[0], count: 2)\nmesh.size(targets: b.volume, size: 2.5)\n"
        'mesh.scheme(targets: b.volume, scheme: "map")\n' + TAIL
    )
    return cases


def cmd_p0(smesh):
    """Cell, vertex and volume counts (and the row 4 mesh hashes) of the P0 rows, measured."""
    with tempfile.TemporaryDirectory(prefix="census-p0-") as tmp:
        for name, text in p0_cases().items():
            path = os.path.join(tmp, f"{name}.smesh")
            with open(path, "w", encoding="utf-8") as f:
                f.write(text)
            r = run([smesh, "run", path, "--memory", "--json"], tmp)
            infos = [
                e["result"]
                for e in json.loads(r.stdout)["events"]
                if e.get("event") == "query" and e.get("name") == "mesh.info"
            ]
            if len(infos) != 1:
                sys.exit(f"census: p0 {name}: {len(infos)} mesh.info results")
            i = infos[0]
            row("p0_rows", f"{name}.cells", "any", i["cells"])
            row("p0_rows", f"{name}.vertices", "any", i["vertices"])
            row("p0_rows", f"{name}.hexes", "any", i["cell_shapes"].get("Hex", 0))
            row("p0_rows", f"{name}.volume", "any", f"{i['volume']:.9g}")
            if name.startswith("row4"):
                row("p0_rows", f"{name}.topology_hash", "any", i["topology_hash"])
                row("p0_rows", f"{name}.geometry_hash", osarch(), i["geometry_hash"])


FLAT = re.compile(
    r"(\w+): \(structure hash, bundle hash\) = \((0x[0-9a-f]+), (0x[0-9a-f]+)\)"
)


def cmd_flat(log):
    """flat_golden tuples of this machine from a `--nocapture` run."""
    seen = {}
    with open(log, encoding="utf-8") as f:
        for m in FLAT.finditer(f.read()):
            seen.setdefault(m.group(1), (m.group(2), m.group(3)))
    if not seen:
        sys.exit("census: no flat_golden tuples in the test output")
    for n, (s, b) in seen.items():
        row("flat_golden", n, osarch(), f"structure={s} bundle={b}")


def cmd_rtbench(*files):
    """Lines of `smrt-bench gen --print` as `<sha>  <name>[@os-arch]`."""
    for p in files:
        with open(p, encoding="utf-8") as f:
            for ln in f:
                if not ln.strip():
                    continue
                sha, name = ln.split()
                name, _, plat = name.partition("@")
                row("rtbench", name, plat or "any", sha)


def cmd_export(smesh, root):
    """sha256 of each format written by the two-brick hex fixture of export_wire.rs, and of the format goldens."""
    script = (
        "let a = model.create_brick(x: 2.0, y: 2.0, z: 2.0, center: [1.0, 1.0, 1.0])\n"
        "let b = model.create_brick(x: 2.0, y: 2.0, z: 2.0, center: [3.0, 1.0, 1.0])\n"
        "model.imprint(bodies: a.body | b.body)\nmodel.merge(entities: a.body | b.body)\n"
        'fem.block_add(block: 1, volumes: a.volume, name: "fuel")\n'
        'fem.block_add(block: 2, volumes: b.volume, name: "water")\n'
        "mesh.size(targets: a.volume | b.volume, size: 1.0)\n"
        'mesh.scheme(targets: a.volume | b.volume, scheme: "map")\n'
        "let m = mesh.run(targets: a.volume | b.volume)\n"
    )
    exts = ["h5m", "msh", "smrt"]
    with tempfile.TemporaryDirectory(prefix="census-export-") as tmp:
        for e in exts:
            script += f'session.export("x.{e}")\n'
        path = os.path.join(tmp, "ex.smesh")
        with open(path, "w", encoding="utf-8") as f:
            f.write(script)
        run([smesh, "run", path, "--memory", *PIN], tmp)
        for e in exts:
            plat = osarch() if e in ("h5m", "smrt") else "any"
            row(
                "export",
                f"hex_server.{e}.sha256",
                plat,
                sha256_file(os.path.join(tmp, f"x.{e}")),
            )
    for pat in ("crates/sm-meshio/tests/data/**", "tools/conformance/fixtures/ref/**"):
        for p in sorted(glob.glob(os.path.join(root, pat), recursive=True)):
            if os.path.isfile(p):
                row(
                    "export",
                    f"golden.{os.path.relpath(p, root)}",
                    "any",
                    sha256_file(p),
                )


def cmd_files(root, emit):
    """sha256 groups: conformance, csgio oracles, golden-reviewed."""
    for p in sorted(
        glob.glob(os.path.join(root, "tools/conformance/fixtures/*.smesh"))
    ):
        row("conformance", os.path.basename(p), "any", sha256_file(p))
    # w3a_poly is the polyhedral dual built by w3a_tet.smesh; it has no committed file, so the
    # emitted scripts that build it are recorded.
    for n in ("w3a_hybrid.smesh", "w3a_tet.smesh"):
        row("conformance", f"emitted.{n}", "any", sha256_file(os.path.join(emit, n)))
    for p in sorted(
        glob.glob(
            os.path.join(root, "crates/sm-csgio/tests/fixtures/openmc/*/oracle.json")
        )
    ):
        row(
            "csgio",
            f"oracle.{os.path.basename(os.path.dirname(p))}.sha256",
            "any",
            sha256_file(p),
        )
    for p in sorted(glob.glob(os.path.join(root, "crates/sm-csgio/tests/golden/*"))):
        row("csgio", f"golden.{os.path.basename(p)}.sha256", "any", sha256_file(p))
    seen = set()
    for pat in ("crates/*/golden/**", "crates/*/tests/golden/**"):
        for p in glob.glob(os.path.join(root, pat), recursive=True):
            if os.path.isfile(p):
                seen.add(os.path.relpath(p, root))
    for rel in sorted(seen):
        row("golden-reviewed", rel, "any", sha256_file(os.path.join(root, rel)))


TESTS = [
    "identity_hashes_match_expected_on_every_platform",
    "milestone_w3c_pin",
    "each_format_is_byte_identical_in_process_cli_and_mcp",
    "acc7_determinism",
    "flat_inputs_keep_their_w3a_hashes",
]


def gh_json(args):
    """JSON of a gh call, or None."""
    r = subprocess.run(["gh", *args], capture_output=True, text=True, check=False)
    if r.returncode != 0:
        return None
    return json.loads(r.stdout)


def cmd_linux(tmp, run_id):
    """Linux rows from the log of the last green CI run on main (header line first on stderr-free stdout as `#`)."""
    if not run_id:
        runs = gh_json(
            [
                "run",
                "list",
                "--branch",
                "main",
                "--workflow",
                "ci",
                "--status",
                "success",
                "--limit",
                "1",
                "--json",
                "databaseId,headSha",
            ]
        )
        if not runs:
            print("# ci-run: none (gh run list failed or found no green run)")
            row("flat_golden", "all", LINUX, "missing: no green CI run found with gh")
            row(
                "platform_linux",
                "expected.txt",
                LINUX,
                "missing: no green CI run found with gh",
            )
            return
        run_id, head = str(runs[0]["databaseId"]), runs[0]["headSha"]
    else:
        info = gh_json(["run", "view", run_id, "--json", "headSha,conclusion"]) or {}
        head = info.get("headSha", "unknown")
    print(f"# ci-run: {run_id} (success, main, headSha {head})")
    log = os.path.join(tmp, "ci.log")
    with open(log, "w", encoding="utf-8") as f:
        r = subprocess.run(
            ["gh", "run", "view", run_id, "--log"],
            stdout=f,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )
    if r.returncode != 0:
        row(
            "flat_golden",
            "all",
            LINUX,
            f"missing: gh run view --log failed: {r.stderr.strip()[:120]}",
        )
        row("platform_linux", "expected.txt", LINUX, "missing: log unavailable")
        return
    seen = {}
    status = {}
    with open(log, encoding="utf-8", errors="replace") as f:
        for ln in f:
            m = FLAT.search(ln)
            if m:
                seen.setdefault(m.group(1), (m.group(2), m.group(3)))
            m = re.search(r"\btest (\w+) \.\.\. (\w+)", ln)
            if m and m.group(1) in TESTS:
                status.setdefault(m.group(1), set()).add(m.group(2))
    for n in ("hex_block6", "nonconvex_poly", "mixed_transition", "nested_facets"):
        s = seen.get(n)
        row(
            "flat_golden",
            n,
            LINUX,
            f"structure={s[0]} bundle={s[1]}"
            if s
            else "missing: the log has no tuple for this case",
        )
    for t in TESTS:
        st = status.get(t)
        row(
            "platform_linux",
            f"test.{t}",
            LINUX,
            "ok"
            if st == {"ok"}
            else f"missing: log states {sorted(st) if st else 'no result line'}",
        )
    row(
        "platform_linux",
        "expected.txt",
        LINUX,
        "missing: the log prints only the pass line of the test, not the 62 values",
    )
    row(
        "rtbench_linux",
        "expected_sha256.txt",
        LINUX,
        "missing: the file has no Linux lines and the log does not print them",
    )


def cmd_finish(out_rows, header):
    """Sort rows and print the header and rows."""
    with open(header, encoding="utf-8") as f:
        sys.stdout.write(f.read())
    with open(out_rows, encoding="utf-8") as f:
        rows = sorted(
            {ln.rstrip("\n") for ln in f if ln.strip() and not ln.startswith("#")}
        )
    keys = [r.split("|", 3)[:3] for r in rows]
    dup = {tuple(k) for i, k in enumerate(keys) if k in keys[:i]}
    if dup:
        sys.exit(f"census: duplicate keys: {sorted(dup)[:3]}")
    sys.stdout.write("".join(r + "\n" for r in rows))


def load(path, skip_linux):
    """Rows of a census file as {key: value}."""
    d = {}
    with open(path, encoding="utf-8") as f:
        for ln in f:
            ln = ln.rstrip("\n")
            if not ln or ln.startswith("#"):
                continue
            g, k, o, v = ln.split("|", 3)
            if skip_linux and o.startswith("linux"):
                continue
            d[f"{g}|{k}|{o}"] = v
    return d


def cmd_compare(base, now, skip_linux):
    """Print differing or one-sided keys; exit 1 if any."""
    a, b = load(base, skip_linux), load(now, skip_linux)
    bad = 0
    for k in sorted(set(a) | set(b)):
        if k not in a:
            print(f"ONLY-NOW {k}: {b[k]}")
        elif k not in b:
            print(f"ONLY-BASE {k}: {a[k]}")
        elif a[k] != b[k]:
            print(f"DIFF {k}: base={a[k]} now={b[k]}")
        else:
            continue
        bad += 1
    print(
        f"census compare: {bad} differing of {len(set(a) | set(b))} keys",
        file=sys.stderr,
    )
    sys.exit(1 if bad else 0)


def main():
    """Dispatch."""
    c, a = sys.argv[1], sys.argv[2:]
    if c == "platform":
        cmd_platform(*a)
    elif c == "w3c":
        cmd_w3c(*a)
    elif c == "p0":
        cmd_p0(*a)
    elif c == "flat":
        cmd_flat(*a)
    elif c == "rtbench":
        cmd_rtbench(*a)
    elif c == "export":
        cmd_export(*a)
    elif c == "files":
        cmd_files(*a)
    elif c == "linux":
        cmd_linux(*a)
    elif c == "finish":
        cmd_finish(*a)
    elif c == "compare":
        cmd_compare(a[0], a[1], len(a) > 2 and a[2] == "skip-linux")
    else:
        sys.exit(f"census.py: unknown command {c}")


main()

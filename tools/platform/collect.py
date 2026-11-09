#!/usr/bin/env python3
"""Run one script through the smesh CLI and print `script|key|value` lines (sorted).

usage: collect.py SMESH_BIN NAME SCRIPT SCRATCH_DIR

Keys: state, layer.<name>, mem.state (memory run, `--print-hash`), print.<name> (hash-like
prints only), branch.<name> (state hash at each branch tip), node.<NNN>.id / node.<NNN>.state
(every logged node, oldest first). A missing step prints `error.<step>` instead of vanishing, so
compare.py reports it as a difference. Pinning is the same as the milestone tests use.
"""
import json
import os
import re
import shutil
import subprocess
import sys

PIN = ["--author", "it", "--fixed-time", "2026-10-24T00:00:00Z"]
# prints that are identity hashes or counts (step_bytes only matters for the curved probe);
# floats (volumes, deltas) are not recorded
KEEP = re.compile(r"^(h_|.*hash|mesh|cells|face_cells|head|tess_|step_bytes)")


def last_line(r):
    """Last non-empty line of a failed run's stderr (else stdout), or its return code."""
    text = (r.stderr or r.stdout).strip().splitlines()
    return text[-1][:300] if text else f"rc={r.returncode}"


def main():
    if len(sys.argv) != 5 or sys.argv[1] in ("-h", "--help"):
        print(__doc__)
        sys.exit(0 if len(sys.argv) > 1 and sys.argv[1] in ("-h", "--help") else 2)
    smesh, name, script, scratch = sys.argv[1:5]
    script = os.path.abspath(script)
    smesh = os.path.abspath(smesh)
    scratch = os.path.abspath(scratch)
    shutil.rmtree(scratch, ignore_errors=True)
    os.makedirs(scratch)
    proj = os.path.join(scratch, "proj")
    rows = {}

    def run(args, cwd=scratch):
        return subprocess.run([smesh, *args], cwd=cwd, capture_output=True, text=True)

    def js(step, args, cwd=scratch):
        r = run(args, cwd)
        if r.returncode != 0:
            rows[f"error.{step}"] = last_line(r)
            return None
        try:
            return json.loads(r.stdout)
        except ValueError as e:
            rows[f"error.{step}"] = f"bad json: {e}"
            return None

    r_new = run(["new", proj])
    if r_new.returncode != 0:
        rows["error.new"] = last_line(r_new)
    out = js("run", ["run", script, "-p", proj, *PIN, "--json"])
    if out is not None:
        for line in str(out.get("output", "")).splitlines():
            parts = line.split(None, 1)
            if len(parts) == 2 and KEEP.match(parts[0]):
                rows[f"print.{parts[0]}"] = parts[1].strip()
        st = out.get("state")
        rows["run.state"] = st["state"] if isinstance(st, dict) else st
    h = js("hash", ["hash", "-p", proj, "--json"])
    if h is not None:
        rows["state"] = h["state"]
        for k, v in h["layers"].items():
            rows[f"layer.{k}"] = v
    log = js("log", ["log", "-p", proj, "-n", "1000", "--json"])
    if log is not None:
        for b, tip in log["branches"].items():
            hb = js(f"hash.{b}", ["hash", "--at", b, "-p", proj, "--json"])
            if hb is not None:
                rows[f"branch.{b}"] = hb["state"]
            rows[f"branch.{b}.node"] = tip
        nodes = [n["node"] for n in log["nodes"]][::-1]
        for i, n in enumerate(nodes):
            rows[f"node.{i:03}.id"] = n
            hn = js(f"hash.node{i}", ["hash", "--at", n, "-p", proj, "--json"])
            if hn is not None:
                rows[f"node.{i:03}.state"] = hn["state"]
    # memory run, own cwd (the w3b script writes files relative to it)
    mem = os.path.join(scratch, "mem")
    os.makedirs(mem)
    r = run(["run", script, "--memory", "--print-hash", *PIN], cwd=mem)
    if r.returncode == 0 and r.stdout.strip():
        rows["mem.state"] = r.stdout.strip().splitlines()[-1]
    else:
        rows["error.mem"] = last_line(r)
    for k in sorted(rows):
        print(f"{name}|{k}|{rows[k]}")


if __name__ == "__main__":
    main()

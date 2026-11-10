"""Structural comparison of our `source: model` .h5m with MOAB's `facets_to_h5m` output.

usage: dagmc_vs_moab.py OURS.h5m WORKDIR

1. `tools/conformance/check_h5m.py` runs on OURS (h5py decode, closed volumes, positive signed
   volumes, the subset of the MOAB reference file layout).
2. The triangles of OURS become a `smfacets v1` file (front/back = the volumes the normal points
   out of and into), which the benchmark's MOAB image turns into an h5m with `facets_to_h5m`
   (docker image `smrt-bench-dagmc-moab`, run with a bash entry point override).
3. DAGMC's `check_watertight` runs on OURS (exit 0, no 'not watertight').
4. Both files must agree on: triangle count, volume count, the triangles themselves (oriented,
   as coordinate triples), the signed volume of every region, and the surfaces under MOAB's
   grouping rule (triangles with the same sense pair, connected through shared vertices; our
   surfaces are B-rep faces, so ours are merged by that rule before the comparison).

Exit 0 on agreement. Needs h5py, numpy and docker.
"""
import importlib.util
import json
import os
import pathlib
import struct
import subprocess
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[3]
CONF = ROOT / "tools" / "conformance"
IMAGE = os.environ.get("SM_MOAB_IMAGE", "smrt-bench-dagmc-moab")


def load_check():
    spec = importlib.util.spec_from_file_location("check_h5m", CONF / "check_h5m.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def decode_file(chk, path, label, problems):
    """Handle tables as plain Python. Unlike `check_h5m.decode` this also reads MOAB's dense
    set tags (`sets/tags/<NAME>`, one value per set) next to the sparse ones (`tags/<NAME>`)."""
    import h5py

    f = h5py.File(path, "r")
    t = f["tstt"]
    nodes = t["nodes/coordinates"]
    coords = nodes[:]
    tri_start, tri_conn = None, None
    for name, grp in t["elements"].items():
        if name == "Tri3":
            c = grp["connectivity"]
            tri_start, tri_conn = int(c.attrs["start_id"]), c[:].astype(np.int64)
    lst = t["sets/list"]
    set0 = int(lst.attrs["start_id"])
    rows = lst[:]
    tables = {n: (t["sets/" + n][:] if n in t["sets"] else np.zeros(0, dtype=np.uint64)) for n in ("contents", "children", "parents")}

    def span(k, col):
        lo = 0 if k == 0 else rows[k - 1, col] + 1
        return int(lo), int(rows[k, col] + 1)

    def get(k, name, col):
        a, b = span(k, col)
        return tables[name][a:b].astype(np.int64)

    def contents(k):
        raw = get(k, "contents", 0)
        if rows[k, 3] & 8:
            out = []
            for st, c in zip(raw[0::2], raw[1::2]):
                out.extend(range(int(st), int(st + c)))
            return np.array(out, dtype=np.int64)
        return raw

    def tag(name):
        out = {}
        if "tags" in t and name in t["tags"] and "id_list" in t["tags"][name]:
            ids = t["tags"][name]["id_list"][:]
            out.update(zip((int(i) for i in ids), t["tags"][name]["values"][:]))
        if name in t["sets/tags"]:
            vals = t["sets/tags"][name][:]
            out.update({set0 + k: v for k, v in enumerate(vals)})
        return out

    dims = {h: int(v) for h, v in tag("GEOM_DIMENSION").items()}
    gid = {h: int(v) for h, v in tag("GLOBAL_ID").items()}
    sense = {h: (int(v[0]), int(v[1])) for h, v in tag("GEOM_SENSE_2").items()}
    surfaces = sorted(h for h, d in dims.items() if d == 2)
    volumes = sorted(h for h, d in dims.items() if d == 3)
    sset = {}
    for h in surfaces:
        c = contents(h - set0)
        sset[h] = c[(c >= tri_start) & (c < tri_start + len(tri_conn))]
        if h not in sense:
            problems.append(f"{label}: surface {h} has no GEOM_SENSE_2")
    gids = np.zeros(len(rows), dtype=np.int64)
    for h, g in gid.items():
        gids[h - set0] = g
    return dict(coords=coords, tris=(tri_start, tri_conn), surfaces=surfaces, volumes=volumes, sset=sset, sense=sense, set0=set0, gids=gids)


def triangles(d):
    """Oriented triangles as (sense pair of volume handles, 3 coordinate tuples) per triangle."""
    s, conn = d["tris"]
    out = []
    for h in d["surfaces"]:
        fwd, bwd = d["sense"][h]
        for t in d["sset"][h]:
            tri = [int(x) for x in conn[int(t) - s]]
            out.append((fwd, bwd, tri))
    return out


def canon_tri(d, tri):
    pts = [tuple(float(x) for x in d["coords"][i - 1]) for i in tri]
    k = pts.index(min(pts))
    return tuple(pts[k:] + pts[:k])


def surfaces_moab_rule(items):
    """items: (fwd, bwd, [3 node ids]); group by pair and vertex connectivity; list of (pair, n)."""
    parent = {}

    def find(x):
        while parent.setdefault(x, x) != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for _, _, tri in items:
        for v in tri[1:]:
            parent[find(v)] = find(tri[0])
    groups = {}
    for fwd, bwd, tri in items:
        groups.setdefault(((fwd, bwd), find(tri[0])), []).append(tri)
    return sorted((k[0], len(v)) for k, v in groups.items())


def main(argv):
    ours_path, work = argv[0], pathlib.Path(argv[1])
    chk = load_check()
    problems, facts = [], {}

    # 1. our file under the existing Tier A check
    out_json = work / "tier_a.json"
    rc = chk.main([str(out_json), ours_path, str(CONF / "fixtures" / "ref" / "dagmc_two_volume.h5m")])
    if rc != 0:
        problems.append(f"check_h5m.py failed on our file: {json.load(open(out_json))['problems']}")

    ours = decode_file(chk, ours_path, "ours", problems)
    vol_index = {h: i for i, h in enumerate(ours["volumes"])}
    tris = triangles(ours)
    facts["triangles"] = len(tris)
    facts["volumes"] = len(ours["volumes"])
    facts["our_surfaces"] = len(ours["surfaces"])

    # 2. smfacets v1 from our triangles
    verts = np.asarray(ours["coords"], dtype="<f8")
    with open(work / "facets.bin", "wb") as f:
        f.write(struct.pack("<II", len(verts), len(tris)))
        f.write(verts.tobytes())
        f.write(np.array([[i - 1 for i in t] for _, _, t in tris], dtype="<u4").tobytes())
        f.write(np.array([[vol_index.get(a, -1), vol_index.get(b, -1)] for a, b, _ in tris], dtype="<i4").tobytes())
    uid = ["--user", f"{os.getuid()}:{os.getgid()}"]
    # DAGMC's own watertightness check on OUR file.
    r = subprocess.run(
        ["docker", "run", "--rm", *uid, "-v", f"{work}:/work", "--entrypoint", "check_watertight", IMAGE, "/work/ours.h5m"],
        capture_output=True, text=True,
    )
    text = (r.stdout + r.stderr).strip()
    facts["our_check_watertight"] = {"exit": r.returncode, "tail": text.splitlines()[-6:]}
    if r.returncode != 0 or "not watertight" in text.lower():
        problems.append(f"check_watertight on our file: exit {r.returncode}: {text[-400:]}")
    cmd = [
        "docker", "run", "--rm", *uid, "-v", f"{work}:/work",
        "--entrypoint", "/bench/facets_to_h5m", IMAGE,
        "/work/facets.bin", "/work/moab.h5m", "--report", "/work/report.json",
    ]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        problems.append(f"facets_to_h5m exited {r.returncode}: {r.stdout[-400:]} {r.stderr[-400:]}")
        print("\n".join(problems), file=sys.stderr)
        return 1
    facts["moab_report"] = json.load(open(work / "report.json"))

    # 3. compare
    moab = decode_file(chk, work / "moab.h5m", "moab", problems)
    mt = triangles(moab)
    if len(mt) != len(tris):
        problems.append(f"triangle counts: ours {len(tris)}, MOAB {len(mt)}")
    if len(moab["volumes"]) != len(ours["volumes"]):
        problems.append(f"volume counts: ours {len(ours['volumes'])}, MOAB {len(moab['volumes'])}")

    def region_of(d, h):
        # MOAB: GLOBAL_ID = region + 1; ours: index in ascending handle order.
        return int(d["gids"][h - d["set0"]]) - 1 if d is moab else vol_index[h]

    def keyed(d, items):
        return sorted((canon_tri(d, t), region_of(d, a) if a else -1, region_of(d, b) if b else -1) for a, b, t in items)

    if keyed(ours, tris) != keyed(moab, mt):
        problems.append("the oriented triangles, or their sense pairs, differ")

    def moab_rule(d, items):
        return surfaces_moab_rule([(region_of(d, a) if a else -1, region_of(d, b) if b else -1, t) for a, b, t in items])

    ours_surf, moab_surf = moab_rule(ours, tris), moab_rule(moab, mt)
    facts["surfaces_by_moab_rule"] = {"ours": len(ours_surf), "moab": len(moab_surf)}
    if ours_surf != moab_surf:
        problems.append(f"surfaces under MOAB's rule differ: ours {ours_surf} vs MOAB {moab_surf}")
    if len(moab["surfaces"]) != len(moab_surf):
        problems.append("MOAB's own surface count disagrees with the grouping rule")

    vo = chk.volume_report(ours, "ours", problems)
    vm = chk.volume_report(moab, "moab", problems)
    a = sorted(vo[h] for h in ours["volumes"])
    b = sorted(vm[h] for h in moab["volumes"])
    facts["signed_volumes"] = {"ours": a, "moab": b}
    if len(a) == len(b) and any(abs(x - y) > 1e-12 * max(1.0, abs(x)) for x, y in zip(a, b)):
        problems.append(f"signed volumes differ: {a} vs {b}")
    if not facts["moab_report"].get("watertight"):
        problems.append(f"DAGMC check_watertight did not pass on MOAB's file: {facts['moab_report']}")

    print(json.dumps({"status": "fail" if problems else "pass", "problems": problems, "facts": facts}, indent=1, default=str))
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

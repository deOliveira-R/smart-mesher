"""Tier A check of a DAGMC .h5m we wrote, with h5py (an HDF5 library independent of our writer).

usage: check_h5m.py OUT.json FILE.h5m REFERENCE.h5m [EXPECT.json] [--twin OTHER.h5m]

REFERENCE.h5m is tools/conformance/fixtures/ref/dagmc_two_volume.h5m, written by MOAB. The check
is "the subset H5M_FORMAT.md promises", not equality with the reference (we write no curves, no
OBB, no history):

1. h5py opens the file and reads every dataset (superblock, object header checksums, layout).
2. Every object we write exists in the reference with the same kind, the same dtype and the same
   rank; every attribute we write that the reference also has carries the same dtype, and the
   same value except `start_id` and `max_id` (counts differ).
3. The set, tag and handle tables decode (cumulative inclusive end columns, flag bits, contents,
   children, parents, sense pairs). The same invariants run over the reference and over our
   file: handles contiguous, volume children = surface parents, GEOM_SENSE_2 volumes = surface
   parents, NAME and CATEGORY NUL padded 32 byte strings, every volume closed (each edge used
   twice in opposite directions by the triangles oriented out of it) with positive volume by the
   divergence theorem. The reference passing is what fixes our sign convention: forward volume =
   the one the right-hand normal points OUT of.
4. EXPECT.json (from `cargo run -p sm-meshio --example export_conformance`): counts, group names,
   per-volume analytic boxes by offset points and divergence volume, graveyard boxes.
5. --twin: a second run must be byte-identical and h5py must see identical metadata.
"""
import json
import sys

import h5py
import numpy as np

VALUE_FREE = {"start_id", "max_id"}
CATEGORY_DIM = {"Vertex": 0, "Curve": 1, "Surface": 2, "Volume": 3}


def strings(ds):
    raw = np.asarray(ds[:]).tobytes()
    out = []
    for i in range(0, len(raw), 32):
        chunk = raw[i : i + 32]
        if 0 not in chunk:
            raise ValueError("opaque tag value without a NUL terminator")
        out.append(chunk.split(b"\0", 1)[0].decode())
    return out


def kind(o):
    return "group" if isinstance(o, h5py.Group) else "dataset" if isinstance(o, h5py.Dataset) else "type"


def objects(f):
    out = {}

    def visit(name, o):
        out[name] = o

    f.visititems(visit)
    return out


def read_everything(f):
    n = 0

    def visit(name, o):
        nonlocal n
        if isinstance(o, h5py.Dataset):
            o[()]
            n += 1
        for k in o.attrs:
            o.attrs[k]

    f.visititems(visit)
    return n


def compare_subset(ours, ref, problems, facts):
    a, b = objects(ours), objects(ref)
    facts["objects"] = len(a)
    facts["reference_objects_not_written"] = sorted(set(b) - set(a))
    for name, o in sorted(a.items()):
        r = b.get(name)
        if r is None:
            problems.append(f"{name}: not in the reference layout")
            continue
        if kind(o) != kind(r):
            problems.append(f"{name}: {kind(o)} but the reference has a {kind(r)}")
            continue
        if isinstance(o, h5py.Dataset):
            if o.dtype != r.dtype:
                problems.append(f"{name}: dtype {o.dtype} vs reference {r.dtype}")
            if len(o.shape) != len(r.shape) or (len(o.shape) == 2 and o.shape[1] != r.shape[1]):
                problems.append(f"{name}: shape {o.shape} vs reference {r.shape}")
        elif isinstance(o, h5py.Datatype):
            if o.dtype != r.dtype:
                problems.append(f"{name}: committed type {o.dtype} vs reference {r.dtype}")
        for k, v in o.attrs.items():
            if k not in r.attrs:
                problems.append(f"{name}@{k}: not in the reference")
                continue
            w = r.attrs[k]
            if np.asarray(v).dtype != np.asarray(w).dtype:
                problems.append(f"{name}@{k}: dtype {np.asarray(v).dtype} vs reference {np.asarray(w).dtype}")
            elif k not in VALUE_FREE and not np.array_equal(v, w):
                problems.append(f"{name}@{k}: value {v} vs reference {w}")


def decode(f, label, problems):
    """Handle tables of a file as plain Python."""
    t = f["tstt"]
    max_id = int(t.attrs["max_id"])
    ranges = []
    nodes = t["nodes/coordinates"]
    node0 = int(nodes.attrs["start_id"])
    coords = nodes[:]
    ranges.append((node0, node0 + len(coords) - 1, "nodes"))
    tris = None
    for name, grp in t["elements"].items():
        c = grp["connectivity"]
        s = int(c.attrs["start_id"])
        ranges.append((s, s + c.shape[0] - 1, name))
        if name == "Tri3":
            tris = (s, c[:].astype(np.int64))
    lst = t["sets/list"]
    set0 = int(lst.attrs["start_id"])
    rows = lst[:]
    ranges.append((set0, set0 + len(rows) - 1, "sets"))
    ranges.sort()
    for (a0, a1, an), (b0, b1, bn) in zip(ranges, ranges[1:]):
        if b0 != a1 + 1:
            problems.append(f"{label}: handles of {an} end at {a1} but {bn} starts at {b0}")
    if ranges[-1][1] != max_id:
        problems.append(f"{label}: max_id {max_id} but the last handle is {ranges[-1][1]}")
    if node0 != 1:
        problems.append(f"{label}: nodes start at {node0}, not 1")
    tables = {n: (t["sets/" + n][:] if n in t["sets"] else np.zeros(0, dtype=np.uint64)) for n in ("contents", "children", "parents")}
    for col, n in enumerate(("contents", "children", "parents")):
        ends = rows[:, col]
        if len(ends) and (ends.min() < -1 or np.any(np.diff(ends) < 0)):
            problems.append(f"{label}: set table column {n} is not a cumulative end index")
        if len(ends) and ends[-1] + 1 != len(tables[n]):
            problems.append(f"{label}: {n} table has {len(tables[n])} entries but the last end says {ends[-1] + 1}")

    def span(k, col):
        lo = 0 if k == 0 else rows[k - 1, col] + 1
        return int(lo), int(rows[k, col] + 1)

    def get(k, name, col):
        a, b = span(k, col)
        return tables[name][a:b].astype(np.int64)

    def contents(k):
        raw = get(k, "contents", 0)
        if rows[k, 3] & 8:  # range encoded (start, count) pairs
            out = []
            for s, c in zip(raw[0::2], raw[1::2]):
                out.extend(range(int(s), int(s + c)))
            return np.array(out, dtype=np.int64)
        return raw

    tags = t["tags"]

    def sparse(name):
        if name not in tags or "id_list" not in tags[name]:
            return {}
        ids = tags[name]["id_list"][:]
        vals = tags[name]["values"][:]
        return dict(zip((int(i) for i in ids), vals))

    dims = {h: int(v) for h, v in sparse("GEOM_DIMENSION").items()}
    cats = {}
    if "CATEGORY" in tags:
        cats = dict(zip((int(i) for i in tags["CATEGORY"]["id_list"][:]), strings(tags["CATEGORY"]["values"])))
    names = {}
    if "NAME" in tags:
        names = dict(zip((int(i) for i in tags["NAME"]["id_list"][:]), strings(tags["NAME"]["values"])))
    sense = {h: (int(v[0]), int(v[1])) for h, v in sparse("GEOM_SENSE_2").items()}
    for h, c in cats.items():
        if c in CATEGORY_DIM and dims.get(h) != CATEGORY_DIM[c]:
            problems.append(f"{label}: set {h} has CATEGORY {c} but GEOM_DIMENSION {dims.get(h)}")
    surfaces = sorted(h for h, d in dims.items() if d == 2)
    volumes = sorted(h for h, d in dims.items() if d == 3)
    sset = {}
    for h in surfaces:
        k = h - set0
        c = contents(k)
        c = c[(c >= tris[0]) & (c < tris[0] + len(tris[1]))] if tris is not None else c
        sset[h] = c
        par = sorted(int(x) for x in get(k, "parents", 2))
        want = sorted(v for v in sense.get(h, (0, 0)) if v)
        if par != want:
            problems.append(f"{label}: surface {h} parents {par} differ from its GEOM_SENSE_2 volumes {want}")
        if h not in sense:
            problems.append(f"{label}: surface {h} has no GEOM_SENSE_2")
    for v in volumes:
        ch = sorted(int(x) for x in get(v - set0, "children", 1))
        want = sorted(h for h in surfaces if v in sense.get(h, ()))
        if ch != want:
            problems.append(f"{label}: volume {v} children {ch} differ from the surfaces that name it {want}")
    for h, s in names.items():
        for x in contents(h - set0):
            if int(x) not in volumes:
                problems.append(f"{label}: group {s!r} holds {int(x)}, which is not a volume")
    return dict(coords=coords, tris=tris, surfaces=surfaces, volumes=volumes, sset=sset, sense=sense, names=names, set0=set0, contents=contents)


def tri_xyz(d, h, i):
    s, conn = d["tris"]
    return d["coords"][conn[i] - 1]


def volume_report(d, label, problems):
    """Signed volume and closure of every volume by the senses and the triangles."""
    s, conn = d["tris"]
    vols = {}
    for v in d["volumes"]:
        total = 0.0
        uses = {}
        for h in d["surfaces"]:
            fwd, bwd = d["sense"][h]
            if v not in (fwd, bwd):
                continue
            rev = v == bwd and v != fwd
            for t in d["sset"][h]:
                tri = [int(x) for x in conn[int(t) - s]]
                if rev:
                    tri = [tri[0], tri[2], tri[1]]
                a, b, c = (d["coords"][i - 1] for i in tri)
                total += float(np.dot(a, np.cross(b, c))) / 6.0
                for k in range(3):
                    p, q = tri[k], tri[(k + 1) % 3]
                    e = uses.setdefault((min(p, q), max(p, q)), [0, 0])
                    e[0 if p < q else 1] += 1
        bad = [e for e, u in uses.items() if u != [1, 1]]
        if bad:
            problems.append(f"{label}: volume {v} is not closed at {len(bad)} edges (first {bad[0]})")
        if not total > 0:
            problems.append(f"{label}: volume {v} has signed volume {total} (senses point the wrong way)")
        vols[v] = total
    return vols


def check_expect(d, vols, expect, problems, facts):
    s, conn = d["tris"]
    for key, got in (("triangles", len(conn)), ("surfaces", len(d["surfaces"])), ("volumes", len(d["volumes"]))):
        if key in expect and expect[key] != got:
            problems.append(f"{key}: {got}, expected {expect[key]}")
    if "group_names" in expect:
        got = sorted(d["names"].values())
        if got != sorted(expect["group_names"]):
            problems.append(f"group names {got}, expected {sorted(expect['group_names'])}")
    gids = {}
    gid = f_global_ids(d)
    boxes = {int(k): v for k, v in expect.get("boxes", {}).items()}
    for v_handle in d["volumes"]:
        g = gid[v_handle - d["set0"]]
        gids[g] = v_handle
    for g, box in boxes.items():
        v = gids.get(g)
        if v is None:
            problems.append(f"volume with GLOBAL_ID {g} missing")
            continue
        lo, hi = np.array(box["lo"]), np.array(box["hi"])
        want = float(np.prod(hi - lo))
        if abs(vols[v] - want) > 1e-12 * want:
            problems.append(f"volume {g}: {vols[v]} vs analytic {want}")
        inside = lambda p, lo=lo, hi=hi: bool(np.all(p >= lo) and np.all(p <= hi))
        for h in d["surfaces"]:
            fwd, bwd = d["sense"][h]
            if v not in (fwd, bwd):
                continue
            for t in d["sset"][h]:
                a, b, c = (d["coords"][i - 1] for i in conn[int(t) - s])
                n = np.cross(b - a, c - a)
                n = n / np.linalg.norm(n)
                p = (a + b + c) / 3 + 1e-6 * n
                if (v == fwd) == inside(p):
                    problems.append(f"volume {g}: a normal of surface {h} points the wrong way")
                    break
    gy = expect.get("graveyard")
    if gy:
        v = gids.get(gy["volume"])
        pts = set()
        for h in d["surfaces"]:
            if v in d["sense"][h]:
                for t in d["sset"][h]:
                    for i in conn[int(t) - s]:
                        pts.add(tuple(d["coords"][i - 1]))
        pts = np.array(sorted(pts))
        for k, name in ((0, "lo"), (1, "hi")):
            got = pts.min(axis=0) if k == 0 else pts.max(axis=0)
            if np.max(np.abs(got - np.array(gy["outer"][name]))) > 1e-12:
                problems.append(f"graveyard outer {name} {got}")
        il, ih = np.array(gy["inner"]["lo"]), np.array(gy["inner"]["hi"])
        inner = [p for p in pts if np.all(p >= il - 1e-12) and np.all(p <= ih + 1e-12)]
        if len(inner) != 8:
            problems.append(f"graveyard inner box has {len(inner)} corners, expected 8")
        facts["graveyard_vertices"] = len(pts)


def f_global_ids(d):
    return d["gids"]


def metadata(f):
    out = {}

    def visit(name, o):
        out[name] = (kind(o), str(getattr(o, "dtype", "")), tuple(getattr(o, "shape", ())), {k: np.asarray(v).tolist() for k, v in o.attrs.items()}, np.asarray(o[()]).tobytes() if isinstance(o, h5py.Dataset) else b"")

    f.visititems(visit)
    return out


def main(argv):
    out, path, ref_path = argv[0], argv[1], argv[2]
    rest = argv[3:]
    twin = None
    if "--twin" in rest:
        twin = rest[rest.index("--twin") + 1]
        rest = [r for i, r in enumerate(rest) if r != "--twin" and (i == 0 or rest[i - 1] != "--twin")]
    expect = json.load(open(rest[0])) if rest else {}
    problems, facts = [], {}
    f = h5py.File(path, "r")
    facts["datasets_read"] = read_everything(f)
    ref = h5py.File(ref_path, "r")
    compare_subset(f, ref, problems, facts)
    for label, h5 in (("reference", ref), ("file", f)):
        d = decode(h5, label, problems)
        d["gids"] = h5["tstt/sets/tags/GLOBAL_ID"][:]
        vols = volume_report(d, label, problems)
        facts[f"{label}_volumes"] = {int(d["gids"][v - d["set0"]]): round(x, 12) for v, x in vols.items()}
        facts[f"{label}_triangles"] = int(len(d["tris"][1]))
        if label == "file":
            check_expect(d, vols, expect, problems, facts)
            bad = [n for n in d["names"].values() if not n.startswith("mat:")]
            if bad:
                problems.append(f"group names without the mat: prefix: {bad}")
    if twin:
        a, b = open(path, "rb").read(), open(twin, "rb").read()
        if a != b:
            problems.append("two runs differ in bytes")
        if metadata(f) != metadata(h5py.File(twin, "r")):
            problems.append("two runs differ in h5py metadata")
        facts["twin_identical_bytes"] = a == b
    json.dump({"status": "fail" if problems else "pass", "problems": problems, "facts": facts}, open(out, "w"), indent=1, default=str)
    if problems:
        print("\n".join(problems), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

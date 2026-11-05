"""Tier B check of a DAGMC .h5m we wrote: MOAB and DAGMC read it. Runs inside the image of
tools/conformance/dagmc/Dockerfile (conda-forge moab, dagmc; /opt/probe is built from probe.cpp).

usage: check_dagmc.py FILE.h5m EXPECT.json [OUT.json]

1. `mbconvert FILE out.vtk` succeeds, and `mbconvert FILE rt.h5m` (MOAB re-writes what it read)
   keeps the triangle count, the surface and volume sets and the group names (h5py reads both).
2. DAGMC (libdagmc through /opt/probe): `init_OBBTree` succeeds; surface and volume counts and
   the `mat:` names of the volumes match EXPECT; `point_in_volume` for 1000 seeded points matches
   the analytic box membership; `ray_fire` from 200 seeded points inside each box volume gives the
   analytic exit distance to 1e-9 and a surface; rays in the graveyard wall hit its inner and
   outer surface at the analytic distances.

pydagmc.Model would list volumes, groups and surfaces too, but needs pymoab, which the
conda-forge MOAB package does not ship; the probe reports the same lists from libdagmc.
EXPECT.json comes from `cargo run -p sm-meshio --example export_conformance`.
"""
import json
import os
import subprocess
import sys
import tempfile

import h5py
import numpy as np

SEED = 20261104
N_POINTS = 1000
N_RAYS = 200


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def inside(box, p):
    lo, hi = np.array(box["lo"]), np.array(box["hi"])
    return bool(np.all(p >= lo) and np.all(p <= hi))


def exit_distance(box, o, d):
    lo, hi = np.array(box["lo"]), np.array(box["hi"])
    t = np.inf
    for k in range(3):
        if d[k] > 0:
            t = min(t, (hi[k] - o[k]) / d[k])
        elif d[k] < 0:
            t = min(t, (lo[k] - o[k]) / d[k])
    return float(t)


def mbconvert(path, problems, facts):
    with tempfile.TemporaryDirectory() as tmp:
        r = run(["mbconvert", path, os.path.join(tmp, "out.vtk")])
        facts["mbconvert_vtk_rc"] = r.returncode
        if r.returncode != 0 or not os.path.exists(os.path.join(tmp, "out.vtk")):
            problems.append(f"mbconvert to vtk failed: {r.stderr.strip()[-300:] or r.stdout.strip()[-300:]}")
        rt = os.path.join(tmp, "rt.h5m")
        r = run(["mbconvert", path, rt])
        facts["mbconvert_h5m_rc"] = r.returncode
        if r.returncode != 0 or not os.path.exists(rt):
            problems.append(f"mbconvert h5m round trip failed: {r.stderr.strip()[-300:] or r.stdout.strip()[-300:]}")
            return
        a, b = h5py.File(path, "r"), h5py.File(rt, "r")
        ta, tb = a["tstt/elements/Tri3/connectivity"].shape[0], b["tstt/elements/Tri3/connectivity"].shape[0]
        facts["triangles"] = [int(ta), int(tb)]
        if ta != tb:
            problems.append(f"MOAB re-wrote {tb} triangles of {ta}")
        for dim in (2, 3):
            ca = int((a["tstt/tags/GEOM_DIMENSION/values"][:] == dim).sum())
            cb = int((b["tstt/tags/GEOM_DIMENSION/values"][:] == dim).sum())
            if ca != cb:
                problems.append(f"MOAB re-wrote {cb} sets of dimension {dim} of {ca}")

        def names(f):
            if "NAME" not in f["tstt/tags"]:
                return []
            raw = np.asarray(f["tstt/tags/NAME/values"][:]).tobytes()
            return sorted(raw[i : i + 32].split(b"\0", 1)[0].decode() for i in range(0, len(raw), 32))

        if names(a) != names(b):
            problems.append(f"group names changed through MOAB: {names(a)} -> {names(b)}")


def probe(path, expect, problems, facts):
    rs = np.random.RandomState(SEED)
    boxes = {int(k): v for k, v in expect.get("boxes", {}).items()}
    gy = expect.get("graveyard")
    allbox = list(boxes.values()) + ([gy["outer"]] if gy else [])
    lo = np.min([b["lo"] for b in allbox], axis=0) - 0.2
    hi = np.max([b["hi"] for b in allbox], axis=0) + 0.2
    pts = lo + rs.rand(N_POINTS, 3) * (hi - lo)
    rays = []
    for vid, box in sorted(boxes.items()):
        blo, bhi = np.array(box["lo"]), np.array(box["hi"])
        for _ in range(N_RAYS):
            o = blo + 1e-3 + rs.rand(3) * (bhi - blo - 2e-3)
            d = rs.randn(3)
            d /= np.linalg.norm(d)
            rays.append((vid, o, d, exit_distance(box, o, d), "box"))
    if gy:
        # Mid-wall start on the x axis through the centre: +x hits the inner surface, -x the outer.
        il, ol = np.array(gy["inner"]["lo"]), np.array(gy["outer"]["lo"])
        x = (il[0] + ol[0]) / 2
        for sgn in (1.0, -1.0):
            want = (il[0] - x) if sgn > 0 else (x - ol[0])
            rays.append((gy["volume"], np.array([x, 0.5, 0.5]), np.array([sgn, 0.0, 0.0]), want, "graveyard"))
    q = [f"P {len(pts)}"] + [" ".join(repr(float(v)) for v in p) for p in pts]
    q += [f"R {len(rays)}"] + [f"{vid} " + " ".join(repr(float(v)) for v in o) + " " + " ".join(repr(float(v)) for v in d) for vid, o, d, _, _ in rays]
    r = run(["/opt/probe", path], input="\n".join(q) + "\n")
    facts["probe_rc"] = r.returncode
    if r.returncode != 0:
        problems.append(f"probe failed (rc {r.returncode}): {r.stdout.strip()[-300:]} {r.stderr.strip()[-300:]}")
        return
    vols, surfs, piv, ray = {}, [], {}, {}
    for line in r.stdout.splitlines():
        w = line.split()
        if w[0] == "COUNTS":
            facts["dagmc_surfaces"], facts["dagmc_volumes"] = int(w[1]), int(w[2])
        elif w[0] == "VOL":
            vols[int(w[1])] = w[2]
        elif w[0] == "SURF":
            surfs.append(int(w[1]))
        elif w[0] == "PIV":
            piv[(int(w[1]), int(w[2]))] = int(w[3])
        elif w[0] == "RAY":
            ray[int(w[1])] = (int(w[3]), float(w[4]))
    facts["dagmc_volume_materials"] = {str(k): v for k, v in sorted(vols.items())}
    # DAGMC adds the implicit complement as one more volume (no `mat` property).
    for key, got, extra in (("surfaces", facts.get("dagmc_surfaces"), 0), ("volumes", facts.get("dagmc_volumes"), 1)):
        if key in expect and expect[key] + extra != got:
            problems.append(f"DAGMC sees {got} {key}, expected {expect[key]} + {extra}")
    if sum(1 for v in vols.values() if v == "-") != 1:
        problems.append(f"expected exactly one volume without a material (the implicit complement): {vols}")
    mats = {v for v in vols.values() if v != "-"}
    want = {n[4:] for n in expect.get("group_names", []) if n.startswith("mat:")}
    if mats != want:
        problems.append(f"DAGMC materials {sorted(mats)} differ from the groups {sorted(want)}")
    # point_in_volume against analytic membership.
    bad = 0
    for k, p in enumerate(pts):
        for vid, box in boxes.items():
            if piv.get((k, vid)) != int(inside(box, p)):
                bad += 1
        if gy:
            want_in = inside(gy["outer"], p) and not inside(gy["inner"], p)
            if piv.get((k, gy["volume"])) != int(want_in):
                bad += 1
    facts["point_in_volume_checked"] = len(pts) * (len(boxes) + (1 if gy else 0))
    facts["point_in_volume_mismatches"] = bad
    if bad:
        problems.append(f"point_in_volume disagrees with the analytic boxes at {bad} (point, volume) pairs")
    # ray_fire distances.
    worst, misses = 0.0, 0
    for k, (vid, o, d, want, what) in enumerate(rays):
        sid, dist = ray.get(k, (0, -1.0))
        if sid == 0 or dist < 0:
            misses += 1
            continue
        worst = max(worst, abs(dist - want))
    facts["ray_fire_checked"] = len(rays)
    facts["ray_fire_max_abs_error"] = worst
    if misses:
        problems.append(f"ray_fire found no surface for {misses} rays")
    if worst > 1e-9:
        problems.append(f"ray_fire distance error {worst} exceeds 1e-9")


def main(argv):
    path, expect_path = argv[0], argv[1]
    out = argv[2] if len(argv) > 2 else None
    expect = json.load(open(expect_path))
    problems, facts = [], {}
    mbconvert(path, problems, facts)
    probe(path, expect, problems, facts)
    res = {"status": "fail" if problems else "pass", "problems": problems, "facts": facts}
    print(json.dumps(res, indent=1))
    if out:
        json.dump(res, open(out, "w"), indent=1)
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

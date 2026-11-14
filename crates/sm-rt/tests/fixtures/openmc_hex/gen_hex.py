#!/usr/bin/env python
"""Generate the OpenMC hexagonal-lattice fixtures of sm-rt (W4-RT-HEX, ruling C7).

Run with the conda env's python (OpenMC 0.16.0):

    /Users/rodrigo/opt/miniforge/envs/openmc/bin/python gen_hex.py [--out DIR] [hex_x hex_y]

For each model this writes `<model>/geometry.xml`, `<model>/materials.xml` and
`<model>/oracle.json`. The Rust test (`crates/sm-rt/tests/openmc_hex.rs`) reads only
`oracle.json`; OpenMC is not needed at test time.

The models are a 3-ring `openmc.HexLattice` of 19 pins, one *universe per element* (fuel radius
`0.20 + 0.012 i`, cell ids `100 + 10 i` and `101 + 10 i`, `i` in OpenMC's ring order: outer ring
first, from the top for orientation `y`), centre off the origin, in a slab `z` in `[-1, 1]`. The
asymmetry (distinct universe and radius per element, off-centre) makes the element index,
the orientation and the index map observable: only one pairing of OpenMC orientation and
sm-rt lattice kind reproduces all points.

OpenMC is asked as in `crates/sm-csgio/tests/fixtures/openmc/gen.py` (a one-group dummy
multi-group library, `openmc.lib.find_cell`). For every cell the oracle records OpenMC's own
lattice path (`determine_paths`): the centred axial indices `(q, r)` of the element. Points:
600 uniform in the slab over the lattice's bounding box, and 600 within 5e-3 of a random face
or vertex of the hexagonal tiling (both sides), where an off-by-one in the element lookup shows.
"""
import argparse
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import warnings
import xml.etree.ElementTree as ET

import numpy as np

warnings.filterwarnings("ignore", message=".*instance already exists with id.*")

HERE = os.path.dirname(os.path.abspath(__file__))
N_UNIFORM = 600
N_EDGE = 600
NUDGE = 5.0e-3
PITCH = 1.5
CENTER = (0.35, -0.2)
N_RINGS = 3
ZH = 1.0


def radius(i):
    return 0.20 + 0.012 * i


def build(orientation):
    """Return (geometry, materials, elements) for the orientation `x` or `y`."""
    import openmc

    openmc.reset_auto_ids()
    fuel = openmc.Material(material_id=1, name="fuel")
    water = openmc.Material(material_id=2, name="water")
    gas = openmc.Material(material_id=3, name="gas")
    for m in (fuel, water, gas):
        m.add_macroscopic(openmc.Macroscopic("dummy"))
        m.set_density("macro", 1.0)
    n_el = 3 * N_RINGS * (N_RINGS - 1) + 1
    unis = []
    for i in range(n_el):
        cyl = openmc.ZCylinder(r=radius(i))
        a = openmc.Cell(100 + 10 * i, fill=fuel, region=-cyl)
        b = openmc.Cell(101 + 10 * i, fill=water, region=+cyl)
        unis.append(openmc.Universe(10 + i, cells=[a, b]))
    # A point outside the hexagonal array but inside the slab would abort OpenMC without an outer
    # universe: one gas cell (id 900). sm-rt maps it to "no real element".
    lat = openmc.HexLattice(50)
    lat.outer = openmc.Universe(30, cells=[openmc.Cell(900, fill=gas)])
    lat.center = CENTER
    lat.pitch = (PITCH,)
    lat.orientation = orientation
    rings, k = [], 0
    for ring in range(N_RINGS):  # outermost first
        n = 6 * (N_RINGS - 1 - ring) or 1
        rings.append(unis[k : k + n])
        k += n
    lat.universes = rings
    zlo, zhi = openmc.ZPlane(-ZH, boundary_type="vacuum"), openmc.ZPlane(ZH, boundary_type="vacuum")
    big = 2.0 * N_RINGS * PITCH
    x0, x1 = openmc.XPlane(-big, boundary_type="vacuum"), openmc.XPlane(big, boundary_type="vacuum")
    y0, y1 = openmc.YPlane(-big, boundary_type="vacuum"), openmc.YPlane(big, boundary_type="vacuum")
    root = openmc.Universe(0, cells=[openmc.Cell(1, fill=lat, region=+x0 & -x1 & +y0 & -y1 & +zlo & -zhi)])
    return openmc.Geometry(root), openmc.Materials([fuel, water, gas]), n_el


def write_support(work):
    import openmc

    groups = openmc.mgxs.EnergyGroups([0.0, 20.0e6])
    xs = openmc.XSdata("dummy", groups)
    xs.order = 0
    xs.set_total(np.array([1.0]))
    xs.set_absorption(np.array([0.5]))
    xs.set_scatter_matrix(np.array([[[0.5]]]))
    lib = openmc.MGXSLibrary(groups)
    lib.add_xsdata(xs)
    lib.export_to_hdf5(os.path.join(work, "mgxs.h5"))
    s = openmc.Settings()
    s.batches = 1
    s.particles = 10
    s.inactive = 0
    s.energy_mode = "multi-group"
    s.source = openmc.IndependentSource(space=openmc.stats.Point((0, 0, 0)))
    s.export_to_xml(os.path.join(work, "settings.xml"))


def rewrite_materials_xs(path):
    t = ET.parse(path)
    root = t.getroot()
    for e in root.findall("cross_sections"):
        root.remove(e)
    ET.SubElement(root, "cross_sections").text = "mgxs.h5"
    t.write(path)


def hex_center(orientation, q, r):
    """Centre of the element with OpenMC path indices (q, r) relative to the lattice centre (the
    map that DESIGN.md documents; the oracle checks it)."""
    s = math.sqrt(3.0) / 2.0
    if orientation == "x":
        return (PITCH * (q + r / 2.0), PITCH * s * r)
    return (PITCH * s * q, PITCH * (r + q / 2.0))


def oracle(work, orientation):
    """Runs inside the subprocess: ask openmc.lib about the model in `work`."""
    import openmc
    import openmc.lib

    geom, _, n_el = build(orientation)
    openmc.lib.init(["-s", "1"], output=False)
    rng = np.random.default_rng(20260 + (1 if orientation == "y" else 0))

    def find(p):
        try:
            cell, inst = openmc.lib.find_cell(tuple(float(x) for x in p))
        except Exception:
            return None
        return int(cell.id), int(inst)

    # OpenMC's own lattice path of every cell (the element indices).
    g = openmc.Geometry.from_xml(os.path.join(work, "geometry.xml"), os.path.join(work, "materials.xml"))
    g.determine_paths()
    cells = g.get_all_cells()
    elements = []
    for i in range(n_el):
        fc, mc = 100 + 10 * i, 101 + 10 * i
        paths = cells[fc].paths
        assert len(paths) == 1, paths
        m = re.findall(r"l(\d+)\((-?\d+),(-?\d+)\)", paths[0])
        assert len(m) == 1, paths[0]
        # The path prints centred axial indices (they run from -(n-1) to n-1 and can be negative).
        q, r = int(m[0][1]), int(m[0][2])
        assert [int(m[0][0])] == [50]
        elements.append({"universe": i, "fuel_cell": fc, "mod_cell": mc, "radius": radius(i), "q": q, "r": r})
    # Ring and position in OpenMC's ring order (outermost ring first, then along the ring).
    k = 0
    for ring in range(N_RINGS):
        n = 6 * (N_RINGS - 1 - ring) or 1
        for pos in range(n):
            elements[k]["ring"], elements[k]["position"] = ring, pos
            k += 1

    lo = np.array([CENTER[0] - N_RINGS * PITCH, CENTER[1] - N_RINGS * PITCH, -ZH])
    hi = np.array([CENTER[0] + N_RINGS * PITCH, CENTER[1] + N_RINGS * PITCH, ZH])
    pts = []

    def add(p, kind):
        r = find(p)
        pts.append({"p": [float(x) for x in p], "kind": kind, "cell": None if r is None else r[0]})

    for _ in range(N_UNIFORM):
        p = lo + rng.random(3) * (hi - lo)
        p[2] = (rng.random() * 2.0 - 1.0) * (ZH - NUDGE)
        add(p, "uniform")
    # Near the faces and vertices of the tiling: from a random element, a point at the middle of a
    # random one of its six flats or at one of its six corners, shifted by a random vector of
    # length <= NUDGE (both sides of the face), and also a few exactly on them.
    s3 = math.sqrt(3.0)
    for _ in range(N_EDGE):
        e = elements[rng.integers(n_el)]
        c = hex_center(orientation, e["q"], e["r"])
        k = int(rng.integers(6))
        if rng.random() < 0.5:  # a flat (apothem PITCH/2) at angle base + 60 k
            base = 0.0 if orientation == "x" else math.pi / 6
            ang, rad = base + k * math.pi / 3, 0.5 * PITCH * (0.2 * (rng.random() - 0.5) + 1.0 if rng.random() < 0.5 else 1.0)
            off = (rad * math.cos(ang), rad * math.sin(ang))
            shift = (rng.random() * 2 - 1) * NUDGE
            nrm = (math.cos(ang), math.sin(ang))
            p = [CENTER[0] + c[0] + off[0] + shift * nrm[0], CENTER[1] + c[1] + off[1] + shift * nrm[1]]
            # slide along the flat, within it
            tang = (-nrm[1], nrm[0])
            slide = (rng.random() * 2 - 1) * (PITCH / (2 * s3)) * 0.98
            p = [p[0] + slide * tang[0], p[1] + slide * tang[1]]
        else:  # a corner at radius PITCH / sqrt(3)
            base = math.pi / 6 if orientation == "x" else 0.0
            ang = base + k * math.pi / 3
            rad = PITCH / s3
            d = rng.random(2) * 2 - 1
            d = d / max(np.linalg.norm(d), 1e-9) * NUDGE * rng.random()
            p = [CENTER[0] + c[0] + rad * math.cos(ang) + d[0], CENTER[1] + c[1] + rad * math.sin(ang) + d[1]]
        add([p[0], p[1], (rng.random() * 2.0 - 1.0) * (ZH - NUDGE)], "edge")

    out = {
        "openmc_version": openmc.__version__,
        "method": "openmc.lib.find_cell on the committed geometry.xml/materials.xml (multi-group dummy library workaround)",
        "orientation": orientation,
        "n_rings": N_RINGS,
        "pitch": PITCH,
        "center": list(CENTER),
        "zhalf": ZH,
        "elements": elements,
        "cell_material": {str(100 + 10 * i): "fuel" for i in range(n_el)} | {str(101 + 10 * i): "water" for i in range(n_el)} | {"900": "gas"},
        "outer_cell": 900,
        "points": pts,
    }
    with open(os.path.join(work, "oracle.json"), "w") as f:
        json.dump(out, f, indent=None, separators=(",", ":"))
    openmc.lib.finalize()


def run_model(name, outdir):
    orientation = name.split("_")[1]
    geom, mats, _ = build(orientation)
    work = tempfile.mkdtemp(prefix="smrt_hex_omc_")
    prev = {signal.SIGINT: signal.signal(signal.SIGINT, lambda *_: sys.exit(130)), signal.SIGTERM: signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))}
    try:
        geom.export_to_xml(os.path.join(work, "geometry.xml"))
        mats.export_to_xml(os.path.join(work, "materials.xml"))
        write_support(work)
        dest = os.path.join(outdir, name)
        os.makedirs(dest, exist_ok=True)
        shutil.copy(os.path.join(work, "materials.xml"), os.path.join(dest, "materials.xml"))
        rewrite_materials_xs(os.path.join(work, "materials.xml"))
        subprocess.run([sys.executable, os.path.abspath(__file__), "--oracle", orientation, "--work", work], check=True, cwd=work, env={**os.environ, "TMPDIR": work, "TEMP": work, "TMP": work})
        for fn in ("geometry.xml", "oracle.json"):
            shutil.copy(os.path.join(work, fn), os.path.join(dest, fn))
        print("wrote", dest)
    finally:
        for sig, h in prev.items():
            signal.signal(sig, h)
        shutil.rmtree(work, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("models", nargs="*")
    ap.add_argument("--out", default=HERE)
    ap.add_argument("--oracle")
    ap.add_argument("--work")
    a = ap.parse_args()
    if a.oracle:
        os.chdir(a.work)
        oracle(a.work, a.oracle)
        return
    for name in a.models or ["hex_x", "hex_y"]:
        run_model(name, a.out)


if __name__ == "__main__":
    main()

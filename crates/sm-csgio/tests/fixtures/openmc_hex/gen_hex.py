#!/usr/bin/env python
"""Generate the OpenMC hexagonal-lattice fixtures of sm-csgio (W4-0c ACR-CSG-HEXIR, ruling C7).

Run with the conda env's python (OpenMC 0.16.0):

    /Users/rodrigo/opt/miniforge/envs/openmc/bin/python gen_hex.py [--out DIR] [MODEL ...]

Models: hex_x, hex_y (3 rings, 2-D lattice) and hex_x3d, hex_y3d (3 rings, 2 axial layers). For each
this writes `<model>/geometry.xml`, `<model>/materials.xml` and `<model>/oracle.json`; the Rust test
`tests/openmc_hex.rs` reads only those files (OpenMC is not needed at test time).

Every element has its own universe (a fuel disc of radius `0.20 + 0.012 u` in water, cells `100 + 10 u`
and `101 + 10 u`, universe id `10 + u`, `u` counting in OpenMC's own order: layer by layer from the
lowest z, ring by ring outermost first, along the ring), the lattice is off the origin and has a
gas `outer` universe, so the element index, the orientation, the ring order and the layer order are all
observable.

Checks done here, before anything is written (the script fails otherwise):

* OpenMC's own path indices of every cell (`determine_paths`, centred axial `(q, r)`) equal the
  ring/position map documented in `DESIGN.md` (`ring_walk` below is that map);
* `openmc.lib.find_cell` at the centre of every element (centre from the DESIGN.md formulas) is
  that element's fuel cell, so the centres are right;
* the oracle then records 600 points (300 uniform, 300 within 5e-3 of a flat or corner of the tiling)
  and the cell OpenMC finds at each.
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
N_UNIFORM = 300
N_EDGE = 300
NUDGE = 5.0e-3
PITCH = 1.5
ZPITCH = 0.8
CENTER = (0.35, -0.2, 0.1)
N_RINGS = 3
MODELS = {"hex_x": ("x", 1), "hex_y": ("y", 1), "hex_x3d": ("x", 2), "hex_y3d": ("y", 2)}


def radius(u):
    return 0.20 + 0.012 * u


def ring_len(ring):
    """Elements in ring number `ring` (0 = outermost) of an N_RINGS lattice."""
    r = N_RINGS - 1 - ring
    return 6 * r or 1


def ring_walk(orientation):
    """The documented map: [(ring, position, q, r)] in OpenMC order (outermost first), centred axial
    indices. Radius R = N_RINGS-1-ring; start at (R, 0) for 'x', (0, R) for 'y'; then R steps along each
    of six directions, clockwise."""
    if orientation == "x":
        dirs = [(0, -1), (-1, 0), (-1, 1), (0, 1), (1, 0), (1, -1)]

        def start(R):
            return (R, 0)
    else:
        dirs = [(1, -1), (0, -1), (-1, 0), (-1, 1), (0, 1), (1, 0)]

        def start(R):
            return (0, R)

    out = []
    for ring in range(N_RINGS):
        R = N_RINGS - 1 - ring
        if R == 0:
            out.append((ring, 0, 0, 0))
            continue
        q, r = start(R)
        pos = 0
        for d in dirs:
            for _ in range(R):
                out.append((ring, pos, q, r))
                pos += 1
                q, r = q + d[0], r + d[1]
        assert (q, r) == start(R)
    return out


def hex_center(orientation, q, r):
    s = math.sqrt(3.0) / 2.0
    if orientation == "x":
        return (CENTER[0] + PITCH * (q + r / 2.0), CENTER[1] + PITCH * s * r)
    return (CENTER[0] + PITCH * s * q, CENTER[1] + PITCH * (r + q / 2.0))


def layer_z(n_axial, k):
    return CENTER[2] + ZPITCH * (k + 0.5 - n_axial / 2.0)


def build(orientation, n_axial):
    import openmc

    openmc.reset_auto_ids()
    fuel = openmc.Material(material_id=1, name="fuel")
    water = openmc.Material(material_id=2, name="water")
    gas = openmc.Material(material_id=3, name="gas")
    for m in (fuel, water, gas):
        m.add_macroscopic(openmc.Macroscopic("dummy"))
        m.set_density("macro", 1.0)
    per = 3 * N_RINGS * (N_RINGS - 1) + 1
    unis = []
    for u in range(per * n_axial):
        cyl = openmc.ZCylinder(r=radius(u))
        a = openmc.Cell(100 + 10 * u, fill=fuel, region=-cyl)
        b = openmc.Cell(101 + 10 * u, fill=water, region=+cyl)
        unis.append(openmc.Universe(10 + u, cells=[a, b]))
    lat = openmc.HexLattice(50)
    lat.outer = openmc.Universe(900, cells=[openmc.Cell(900, fill=gas)])
    if n_axial == 1:
        lat.center = CENTER[:2]
        lat.pitch = (PITCH,)
    else:
        lat.center = CENTER
        lat.pitch = (PITCH, ZPITCH)
    lat.orientation = orientation
    layers, k = [], 0
    for _ in range(n_axial):
        rings = []
        for ring in range(N_RINGS):
            n = ring_len(ring)
            rings.append(unis[k : k + n])
            k += n
        layers.append(rings)
    lat.universes = layers if n_axial > 1 else layers[0]
    big = 2.0 * N_RINGS * PITCH
    zh = ZPITCH * n_axial / 2.0
    zlo = openmc.ZPlane(CENTER[2] - zh if n_axial > 1 else -1.0, boundary_type="vacuum")
    zhi = openmc.ZPlane(CENTER[2] + zh if n_axial > 1 else 1.0, boundary_type="vacuum")
    x0, x1 = openmc.XPlane(-big, boundary_type="vacuum"), openmc.XPlane(big, boundary_type="vacuum")
    y0, y1 = openmc.YPlane(-big, boundary_type="vacuum"), openmc.YPlane(big, boundary_type="vacuum")
    root = openmc.Universe(0, cells=[openmc.Cell(1, fill=lat, region=+x0 & -x1 & +y0 & -y1 & +zlo & -zhi)])
    return openmc.Geometry(root), openmc.Materials([fuel, water, gas]), per, (zlo.z0, zhi.z0)


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


def oracle(work, orientation, n_axial):
    import openmc
    import openmc.lib

    geom, _, per, (zlo, zhi) = build(orientation, n_axial)
    openmc.lib.init(["-s", "1"], output=False)
    rng = np.random.default_rng(20261 + (1 if orientation == "y" else 0) + 2 * n_axial)

    def find(p):
        try:
            cell, inst = openmc.lib.find_cell(tuple(float(x) for x in p))
        except Exception:
            return None
        return int(cell.id)

    g = openmc.Geometry.from_xml(os.path.join(work, "geometry.xml"), os.path.join(work, "materials.xml"))
    g.determine_paths()
    cells = g.get_all_cells()
    walk = ring_walk(orientation)
    assert len(walk) == per
    elements = []
    for k in range(n_axial):
        for i, (ring, pos, q, r) in enumerate(walk):
            u = k * per + i
            fc = 100 + 10 * u
            paths = cells[fc].paths
            assert len(paths) == 1, paths
            m = re.findall(r"l(\d+)\((-?\d+),(-?\d+)(?:,(-?\d+))?\)", paths[0])
            assert len(m) == 1 and int(m[0][0]) == 50, paths[0]
            oq, orr = int(m[0][1]), int(m[0][2])
            assert (oq, orr) == (q, r), f"OpenMC path {(oq, orr)} != documented {(q, r)} for layer {k} ring {ring} pos {pos}"
            if n_axial > 1:
                assert m[0][3] != "" and int(m[0][3]) == k, f"layer index {m[0][3]} != {k}"
            cx, cy = hex_center(orientation, q, r)
            cz = layer_z(n_axial, k) if n_axial > 1 else 0.0
            got = find((cx, cy, cz))
            assert got == fc, f"find_cell at the centre of layer {k} ring {ring} pos {pos} gave {got}, expected {fc}"
            elements.append({"universe": 10 + u, "fuel_cell": fc, "mod_cell": fc + 1, "radius": radius(u), "layer": k, "ring": ring, "position": pos, "q": q, "r": r, "center": [cx, cy, cz]})

    lo = np.array([CENTER[0] - N_RINGS * PITCH, CENTER[1] - N_RINGS * PITCH, zlo])
    hi = np.array([CENTER[0] + N_RINGS * PITCH, CENTER[1] + N_RINGS * PITCH, zhi])
    pts = []

    def zr():
        return zlo + NUDGE + rng.random() * (zhi - zlo - 2 * NUDGE)

    def add(p, kind):
        pts.append({"p": [float(x) for x in p], "kind": kind, "cell": find(p)})

    for _ in range(N_UNIFORM):
        p = lo + rng.random(3) * (hi - lo)
        p[2] = zr()
        add(p, "uniform")
    s3 = math.sqrt(3.0)
    for _ in range(N_EDGE):
        e = elements[int(rng.integers(len(elements)))]
        cx, cy = e["center"][0], e["center"][1]
        k = int(rng.integers(6))
        if rng.random() < 0.5:  # near a flat
            ang = (0.0 if orientation == "x" else math.pi / 6) + k * math.pi / 3
            nrm = (math.cos(ang), math.sin(ang))
            shift = (rng.random() * 2 - 1) * NUDGE
            tang = (-nrm[1], nrm[0])
            slide = (rng.random() * 2 - 1) * (PITCH / (2 * s3)) * 0.98
            rad = 0.5 * PITCH + shift
            p = [cx + rad * nrm[0] + slide * tang[0], cy + rad * nrm[1] + slide * tang[1]]
        else:  # near a corner
            ang = (math.pi / 6 if orientation == "x" else 0.0) + k * math.pi / 3
            d = rng.random(2) * 2 - 1
            d = d / max(np.linalg.norm(d), 1e-9) * NUDGE * rng.random()
            p = [cx + PITCH / s3 * math.cos(ang) + d[0], cy + PITCH / s3 * math.sin(ang) + d[1]]
        add([p[0], p[1], zr()], "edge")

    out = {
        "openmc_version": openmc.__version__,
        "method": "openmc.lib.find_cell on the committed geometry.xml/materials.xml (multi-group dummy library workaround); element paths from determine_paths; centres checked by find_cell",
        "orientation": orientation,
        "n_rings": N_RINGS,
        "n_axial": n_axial,
        "pitch": PITCH,
        "z_pitch": ZPITCH if n_axial > 1 else None,
        "center": list(CENTER[:2]) + ([CENTER[2]] if n_axial > 1 else []),
        "z_range": [zlo, zhi],
        "elements": elements,
        "outer_cell": 900,
        "points": pts,
    }
    with open(os.path.join(work, "oracle.json"), "w") as f:
        json.dump(out, f, indent=None, separators=(",", ":"))
    openmc.lib.finalize()


def run_model(name, outdir):
    orientation, n_axial = MODELS[name]
    geom, mats, _, _ = build(orientation, n_axial)
    work = tempfile.mkdtemp(prefix="smcsg_hex_omc_")
    prev = {signal.SIGINT: signal.signal(signal.SIGINT, lambda *_: sys.exit(130)), signal.SIGTERM: signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))}
    try:
        geom.export_to_xml(os.path.join(work, "geometry.xml"))
        mats.export_to_xml(os.path.join(work, "materials.xml"))
        write_support(work)
        dest = os.path.join(outdir, name)
        os.makedirs(dest, exist_ok=True)
        shutil.copy(os.path.join(work, "materials.xml"), os.path.join(dest, "materials.xml"))
        rewrite_materials_xs(os.path.join(work, "materials.xml"))
        subprocess.run([sys.executable, os.path.abspath(__file__), "--oracle", name, "--work", work], check=True, cwd=work, env={**os.environ, "TMPDIR": work, "TEMP": work, "TMP": work})
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
        oracle(a.work, *MODELS[a.oracle])
        return
    for name in a.models or list(MODELS):
        run_model(name, a.out)


if __name__ == "__main__":
    main()

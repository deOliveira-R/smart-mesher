#!/usr/bin/env python
"""Generate the OpenMC fixtures of sm-csgio and record OpenMC's own answers (ruling C7).

Run with the conda env's python (OpenMC 0.16.0):

    /Users/rodrigo/opt/miniforge/envs/openmc/bin/python gen.py [--out DIR] [MODEL ...]

For every model this writes `<model>/geometry.xml`, `<model>/materials.xml` and
`<model>/oracle.json`. The Rust tests read only those files; OpenMC is not needed at test time.

How OpenMC is asked (no nuclear data is configured on this machine): the models use materials with
a one-group dummy *multi-group* cross-section library written on the fly (`openmc.MGXSLibrary`),
so `openmc.lib.init` accepts the very same `geometry.xml`/`materials.xml` that is committed.
`openmc.lib.find_cell(point)` then gives the cell (and instance) that OpenMC's C++ geometry code
finds. Chords are OpenMC's too: along a ray the cell is looked up at fixed steps and every change
is bisected to ~1e-12 (`CHORD_STEP` below is smaller than every feature of the models).
Each model runs in its own subprocess (OpenMC's library state is process-wide) inside a temporary
directory; only the three files above are copied out.
"""
import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

import warnings

import numpy as np

warnings.filterwarnings("ignore", message=".*instance already exists with id.*")

HERE = os.path.dirname(os.path.abspath(__file__))
CHORD_STEP = 2.0e-3
N_POINTS = 600
BISECT_ITERS = 60
MODELS = {}


def model(f):
    MODELS[f.__name__] = f
    return f


# ----------------------------------------------------------------------------------------------
# model builders (return a Spec)
# ----------------------------------------------------------------------------------------------


class Spec:
    """A model: its openmc geometry/materials, a sampling box and a few rays."""

    def __init__(self, geometry=None, materials=None, box=None, rays=(), raw_geometry=None, seed=1):
        self.geometry = geometry
        self.materials = materials
        self.raw_geometry = raw_geometry
        self.box = box  # ((x0,y0,z0),(x1,y1,z1))
        self.rays = list(rays)  # (origin, direction)
        self.seed = seed


class Mats:
    """Named materials with consecutive ids."""

    def __init__(self):
        import openmc

        self.openmc = openmc
        self.by_name = {}

    def __call__(self, name):
        if name not in self.by_name:
            m = self.openmc.Material(material_id=len(self.by_name) + 1, name=name)
            m.add_macroscopic(self.openmc.Macroscopic("dummy"))
            m.set_density("macro", 1.0)
            self.by_name[name] = m
        return self.by_name[name]

    def collection(self):
        mats = self.openmc.Materials(list(self.by_name.values()))
        return mats


def box_region(openmc, lo, hi, bc=None, sides="xyz"):
    """A box and its six planes; `bc` maps 'x-','x+',... to a boundary type."""
    bc = bc or {}
    pl = {}
    for ax, cls in (("x", openmc.XPlane), ("y", openmc.YPlane), ("z", openmc.ZPlane)):
        i = "xyz".index(ax)
        pl[ax + "-"] = cls(lo[i], boundary_type=bc.get(ax + "-", "transmission"))
        pl[ax + "+"] = cls(hi[i], boundary_type=bc.get(ax + "+", "transmission"))
    reg = +pl["x-"] & -pl["x+"] & +pl["y-"] & -pl["y+"] & +pl["z-"] & -pl["z+"]
    return reg, pl


VAC = {k: "vacuum" for k in ("x-", "x+", "y-", "y+", "z-", "z+")}


@model
def deck_o():
    """Sphere in a box (drafts/csg-import.md, Deck O)."""
    import openmc

    m = Mats()
    sph = openmc.Sphere(r=1.0, surface_id=1)
    reg, _ = box_region(openmc, (-2, -2, -2), (2, 2, 2), VAC)
    c1 = openmc.Cell(1, fill=m("fuel"), region=-sph)
    c2 = openmc.Cell(2, fill=m("water"), region=reg & +sph)
    g = openmc.Geometry(openmc.Universe(cells=[c1, c2]))
    rays = [((-2, 0.6, 0), (1, 0, 0)), ((-2, 0, 0), (1, 0, 0)), ((-2, 1.5, 0), (1, 0, 0)), ((0.3, -2, 0.2), (0, 1, 0)), ((-1.9, -1.8, -1.7), (0.5, 0.6, 0.62))]
    return Spec(g, m.collection(), ((-2, -2, -2), (2, 2, 2)), rays)


@model
def pin():
    """Pin cell: fuel, void gap, clad, water; reflective sides, vacuum ends."""
    import openmc

    m = Mats()
    c1, c2, c3 = openmc.ZCylinder(r=0.40), openmc.ZCylinder(r=0.42), openmc.ZCylinder(r=0.46)
    bc = {"x-": "reflective", "x+": "reflective", "y-": "reflective", "y+": "reflective", "z-": "vacuum", "z+": "vacuum"}
    reg, _ = box_region(openmc, (-0.63, -0.63, -2), (0.63, 0.63, 2), bc)
    cells = [
        openmc.Cell(1, fill=m("fuel"), region=-c1 & reg),
        openmc.Cell(2, fill=None, region=+c1 & -c2 & reg),
        openmc.Cell(3, fill=m("clad"), region=+c2 & -c3 & reg),
        openmc.Cell(4, fill=m("water"), region=+c3 & reg),
    ]
    g = openmc.Geometry(openmc.Universe(cells=cells))
    rays = [((-0.63, 0.1, 0.3), (1, 0, 0)), ((0.2, -0.63, -1), (0, 1, 0)), ((-0.6, -0.6, -1.9), (0.3, 0.35, 0.88)), ((0.1, 0.05, -2), (0, 0, 1))]
    return Spec(g, m.collection(), ((-0.63, -0.63, -2), (0.63, 0.63, 2)), rays)


def disc_universe(openmc, m, uid, fuel, r, cid0, center=(0.0, 0.0)):
    """A universe: a z-cylinder of `fuel` at `center`, everything else `water`."""
    cyl = openmc.ZCylinder(x0=center[0], y0=center[1], r=r)
    a = openmc.Cell(cid0, fill=m(fuel), region=-cyl)
    b = openmc.Cell(cid0 + 1, fill=m("water"), region=+cyl)
    return openmc.Universe(uid, cells=[a, b])


@model
def lattice2d():
    """3x2 rect lattice, six distinct universes, pitch (1.2, 1.0), `outer` universe, container
    larger than the array. The array is deliberately not square and not symmetric."""
    import openmc

    m = Mats()
    unis = [disc_universe(openmc, m, 10 + i, f"fuel{i}", 0.15 + 0.06 * i, 100 + 10 * i) for i in range(6)]
    outer = openmc.Universe(30, cells=[openmc.Cell(900, fill=m("gas"))])
    lat = openmc.RectLattice(20)
    lat.lower_left = (-1.8, -1.0)
    lat.pitch = (1.2, 1.0)
    # universes[row][col]: python rows run from the top (highest y) down.
    lat.universes = [[unis[0], unis[1], unis[2]], [unis[3], unis[4], unis[5]]]
    lat.outer = outer
    reg, _ = box_region(openmc, (-2.6, -1.6, -1), (2.6, 1.6, 1), VAC)
    root = openmc.Universe(0, cells=[openmc.Cell(1, fill=lat, region=reg)])
    rays = [((-2.6, 0.5, 0.0), (1, 0, 0)), ((-2.6, -0.5, 0.0), (1, 0, 0)), ((-1.2, -1.6, 0.0), (0, 1, 0)), ((0.65, -1.6, 0.0), (0, 1, 0)), ((-2.5, -1.5, -0.9), (0.8, 0.5, 0.3))]
    return Spec(openmc.Geometry(root), m.collection(), ((-2.6, -1.6, -1), (2.6, 1.6, 1)), rays)


@model
def lattice3d():
    """2x2x3 rect lattice, twelve distinct universes (a sphere each), no `outer`."""
    import openmc

    m = Mats()
    unis = []
    for i in range(12):
        s = openmc.Sphere(r=0.25 + 0.015 * i)
        a = openmc.Cell(100 + 10 * i, fill=m(f"fuel{i}"), region=-s)
        b = openmc.Cell(101 + 10 * i, fill=m("water"), region=+s)
        unis.append(openmc.Universe(10 + i, cells=[a, b]))
    lat = openmc.RectLattice(50)
    lat.lower_left = (-1.0, -1.5, -1.2)
    lat.pitch = (1.0, 1.5, 0.8)
    # python 3D arrays are [z][row][col] with rows from the top.
    lat.universes = [[[unis[4 * k + 0], unis[4 * k + 1]], [unis[4 * k + 2], unis[4 * k + 3]]] for k in range(3)]
    reg, _ = box_region(openmc, (-1, -1.5, -1.2), (1, 1.5, 1.2), VAC)
    root = openmc.Universe(0, cells=[openmc.Cell(1, fill=lat, region=reg)])
    rays = [((-1, 0.4, -1.0), (1, 0, 0)), ((-1, -1.1, 0.0), (1, 0, 0)), ((0.5, -1.5, 0.43), (0, 1, 0)), ((-0.5, -1.4, -1.2), (0, 0.05, 1)), ((-0.9, -1.4, -1.1), (0.3, 0.55, 0.3))]
    return Spec(openmc.Geometry(root), m.collection(), ((-1, -1.5, -1.2), (1, 1.5, 1.2)), rays)


def offset_universe(openmc, m, uid, c0):
    """An asymmetric universe: a box (red), a sphere (blue), the rest (mod). No cell is
    symmetric under any rotation about the origin."""
    px0, px1 = openmc.XPlane(0.5), openmc.XPlane(1.5)
    py0, py1 = openmc.YPlane(0.2), openmc.YPlane(0.7)
    pz0, pz1 = openmc.ZPlane(-0.3), openmc.ZPlane(0.9)
    sph = openmc.Sphere(x0=-1.0, y0=0.5, z0=-0.7, r=0.4)
    boxr = +px0 & -px1 & +py0 & -py1 & +pz0 & -pz1
    a = openmc.Cell(c0, fill=m("red"), region=boxr)
    b = openmc.Cell(c0 + 1, fill=m("blue"), region=-sph)
    c = openmc.Cell(c0 + 2, fill=m("mod"), region=~(boxr | -sph))
    return openmc.Universe(uid, cells=[a, b, c])


def rot_matrix(phi, theta, psi):
    """OpenMC's documented three-angle rotation matrix (hypothesis; the oracle decides)."""
    p, t, s = (math.radians(a) for a in (phi, theta, psi))
    rx = np.array([[1, 0, 0], [0, math.cos(p), -math.sin(p)], [0, math.sin(p), math.cos(p)]])
    ry = np.array([[math.cos(t), 0, math.sin(t)], [0, 1, 0], [-math.sin(t), 0, math.cos(t)]])
    rz = np.array([[math.cos(s), -math.sin(s), 0], [math.sin(s), math.cos(s), 0], [0, 0, 1]])
    return rz @ ry @ rx


ROT_CASES = [
    ("none", None),
    ("x90", (90, 0, 0)),
    ("y90", (0, 90, 0)),
    ("z90", (0, 0, 90)),
    ("x30", (30, 0, 0)),
    ("y40", (0, 40, 0)),
    ("z50", (0, 0, 50)),
    ("xyz", (30, 40, 50)),
    ("neg", (-20, 70, 110)),
    ("matrix", "matrix"),
]


@model
def rotations():
    """One container per rotation case, each filled with the same asymmetric universe, with the
    universe origin translated to the container centre. Covers single-axis, three-angle and the
    nine-entry matrix forms."""
    import openmc

    m = Mats()
    uni = offset_universe(openmc, m, 5, 100)
    n = len(ROT_CASES)
    reg_all, pl = box_region(openmc, (-3, -3, -3), (6 * (n - 1) + 3, 3, 3), VAC)
    xs = [openmc.XPlane(6 * i + 3) for i in range(n - 1)]
    bounds = [pl["x-"]] + xs + [pl["x+"]]
    cells = []
    for i, (name, rot) in enumerate(ROT_CASES):
        reg = +bounds[i] & -bounds[i + 1] & +pl["y-"] & -pl["y+"] & +pl["z-"] & -pl["z+"]
        c = openmc.Cell(1 + i, fill=uni, region=reg)
        c.translation = (6.0 * i, 0.0, 0.0)
        if rot == "matrix":
            c.rotation = rot_matrix(25, -35, 65)
        elif rot is not None:
            c.rotation = rot
        cells.append(c)
    g = openmc.Geometry(openmc.Universe(0, cells=cells))
    rays = [((6 * i - 3, 0.45, 0.3), (1, 0, 0)) for i in range(n)] + [((6 * i - 2.9, -2.9, -2.8), (0.3, 0.75, 0.6)) for i in range(n)]
    return Spec(g, m.collection(), ((-3, -3, -3), (6 * (n - 1) + 3, 3, 3)), rays)


@model
def nested():
    """Two levels of fill, each with translation and rotation: U1 (a box and a rest) carries a
    cell filled with the asymmetric universe U2. Pins the composition order."""
    import openmc

    m = Mats()
    u2 = offset_universe(openmc, m, 5, 100)
    bx = openmc.XPlane(0.0), openmc.XPlane(4.0)
    by = openmc.YPlane(-1.0), openmc.YPlane(3.0)
    bz = openmc.ZPlane(-2.0), openmc.ZPlane(2.0)
    inner_reg = +bx[0] & -bx[1] & +by[0] & -by[1] & +bz[0] & -bz[1]
    inner = openmc.Cell(10, fill=u2, region=inner_reg)
    inner.translation = (2.0, 1.0, 0.0)
    inner.rotation = (0, 40, 70)
    rest = openmc.Cell(11, fill=m("gas"), region=~inner_reg)
    u1 = openmc.Universe(6, cells=[inner, rest])
    reg, _ = box_region(openmc, (-8, -8, -8), (8, 8, 8), VAC)
    top = openmc.Cell(1, fill=u1, region=reg)
    top.translation = (0.7, -0.4, 0.9)
    top.rotation = (20, -30, 45)
    g = openmc.Geometry(openmc.Universe(0, cells=[top]))
    rays = [((-7.9, -3.0, 0.5), (1, 0, 0)), ((-2.0, -7.9, 0.4), (0, 1, 0)), ((-7.5, -7.0, -6.0), (0.5, 0.6, 0.5)), ((7.5, 6.0, 7.0), (-0.45, -0.5, -0.6)), ((0.2, 0.3, -7.9), (0.05, 0.1, 1))]
    return Spec(g, m.collection(), ((-8, -8, -8), (8, 8, 8)), rays)


@model
def translated():
    """Plain translation only, two levels, no rotation (the common case)."""
    import openmc

    m = Mats()
    u2 = offset_universe(openmc, m, 5, 100)
    inner_reg_box, pl = box_region(openmc, (-2, -2, -2), (2, 2, 2))
    inner = openmc.Cell(10, fill=u2, region=inner_reg_box)
    inner.translation = (0.5, -0.25, 0.1)
    rest = openmc.Cell(11, fill=m("gas"), region=~inner_reg_box)
    u1 = openmc.Universe(6, cells=[inner, rest])
    reg, _ = box_region(openmc, (-5, -5, -5), (5, 5, 5), VAC)
    top = openmc.Cell(1, fill=u1, region=reg)
    top.translation = (1.0, 2.0, 3.0)
    g = openmc.Geometry(openmc.Universe(0, cells=[top]))
    rays = [((-4.9, 2.4, 3.1), (1, 0, 0)), ((1.4, -4.9, 3.2), (0, 1, 0)), ((-4.5, -4.0, -4.5), (0.5, 0.6, 0.62)), ((2.0, 2.7, -4.9), (0, 0, 1))]
    return Spec(g, m.collection(), ((-5, -5, -5), (5, 5, 5)), rays)


@model
def lattice_xf():
    """A lattice-filled cell with translation and rotation, with an `outer` universe: the
    lattice frame is the filled cell's frame after the transform."""
    import openmc

    m = Mats()
    unis = [disc_universe(openmc, m, 10 + i, f"fuel{i}", 0.2 + 0.05 * i, 100 + 10 * i, center=(0.1, -0.05)) for i in range(4)]
    outer = openmc.Universe(30, cells=[openmc.Cell(900, fill=m("gas"))])
    lat = openmc.RectLattice(20)
    lat.lower_left = (-1.0, -1.0)
    lat.pitch = (1.0, 1.0)
    lat.universes = [[unis[0], unis[1]], [unis[2], unis[3]]]
    lat.outer = outer
    reg, _ = box_region(openmc, (-4, -4, -1), (4, 4, 1), VAC)
    c = openmc.Cell(1, fill=lat, region=reg)
    c.translation = (1.0, -0.5, 0.0)
    c.rotation = (0, 0, 30)
    g = openmc.Geometry(openmc.Universe(0, cells=[c]))
    rays = [((-4, 0.3, 0), (1, 0, 0)), ((-4, -1.2, 0.1), (1, 0.1, 0)), ((0.3, -4, 0), (0, 1, 0)), ((-3.9, -3.5, -0.9), (0.6, 0.7, 0.2))]
    return Spec(g, m.collection(), ((-4, -4, -1), (4, 4, 1)), rays)


@model
def outer_structured():
    """A 1x1 lattice (pitch 2) whose `outer` universe is position dependent: an off-centre
    cylinder of one material in another. OpenMC looks `outer` up in the frame of each virtual
    out-of-range element, i.e. it tiles it at the lattice pitch around the array. The Rust reader
    refuses such an `outer` (Unsupported); the test uses this oracle to check that the tiling
    reading is what OpenMC does."""
    import openmc

    m = Mats()
    inner = openmc.Universe(10, cells=[openmc.Cell(100, fill=m("fuel"))])
    outer = disc_universe(openmc, m, 30, "gas", 0.5, 900, center=(0.2, -0.1))
    lat = openmc.RectLattice(20)
    lat.lower_left = (-1.0, -1.0)
    lat.pitch = (2.0, 2.0)
    lat.universes = [[inner]]
    lat.outer = outer
    reg, _ = box_region(openmc, (-5, -5, -1), (5, 5, 1), VAC)
    root = openmc.Universe(0, cells=[openmc.Cell(1, fill=lat, region=reg)])
    return Spec(openmc.Geometry(root), m.collection(), ((-5, -5, -1), (5, 5, 1)), [((-5, 0.1, 0.0), (1, 0, 0))])


def _bays(names_and_builders, halfwidth=2.0, spacing=6.0):
    """One bay per surface along x: each holds `in` (negative side) and `out` of one surface."""
    import openmc

    m = Mats()
    n = len(names_and_builders)
    pl_x = [openmc.XPlane(spacing * i - halfwidth - 1.0, boundary_type="vacuum" if i == 0 else "transmission") for i in range(n)]
    pl_x2 = [openmc.XPlane(spacing * i + halfwidth + 1.0, boundary_type="vacuum" if i == n - 1 else "transmission") for i in range(n)]
    # neighbouring bays touch: use one plane between bays
    lo = [pl_x[0]] + [openmc.XPlane(spacing * i - spacing / 2.0) for i in range(1, n)]
    hi = lo[1:] + [pl_x2[-1]]
    py = openmc.YPlane(-3, boundary_type="vacuum"), openmc.YPlane(3, boundary_type="vacuum")
    pz = openmc.ZPlane(-3, boundary_type="vacuum"), openmc.ZPlane(3, boundary_type="vacuum")
    cells = []
    for i, (name, build) in enumerate(names_and_builders):
        s = build(spacing * i)
        reg = +lo[i] & -hi[i] & +py[0] & -py[1] & +pz[0] & -pz[1]
        cells.append(openmc.Cell(10 + 2 * i, fill=m("in"), region=-s & reg))
        cells.append(openmc.Cell(11 + 2 * i, fill=m("out"), region=+s & reg))
    g = openmc.Geometry(openmc.Universe(0, cells=cells))
    return g, m, lo[0], hi[-1]


@model
def surfaces():
    """Every supported surface type, one bay each: `in` is the negative side (f < 0)."""
    import openmc

    S = openmc
    bld = [
        ("plane", lambda c: S.Plane(a=1.0, b=2.0, c=-0.5, d=c + 0.3)),
        ("x-plane", lambda c: S.XPlane(c + 0.4)),
        ("y-plane", lambda c: S.YPlane(0.25)),
        ("z-plane", lambda c: S.ZPlane(-0.35)),
        ("x-cylinder", lambda c: S.XCylinder(y0=0.3, z0=-0.2, r=0.9)),
        ("y-cylinder", lambda c: S.YCylinder(x0=c + 0.2, z0=-0.35, r=0.8)),
        ("z-cylinder", lambda c: S.ZCylinder(x0=c - 0.25, y0=0.4, r=1.1)),
        ("cylinder", lambda c: S.Cylinder(x0=c + 0.1, y0=-0.2, z0=0.3, r=0.7, dx=1.0, dy=0.5, dz=-0.25)),
        ("sphere", lambda c: S.Sphere(x0=c + 0.3, y0=-0.1, z0=0.2, r=1.2)),
        ("x-cone", lambda c: S.XCone(x0=c + 0.2, y0=0.1, z0=-0.3, r2=0.36)),
        ("y-cone", lambda c: S.YCone(x0=c - 0.2, y0=0.4, z0=0.1, r2=0.5)),
        ("z-cone", lambda c: S.ZCone(x0=c + 0.1, y0=-0.3, z0=0.2, r2=0.3)),
        ("cone", lambda c: S.Cone(x0=c + 0.1, y0=0.2, z0=-0.1, r2=0.4, dx=0.3, dy=1.0, dz=0.2)),
        ("quadric", lambda c: _quadric(S, c)),
        ("x-torus", lambda c: S.XTorus(x0=c + 0.1, y0=0.2, z0=-0.1, a=1.2, b=0.4, c=0.4)),
        ("y-torus", lambda c: S.YTorus(x0=c - 0.1, y0=0.1, z0=0.2, a=1.1, b=0.35, c=0.35)),
        ("z-torus", lambda c: S.ZTorus(x0=c + 0.2, y0=-0.1, z0=0.15, a=1.3, b=0.45, c=0.45)),
    ]
    g, m, lo, hi = _bays(bld)
    n = len(bld)
    rays = []
    for i in range(n):
        cx = 6.0 * i
        rays += [((cx - 2.5, 0.12, 0.07), (1, 0, 0)), ((cx + 0.13, -2.9, 0.21), (0, 1, 0)), ((cx - 0.17, 0.3, -2.9), (0, 0, 1)), ((cx - 2.4, -2.3, -2.1), (0.6, 0.62, 0.5))]
    return Spec(g, m.collection(), ((-3, -3, -3), (6 * (n - 1) + 3, 3, 3)), rays)


def _quadric(S, c):
    """A general quadric with all ten coefficients nonzero, f(x - c, y, z) expanded."""
    A, B, C, D, E, F = 0.5, 1.0, 2.0, 0.2, -0.3, 0.1
    G, H, J, K = -0.4, 0.2, 0.1, -1.5
    # A(x-c)^2 + D(x-c)y + F(x-c)z + G(x-c) + ... : shift x by c.
    return S.Quadric(a=A, b=B, c=C, d=D, e=E, f=F, g=G - 2 * A * c, h=H - D * c, j=J - F * c, k=K + A * c * c - G * c)


@model
def grammar():
    """Region grammar, written by hand (raw XML): `+`, `~`, `|`, parentheses and the precedence of
    `|` below intersection, in infinite universes clipped by a vacuum container. Mixes attribute and
    child-element forms."""
    geometry = """<?xml version="1.0"?>
<geometry>
  <!-- infinite universe 5: cell 11 is  -1 | 2 -3  =  -1 | (2 -3) -->
  <cell id="11" material="1" universe="5" region="-1 | 2 -3"/>
  <cell id="12" material="2" universe="5" region="~(-1 | 2 -3)"/>
  <!-- infinite universe 6: double complement -->
  <cell id="13" universe="6"><material>3</material><region>~(+1 | ~(2 -3))</region></cell>
  <cell id="14" material="4" universe="6" region="~(-1 2 -3)"/>
  <!-- the left part is universe 5, the right part universe 6 -->
  <cell id="1" fill="5" region="  +11 -10 12
        -13 14 -15 "/>
  <cell id="2" fill="6" region="10 -16 12 -13 14 -15"/>
  <surface id="1" type="sphere" coeffs="0 0 0 1.5"/>
  <surface id="2" type="x-plane" coeffs="0.5"/>
  <surface id="3" type="sphere" coeffs="1 1 0 1.2"/>
  <surface id="10" type="x-plane" coeffs="0.2"/>
  <surface id="11" type="x-plane" coeffs="-4" boundary="vacuum"/>
  <surface id="12" type="y-plane" coeffs="-4" boundary="vacuum"/>
  <surface id="13" type="y-plane" coeffs="4" boundary="vacuum"/>
  <surface id="14" type="z-plane" coeffs="-4" boundary="vacuum"/>
  <surface id="15" type="z-plane" coeffs="4" boundary="vacuum"/>
  <surface id="16" type="x-plane" coeffs="4" boundary="vacuum"/>
</geometry>
"""
    rays = [((-4, 0.3, 0.1), (1, 0, 0)), ((-4, 1.1, 0.2), (1, 0, 0)), ((-3.9, -3.9, -3.9), (0.55, 0.6, 0.58)), ((0.3, -4, 0.2), (0, 1, 0)), ((1.0, -3.9, -0.1), (0, 1, 0))]
    mats = [(1, "a"), (2, "b"), (3, "c"), (4, "d")]
    return Spec(raw_geometry=geometry, materials=mats, box=((-4, -4, -4), (4, 4, 4)), rays=rays)


# ----------------------------------------------------------------------------------------------
# writing and asking OpenMC
# ----------------------------------------------------------------------------------------------


def write_model(spec, out):
    """Write geometry.xml and materials.xml of `spec` into the directory `out`."""
    import openmc

    os.makedirs(out, exist_ok=True)
    if spec.raw_geometry is not None:
        with open(os.path.join(out, "geometry.xml"), "w") as f:
            f.write(spec.raw_geometry)
        mats = openmc.Materials()
        for mid, name in spec.materials:
            mm = openmc.Material(material_id=mid, name=name)
            mm.add_macroscopic(openmc.Macroscopic("dummy"))
            mm.set_density("macro", 1.0)
            mats.append(mm)
        mats.export_to_xml(os.path.join(out, "materials.xml"))
    else:
        spec.geometry.export_to_xml(os.path.join(out, "geometry.xml"))
        spec.materials.export_to_xml(os.path.join(out, "materials.xml"))


def write_support(work):
    """settings.xml and the dummy multi-group library into `work`."""
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
    """Point materials.xml at the dummy library (OpenMC reads it next to the materials)."""
    import xml.etree.ElementTree as ET

    t = ET.parse(path)
    root = t.getroot()
    for e in root.findall("cross_sections"):
        root.remove(e)
    ET.SubElement(root, "cross_sections").text = "mgxs.h5"
    t.write(path)


def oracle(work, spec_name):
    """Runs inside the subprocess: ask openmc.lib about the model in `work`."""
    import openmc
    import openmc.lib

    spec = MODELS[spec_name]()
    openmc.lib.init(["-s", "1"], output=False)
    box = np.array(spec.box, dtype=float)
    rng = np.random.default_rng(spec.seed)

    def find(p):
        try:
            cell, inst = openmc.lib.find_cell(tuple(float(x) for x in p))
        except Exception:
            return None
        return int(cell.id), int(inst)

    # Material of every cell, read from the committed XML text (attribute or child form).
    import xml.etree.ElementTree as ET

    def val(e, k):
        if k in e.attrib:
            return e.attrib[k].strip()
        c = e.find(k)
        return None if c is None or c.text is None else c.text.strip()

    names = {}
    for e in ET.parse(os.path.join(work, "materials.xml")).getroot().findall("material"):
        names[e.attrib["id"]] = e.attrib.get("name") or ("material_" + e.attrib["id"])
    cell_mat = {}
    for e in ET.parse(os.path.join(work, "geometry.xml")).getroot().findall("cell"):
        mat = val(e, "material")
        if mat is not None:
            cell_mat[int(e.attrib["id"])] = "void" if mat == "void" else names[mat]

    pts = []
    for _ in range(N_POINTS):
        p = box[0] + rng.random(3) * (box[1] - box[0])
        r = find(p)
        pts.append({"p": [float(x) for x in p], "cell": None if r is None else r[0], "instance": None if r is None else r[1]})

    chords = []
    for o, d in spec.rays:
        o = np.array(o, dtype=float)
        d = np.array(d, dtype=float)
        d /= np.linalg.norm(d)
        # Start just inside: a ray origin on a surface is ambiguous for every code.
        o = o + 5.0e-3 * d
        # Step along the ray; every change of (cell, instance) is bisected to ~1e-12. Points in no
        # cell give key None; the ray ends once it has left the model for good.
        far = np.linalg.norm(box[1] - box[0]) * 1.5

        def key(t):
            return find(o + t * d)

        trans = []  # (t, key before, key after)
        prev_t, prev_k = 0.0, key(0.0)
        first_k = prev_k
        seen_inside = prev_k is not None
        gap = 0
        t = CHORD_STEP
        while t < far:
            k = key(t)
            if k != prev_k:
                lo, hi = prev_t, t
                for _ in range(BISECT_ITERS):
                    mid = 0.5 * (lo + hi)
                    if key(mid) == prev_k:
                        lo = mid
                    else:
                        hi = mid
                trans.append((0.5 * (lo + hi), prev_k, k))
            if k is None:
                gap += 1
                if seen_inside and gap > 20:
                    break
            else:
                gap = 0
                seen_inside = True
            prev_t, prev_k = t, k
            t += CHORD_STEP
        segs = []
        start = 0.0
        cur = first_k
        for tb, _, after in trans:
            if cur is not None:
                segs.append((start, tb, cur))
            start, cur = tb, after
        if cur is not None:
            segs.append((start, prev_t, cur))
        chords.append(
            {
                "o": [float(x) for x in o],
                "d": [float(x) for x in d],
                "segments": [{"t0": float(a), "t1": float(b), "cell": k[0]} for a, b, k in segs],
            }
        )
    out = {
        "openmc_version": openmc.__version__,
        "method": "openmc.lib.find_cell on the committed geometry.xml/materials.xml (multi-group dummy library workaround); chords by stepping %g and bisecting to 1e-12" % CHORD_STEP,
        "cell_material": {str(k): v for k, v in sorted(cell_mat.items())},
        "points": pts,
        "chords": chords,
    }
    with open(os.path.join(work, "oracle.json"), "w") as f:
        json.dump(out, f, indent=None, separators=(",", ":"))
    openmc.lib.finalize()


def run_model(name, outdir):
    spec = MODELS[name]()
    work = tempfile.mkdtemp(prefix="smcsg_omc_")
    try:
        write_model(spec, work)
        write_support(work)
        dest = os.path.join(outdir, name)
        os.makedirs(dest, exist_ok=True)
        # The committed materials.xml is the clean one; OpenMC is run on a copy that also names the
        # dummy library (the only difference).
        shutil.copy(os.path.join(work, "materials.xml"), os.path.join(dest, "materials.xml"))
        rewrite_materials_xs(os.path.join(work, "materials.xml"))
        subprocess.run([sys.executable, os.path.abspath(__file__), "--oracle", name, "--work", work], check=True, cwd=work)
        for fn in ("geometry.xml", "oracle.json"):
            shutil.copy(os.path.join(work, fn), os.path.join(dest, fn))
        print("wrote", dest)
    finally:
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
    for name in a.models or list(MODELS):
        run_model(name, a.out)


if __name__ == "__main__":
    main()

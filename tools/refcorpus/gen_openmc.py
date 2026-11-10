"""OpenMC decks written by smart-mesh (W4-REFCORPUS, source 1).

Run by build.py with the conda env python: ``python -I -B gen_openmc.py <id> <outdir>``.
Every deck calls ``openmc.reset_auto_ids()`` first and creates objects in a fixed order, so the
ids and therefore the XML bytes are reproducible. No cross sections are referenced.
"""

from __future__ import annotations

import sys
from collections.abc import Callable
from pathlib import Path
from typing import Any

import openmc  # type: ignore[import-not-found,unused-ignore]

# PWR pin dimensions (cm), the usual textbook values.
R_FUEL, R_GAP, R_CLAD, PITCH = 0.4096, 0.4180, 0.4750, 1.26
# 17x17 guide-tube positions (row, col), top row first; (8, 8) is the instrument tube.
GUIDE_TUBES = [
    (2, 5), (2, 8), (2, 11), (3, 3), (3, 13), (5, 2), (5, 5), (5, 8), (5, 11), (5, 14),
    (8, 2), (8, 5), (8, 11), (8, 14), (11, 2), (11, 5), (11, 8), (11, 11), (11, 14),
    (13, 3), (13, 13), (14, 5), (14, 8), (14, 11),
]  # fmt: skip
Z_HALF = 10.0


class Mats:
    """The four materials every PWR deck shares, created in a fixed order."""

    def __init__(self) -> None:
        self.fuel = openmc.Material(name="UO2 3.1%")
        self.fuel.add_nuclide("U235", 0.0105)
        self.fuel.add_nuclide("U238", 0.3228)
        self.fuel.add_element("O", 0.6667)
        self.fuel.set_density("g/cm3", 10.3)
        self.clad = openmc.Material(name="Zircaloy")
        self.clad.add_element("Zr", 1.0)
        self.clad.set_density("g/cm3", 6.55)
        self.water = openmc.Material(name="Borated water")
        self.water.add_nuclide("H1", 2.0)
        self.water.add_element("O", 1.0)
        self.water.add_s_alpha_beta("c_H_in_H2O")
        self.water.set_density("g/cm3", 0.74)
        self.gas = openmc.Material(name="Helium")
        self.gas.add_element("He", 1.0)
        self.gas.set_density("g/cm3", 0.0015)

    def all(self) -> list[Any]:
        return [self.fuel, self.clad, self.water, self.gas]


def pin_universe(m: Mats, name: str, fuel: bool = True) -> Any:
    """A fuel pin (fuel, gap, clad in water) or a guide tube (water inside clad)."""
    r1 = openmc.ZCylinder(r=R_FUEL if fuel else 0.56)
    r2 = openmc.ZCylinder(r=R_GAP if fuel else 0.56 + 0.04)
    r3 = openmc.ZCylinder(r=R_CLAD if fuel else 0.60)
    cells = []
    inner = m.fuel if fuel else m.water
    cells.append(openmc.Cell(fill=inner, region=-r1, name=f"{name} inner"))
    cells.append(openmc.Cell(fill=m.gas if fuel else m.clad, region=+r1 & -r2, name=f"{name} gap"))
    cells.append(openmc.Cell(fill=m.clad, region=+r2 & -r3, name=f"{name} clad"))
    cells.append(openmc.Cell(fill=m.water, region=+r3, name=f"{name} water"))
    return openmc.Universe(cells=cells, name=name)


def water_universe(m: Mats) -> Any:
    return openmc.Universe(cells=[openmc.Cell(fill=m.water, name="water")], name="water")


def box(half_x: float, half_y: float, z0: float, z1: float, bc: str = "reflective") -> tuple[Any, list[Any]]:
    """A box region with four side planes of boundary type ``bc`` and vacuum z planes."""
    xl, xr = openmc.XPlane(-half_x, boundary_type=bc), openmc.XPlane(half_x, boundary_type=bc)
    yl, yr = openmc.YPlane(-half_y, boundary_type=bc), openmc.YPlane(half_y, boundary_type=bc)
    zl, zr = openmc.ZPlane(z0, boundary_type="vacuum"), openmc.ZPlane(z1, boundary_type="vacuum")
    return +xl & -xr & +yl & -yr & +zl & -zr, [xl, xr, yl, yr, zl, zr]


def rect_lattice(n: int, pitch: float, rows: list[list[Any]], outer: Any = None) -> Any:
    lat = openmc.RectLattice(name=f"{n}x{n} lattice")
    lat.pitch = (pitch, pitch)
    lat.lower_left = (-n * pitch / 2, -n * pitch / 2)
    lat.universes = rows
    if outer is not None:
        lat.outer = outer
    return lat


def finish(root_cell: Any, mats: list[Any], out: Path) -> None:
    geom = openmc.Geometry(openmc.Universe(cells=[root_cell], name="root"))
    geom.export_to_xml(str(out / "geometry.xml"))
    openmc.Materials(mats).export_to_xml(str(out / "materials.xml"))


def deck_pin(out: Path) -> None:
    """PWR pin cell, reflective sides."""
    m = Mats()
    pin = pin_universe(m, "pin")
    region, _ = box(PITCH / 2, PITCH / 2, -Z_HALF, Z_HALF)
    finish(openmc.Cell(fill=pin, region=region, name="root cell"), m.all(), out)


def deck_assembly(out: Path) -> None:
    """17x17 PWR assembly: 264 fuel pins, 24 guide tubes and an instrument tube (two universes)."""
    m = Mats()
    fuel = pin_universe(m, "fuel pin")
    guide = pin_universe(m, "guide tube", fuel=False)
    rows = [[fuel] * 17 for _ in range(17)]
    for r, c in [*GUIDE_TUBES, (8, 8)]:
        rows[r][c] = guide
    lat = rect_lattice(17, PITCH, rows)
    region, _ = box(17 * PITCH / 2, 17 * PITCH / 2, -Z_HALF, Z_HALF)
    finish(openmc.Cell(fill=lat, region=region, name="root cell"), m.all(), out)


def deck_mini(out: Path) -> None:
    """3x3 mini-assembly: eight fuel pins around one guide tube."""
    m = Mats()
    fuel = pin_universe(m, "fuel pin")
    guide = pin_universe(m, "guide tube", fuel=False)
    rows = [[fuel] * 3 for _ in range(3)]
    rows[1][1] = guide
    lat = rect_lattice(3, PITCH, rows)
    region, _ = box(1.5 * PITCH, 1.5 * PITCH, -Z_HALF, Z_HALF)
    finish(openmc.Cell(fill=lat, region=region, name="root cell"), m.all(), out)


def deck_layered(out: Path) -> None:
    """One pin with axial layers: bottom plug, fuel, plenum, top plug (z planes)."""
    m = Mats()
    r1, r2, r3 = openmc.ZCylinder(r=R_FUEL), openmc.ZCylinder(r=R_GAP), openmc.ZCylinder(r=R_CLAD)
    z = [openmc.ZPlane(v) for v in (5.0, 365.0, 400.0, 405.0)]
    cells = [
        openmc.Cell(fill=m.clad, region=-r3 & -z[0], name="bottom plug"),
        openmc.Cell(fill=m.fuel, region=-r1 & +z[0] & -z[1], name="fuel"),
        openmc.Cell(fill=m.gas, region=+r1 & -r2 & +z[0] & -z[1], name="gap"),
        openmc.Cell(fill=m.clad, region=+r2 & -r3 & +z[0] & -z[2], name="clad wall"),
        openmc.Cell(fill=m.gas, region=-r2 & +z[1] & -z[2], name="plenum"),
        openmc.Cell(fill=m.clad, region=-r3 & +z[2], name="top plug"),
        openmc.Cell(fill=m.water, region=+r3, name="water"),
    ]
    pin = openmc.Universe(cells=cells, name="layered pin")
    region, _ = box(PITCH / 2, PITCH / 2, -5.0, 410.0)
    finish(openmc.Cell(fill=pin, region=region, name="root cell"), m.all(), out)


HEX_RINGS = 7
HEX_PITCH = 1.26


def hex_rings(inner: Any, other: Any | None = None) -> list[list[Any]]:
    """Ring lists for ``HexLattice.universes``, outermost ring first. ``other`` replaces the
    centre and the six vertices of the 18-entry ring in the mixed deck."""
    rings: list[list[Any]] = []
    for k in range(HEX_RINGS - 1, 0, -1):
        rings.append([inner] * (6 * k))
    rings.append([inner])
    if other is not None:
        rings[-1] = [other]
        ring3 = rings[HEX_RINGS - 4]  # the 18-entry ring: a vertex every 3 positions
        for i in range(0, len(ring3), 3):
            ring3[i] = other
    return rings


def deck_hex(out: Path, orientation: str, mixed: bool) -> None:
    m = Mats()
    fuel = pin_universe(m, "fuel pin")
    guide = pin_universe(m, "guide tube", fuel=False) if mixed else None
    lat = openmc.HexLattice(name=f"hex {orientation} {HEX_RINGS} rings")
    lat.orientation = orientation
    lat.center = (0.0, 0.0)
    lat.pitch = (HEX_PITCH,)
    lat.n_rings = HEX_RINGS
    lat.universes = hex_rings(fuel, guide)
    lat.outer = water_universe(m)
    # Hexagonal boundary just outside the outer ring (apothem = (rings - 1) * pitch + pitch / 2).
    apothem = (HEX_RINGS - 0.5) * HEX_PITCH
    edge = apothem * 2 / 3**0.5
    prism = openmc.model.HexagonalPrism(edge_length=edge, orientation=orientation, boundary_type="reflective")
    zl, zr = openmc.ZPlane(-Z_HALF, boundary_type="vacuum"), openmc.ZPlane(Z_HALF, boundary_type="vacuum")
    cell = openmc.Cell(fill=lat, region=-prism & +zl & -zr, name="root cell")
    mats = m.all()
    geom = openmc.Geometry(openmc.Universe(cells=[cell], name="root"))
    geom.export_to_xml(str(out / "geometry.xml"))
    openmc.Materials(mats).export_to_xml(str(out / "materials.xml"))
    # Self-check with OpenMC's own point lookup: the corner pin centre is a fuel cell, a point
    # beyond the outer ring is outside, so the prism orientation matches the lattice orientation.
    corner = (HEX_RINGS - 1) * HEX_PITCH
    inside = (0.0, corner, 0.0) if orientation == "y" else (corner, 0.0, 0.0)
    far = (0.0, 2 * apothem, 0.0)
    assert geom.find(inside), f"{orientation}: corner pin centre not inside the boundary"
    assert not geom.find(far), f"{orientation}: boundary too large"


def deck_triso(out: Path) -> None:
    """TRISO-like particles: one five-layer sphere universe in a 4x4x4 regular lattice."""
    graphite = openmc.Material(name="Graphite")
    graphite.add_element("C", 1.0)
    graphite.set_density("g/cm3", 1.7)
    kernel = openmc.Material(name="UCO kernel")
    kernel.add_nuclide("U235", 0.2)
    kernel.add_nuclide("U238", 0.8)
    kernel.add_element("O", 1.0)
    kernel.set_density("g/cm3", 10.4)
    pyc = openmc.Material(name="PyC")
    pyc.add_element("C", 1.0)
    pyc.set_density("g/cm3", 1.9)
    sic = openmc.Material(name="SiC")
    sic.add_element("Si", 1.0)
    sic.add_element("C", 1.0)
    sic.set_density("g/cm3", 3.2)
    radii = [0.02125, 0.0345, 0.0385, 0.0425, 0.0465]
    spheres = [openmc.Sphere(r=r) for r in radii]
    fills = [kernel, graphite, pyc, sic, pyc]
    cells = [openmc.Cell(fill=fills[0], region=-spheres[0], name="kernel")]
    for i in range(1, 5):
        cells.append(openmc.Cell(fill=fills[i], region=+spheres[i - 1] & -spheres[i], name=f"layer {i}"))
    cells.append(openmc.Cell(fill=graphite, region=+spheres[-1], name="matrix"))
    particle = openmc.Universe(cells=cells, name="particle")
    pitch = 0.1
    lat = openmc.RectLattice(name="particle lattice")
    lat.pitch = (pitch, pitch, pitch)
    lat.lower_left = (-2 * pitch, -2 * pitch, -2 * pitch)
    lat.universes = [[[particle] * 4 for _ in range(4)] for _ in range(4)]
    region, _ = box(2 * pitch, 2 * pitch, -2 * pitch, 2 * pitch)
    finish(openmc.Cell(fill=lat, region=region, name="root cell"), [kernel, graphite, pyc, sic], out)


def deck_cone_torus(out: Path) -> None:
    """A cone frustum, a torus and a sphere in a water box (cone and torus surfaces)."""
    m = Mats()
    cone = openmc.ZCone(x0=-3.0, y0=0.0, z0=0.0, r2=0.25)
    zc0, zc1 = openmc.ZPlane(1.0), openmc.ZPlane(4.0)
    torus = openmc.ZTorus(x0=3.0, y0=0.0, z0=2.0, a=1.5, b=0.4, c=0.4)
    ball = openmc.Sphere(x0=0.0, y0=3.0, z0=2.0, r=0.8)
    region, _ = box(6.0, 6.0, 0.0, 5.0)
    inner = [
        openmc.Cell(fill=m.fuel, region=-cone & +zc0 & -zc1, name="cone frustum"),
        openmc.Cell(fill=m.clad, region=-torus, name="torus"),
        openmc.Cell(fill=m.gas, region=-ball, name="ball"),
    ]
    water = openmc.Cell(
        fill=m.water,
        region=region & ~inner[0].region & ~inner[1].region & ~inner[2].region,
        name="water",
    )
    geom = openmc.Geometry(openmc.Universe(cells=[*inner, water], name="root"))
    geom.export_to_xml(str(out / "geometry.xml"))
    openmc.Materials(m.all()).export_to_xml(str(out / "materials.xml"))


DECKS: dict[str, Callable[[Path], None]] = {
    "openmc-pin": deck_pin,
    "openmc-assembly-17x17": deck_assembly,
    "openmc-mini-3x3": deck_mini,
    "openmc-pin-layered": deck_layered,
    "openmc-hex-y-7ring": lambda out: deck_hex(out, "y", False),
    "openmc-hex-x-7ring": lambda out: deck_hex(out, "x", False),
    "openmc-hex-y-mixed": lambda out: deck_hex(out, "y", True),
    "openmc-triso-lattice": deck_triso,
    "openmc-cone-torus": deck_cone_torus,
}


def main(argv: list[str]) -> int:
    if len(argv) != 3 or argv[1] not in DECKS:
        print(f"usage: gen_openmc.py <{'|'.join(DECKS)}> <outdir>", file=sys.stderr)
        return 2
    out = Path(argv[2])
    out.mkdir(parents=True, exist_ok=True)
    openmc.reset_auto_ids()
    DECKS[argv[1]](out)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

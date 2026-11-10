"""BEAVRS extracts (W4-REFCORPUS, source 2): a pin cell, one assembly and a 2D core slice.

Run by build.py with the conda env python and ``SM_RC_BEAVRS_SRC`` set to the pinned BEAVRS
checkout's ``models/openmc`` directory (``python -I`` ignores PYTHONPATH, so the script adds it to
``sys.path`` itself): ``python -I -B gen_beavrs.py <id> <outdir>``. The pin and assembly
extracts follow the repository's own notebooks (``extract-pin.ipynb``, ``extract-assm.ipynb``);
the core slice is its ``make_beavrs.py --2d`` model. The XML is written with
``openmc.reset_auto_ids()`` first, so ids are the order of creation in the (deterministic) builder.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Any

import openmc  # type: ignore[import-not-found,unused-ignore]

# ``python -I`` ignores PYTHONPATH, so the pinned checkout is added explicitly.
if "SM_RC_BEAVRS_SRC" in os.environ:
    sys.path.insert(0, os.environ["SM_RC_BEAVRS_SRC"])


def export(root_universe: Any, out: Path) -> None:
    """geometry.xml and materials.xml for a geometry rooted at ``root_universe``."""
    geom = openmc.Geometry(root_universe)
    geom.export_to_xml(str(out / "geometry.xml"))
    openmc.Materials(geom.get_all_materials().values()).export_to_xml(str(out / "materials.xml"))


def pin(out: Path) -> None:
    import beavrs.builder  # type: ignore[import-not-found,unused-ignore]
    import beavrs.constants as c  # type: ignore[import-not-found,unused-ignore]

    b = beavrs.builder.BEAVRS()
    univ = b.openmc_geometry.get_universes_by_name("Fuel rod active region - 1.6% enr")[0]
    clad_or = openmc.ZCylinder(r=c.cladOR, name="Fuel clad OR")
    sides = openmc.model.RectangularPrism(c.pinPitch, c.pinPitch, axis="z", boundary_type="reflective")
    fuel = openmc.Cell(name="Fuel cell", fill=univ, region=-clad_or)
    water = openmc.Cell(name="Water", fill=b.mats["Borated Water"], region=+clad_or & -sides)
    export(openmc.Universe(name="root universe", cells=[fuel, water]), out)


def assembly(out: Path) -> None:
    import beavrs.builder  # type: ignore[import-not-found,unused-ignore]
    import beavrs.constants as c  # type: ignore[import-not-found,unused-ignore]

    b = beavrs.builder.BEAVRS()
    lat = next(x for x in b.openmc_geometry.get_all_lattices().values() if x.name == "Fuel 1.6% enr instr no BAs")
    sides = openmc.model.RectangularPrism(17 * c.pinPitch, 17 * c.pinPitch, axis="z", boundary_type="reflective")
    zmin = openmc.ZPlane(z0=c.struct_LowestExtent, boundary_type="vacuum")
    zmax = openmc.ZPlane(z0=c.struct_HighestExtent, boundary_type="vacuum")
    root = openmc.Cell(name="root cell", fill=lat, region=-sides & +zmin & -zmax)
    export(openmc.Universe(name="root universe", cells=[root]), out)


def core2d(out: Path) -> None:
    from beavrs.builder import BEAVRS  # type: ignore[import-not-found,unused-ignore]

    b = BEAVRS(is_symmetric=False, is_2d=True)
    os.chdir(out)  # the builder's writers use the working directory
    b.write_openmc_geometry()
    b.write_openmc_materials()


MODELS = {"beavrs-pin": pin, "beavrs-assembly": assembly, "beavrs-core-2d": core2d}


def main(argv: list[str]) -> int:
    if len(argv) != 3 or argv[1] not in MODELS:
        print(f"usage: gen_beavrs.py <{'|'.join(MODELS)}> <outdir>", file=sys.stderr)
        return 2
    out = Path(argv[2]).resolve()
    out.mkdir(parents=True, exist_ok=True)
    openmc.reset_auto_ids()
    MODELS[argv[1]](out)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

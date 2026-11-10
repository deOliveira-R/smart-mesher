"""Clean and deliberately dirty STEP files (W4-REFCORPUS, source 4).

Run by build.py under FreeCAD: ``freecadcmd gen_step.py`` with the environment variables
``SM_RC_STEP_OUT`` (output directory) and ``SM_RC_STEP_IDS`` (comma-separated ids, empty = all).
Each file is written as ``<id>.step`` with the header normalised to fixed strings, so the sha256
does not depend on the date, the user or the host. One defect per dirty file; units are model
units (the unit brick of ``drafts/vtopo.md`` T3).
"""

from __future__ import annotations

import os
import re
from collections.abc import Callable
from pathlib import Path
from typing import Any

import FreeCAD  # type: ignore[import-not-found,unused-ignore]
import Part  # type: ignore[import-not-found,unused-ignore]

V = FreeCAD.Vector
S = 1e-4  # defect size of the notch and the step
GAP = 1e-5
ORIGIN = V(0, 0, 0)


def box(x: float, y: float, z: float, dx: float, dy: float, dz: float) -> Any:
    return Part.makeBox(dx, dy, dz, V(x, y, z))


# ---- clean ----------------------------------------------------------------------------------


def clean_pin() -> Any:
    """Bare pin: cylinder r 0.475, length 10."""
    return Part.makeCylinder(0.475, 10.0)


def clean_cladded_pin() -> Any:
    """Clad tube with end plugs (one solid) around a fuel stack and a plenum (two solids)."""
    r_f, r_c, r_o = 0.4096, 0.418, 0.475
    clad = Part.makeCylinder(r_o, 12.0).cut(Part.makeCylinder(r_c, 10.0, V(0, 0, 1.0)))
    fuel = Part.makeCylinder(r_f, 8.0, V(0, 0, 1.0))
    return Part.makeCompound([clad, fuel])


def clean_assembly_can() -> Any:
    """Square can: 21.5 x 21.5 outer, 0.3 wall, length 20."""
    return box(-10.75, -10.75, 0, 21.5, 21.5, 20.0).cut(box(-10.45, -10.45, -1, 20.9, 20.9, 22.0))


def clean_grid_spacer() -> Any:
    """Grid-spacer-like plate: 13 x 13 x 1 with a 3 x 3 array of round holes (r 0.6, pitch 3)."""
    plate = box(-6.5, -6.5, 0, 13.0, 13.0, 1.0)
    for i in (-1, 0, 1):
        for j in (-1, 0, 1):
            plate = plate.cut(Part.makeCylinder(0.6, 3.0, V(3.0 * i, 3.0 * j, -1.0)))
    return plate


# ---- dirty: one defect each ----------------------------------------------------------------


def dirty_notch() -> Any:
    """Unit brick minus a cube of side 1e-4 at one corner: small edges and small faces."""
    return box(0, 0, 0, 1, 1, 1).cut(box(0, 0, 0, S, S, S))


def dirty_step() -> Any:
    """Two bricks offset by 1e-4 in x, fused: sliver faces."""
    a = box(0, 0, 0, 1, 1, 1)
    b = box(S, 0, 1, 1, 1, 1)
    return a.fuse(b).removeSplitter()


def dirty_gap() -> Any:
    """Two bodies 1e-5 apart (not touching)."""
    return Part.makeCompound([box(0, 0, 0, 1, 1, 1), box(0, 0, 1 + GAP, 1, 1, 1)])


def dirty_open_shell() -> Any:
    """A unit brick shell with one face removed."""
    faces = box(0, 0, 0, 1, 1, 1).Faces
    return Part.makeShell(faces[1:])


def dirty_small_hole() -> Any:
    """A 1 x 1 x 0.1 plate with a through hole of radius 1e-3."""
    return box(0, 0, 0, 1, 1, 0.1).cut(Part.makeCylinder(1e-3, 1.0, V(0.5, 0.5, -0.5)))


def dirty_fillet() -> Any:
    """A unit brick with one edge rounded, radius 0.1 (blend candidate)."""
    b = box(0, 0, 0, 1, 1, 1)
    edge = next(e for e in b.Edges if abs(e.CenterOfMass.x) < 1e-9 and abs(e.CenterOfMass.y) < 1e-9)
    return b.makeFillet(0.1, [edge])


def dirty_tol_edge() -> Any:
    """Six unsewn brick faces, the +x face displaced by 5e-5: every shared edge mismatches by
    5e-5, within a tolerance of 1e-4 but not exact."""
    b = box(0, 0, 0, 1, 1, 1)
    faces = []
    for f in b.Faces:
        if abs(f.CenterOfMass.x - 1.0) < 1e-9:
            f = f.translated(V(5e-5, 0, 0))
        faces.append(f)
    return Part.makeShell(faces)


MODELS: dict[str, Callable[[], Any]] = {
    "step-clean-pin": clean_pin,
    "step-clean-cladded-pin": clean_cladded_pin,
    "step-clean-assembly-can": clean_assembly_can,
    "step-clean-grid-spacer": clean_grid_spacer,
    "step-dirty-notch": dirty_notch,
    "step-dirty-step": dirty_step,
    "step-dirty-gap": dirty_gap,
    "step-dirty-open-shell": dirty_open_shell,
    "step-dirty-small-hole": dirty_small_hole,
    "step-dirty-fillet": dirty_fillet,
    "step-dirty-tol-edge": dirty_tol_edge,
}


def version() -> str:
    """``FreeCAD 1.1.4 20260928``: release and build revision, as build.py records them."""
    v = FreeCAD.Version()
    return str("FreeCAD " + ".".join(v[:3]) + " " + v[3].split()[0])


def normalise(path: Path, name: str) -> None:
    """Replace the header fields that vary (time, author, system) with fixed strings."""
    text = path.read_text(encoding="latin-1")
    fixed = f"FILE_NAME('{name}.step','2000-01-01T00:00:00',('smart-mesh refcorpus'),('smart-mesh'),'normalised','" + version() + "','');"
    text, n = re.subn(r"FILE_NAME\(.*?\);", fixed, text, count=1, flags=re.S)
    assert n == 1, "no FILE_NAME in the STEP header"
    path.write_text(text, encoding="latin-1", newline="\n")


def main() -> None:
    out = Path(os.environ["SM_RC_STEP_OUT"])
    out.mkdir(parents=True, exist_ok=True)
    ids = [i for i in os.environ.get("SM_RC_STEP_IDS", "").split(",") if i] or list(MODELS)
    for i in ids:
        shape = MODELS[i]()
        path = out / f"{i}.step"
        shape.exportStep(str(path))
        normalise(path, i)
        print(f"{i}: {len(shape.Solids)} solids {len(shape.Faces)} faces {path.stat().st_size} bytes")


main()

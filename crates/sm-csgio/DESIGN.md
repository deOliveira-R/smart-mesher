# sm-csgio design (W3c-CSG-CORE)

OpenMC is the only target (ruling C6): there is no `mcnp` module and nothing in the core is
shaped after MCNP. The core is frozen after merge; `src/openmc/` (W3c-CSG-OPENMC) fills the IR.

## Pipeline

`openmc::import(geometry, materials) -> CsgIr` then `ImportedCsg::instantiate` (flatten, no
sampling) or `ImportedCsg::import` (flatten, sampled check, reject overlaps).

1. **IR** (`ir.rs`): `BTreeMap`s of surfaces (`SurfaceDef` plus boundary flag), cells (region
   `Expr`, material, universe, `Fill`), rectangular lattices, materials.
2. **DNF** (`dnf.rs`): De Morgan and distribution; contradictory terms and repeats dropped; cap
   `MAX_TERMS = 4096` per cell (`TooManyTerms { cell }`).
3. **Instantiation** (`instantiate.rs`): depth-first placement. A placed cell's region is its own
   region AND the regions of every cell it was reached through AND the box of its lattice
   element, so infinite universe cells are clipped where used. `outer` fills the container outside
   the array box. Cycles: `UniverseCycle`; depth cap 64; cell cap `ImportOptions::max_cells`
   (default `MAX_FLAT_CELLS`).
4. **Canonical order**: surfaces deduplicated by definition bytes (`-0.0` as `0.0`; planes
   normalised to a unit normal with first nonzero component positive), unreferenced surfaces
   dropped, survivors sorted by definition bytes and renumbered; literals within a term and terms
   within a region sorted; flat cells sorted by (name, terms, source). `content_hash` is computed
   on that form, so it does not depend on deck ids or text order (test `canonical.rs`).
5. **Check** (`ImportedCsg::check`): `sm_rt::check_overlaps` over the flat leaf (default 100 000
   points, 2 000 chords, seed 0; never below the op's 10 000 and 500). An overlap becomes
   `ImportError::Overlap` naming the deck cells and lattice elements; gaps and an unbounded leaf are
   warnings (`import.gap`, `import.unbounded`).

## Conventions (IR level; format readers convert)

| Item | Convention |
|---|---|
| Surface sign | negative side is `f < 0` (inside a sphere or cylinder, below a plane along its normal) |
| `Fill::Universe` transform | maps a universe point `u` to `xf(u)` in the carrying cell's frame |
| Lattice element `[i, j, k]` | box from `lower_left + [i, j, k] * pitch`; `i` along +x, `j` +y, `k` +z |
| Lattice universe origin | element centre |
| Lattice `universes` order | `i` fastest, then `j`, then `k` (readers reverse any top-to-bottom row order) |
| Infinite pitch | allowed on an axis with one element; no clip there, centre coordinate 0 |
| `outer` universe | IR core: placed once with the lattice frame origin, fills container minus array box. OpenMC instead looks `outer` up in the frame of each virtual out-of-range element, i.e. tiles it at the lattice pitch (`outer_structured`); the OpenMC reader therefore accepts only a position-independent `outer` (cells without region, recursively) and refuses the rest as Unsupported |
| Region names (C9) | material `name` if non-empty, unique, not `void`, not another material's fallback; else `material_<id>`; no material is `void`; override map wins; two materials with one name is an error |
| Boundary flags | vacuum: `Respond(Vacuum)`; reflective: `Reflect` (warning `import.reflective_curved` unless planar); white: `Respond(White)`; others: default vacuum |

The OpenMC reader confirmed (against OpenMC itself, ruling C7) the lattice row order, the
rotation convention and the `outer` frame; see "OpenMC reader" below.

## Known limits

* The gap check samples the bounding box of the placed cells (or `ImportOptions::bounds`). Space the
  deck leaves open outside that box, for example a lattice container outside an array without
  `outer`, is not examined unless the container is given as `bounds`.
* Sampling cannot prove disjointness; a thin overlap sliver can be missed.
* The overlap finding's cell indices are read from the check's message (`cells {a} and {b}`) because
  `sm_rt::check_overlaps` returns no structured indices. `world.rs` pins the format in a unit test;
  an unparsable message becomes a `Build` error, never a lost finding.
* A lattice flattens to N copies of its universe cells; cold locate is O(cells).
* Hex lattices, periodic boundaries and DAGMC universes are not in the IR.

## Refusal list (core)

Undefined surface, material, universe or lattice; a universe that contains itself; DNF over 4096
terms; more than `max_cells` flat cells; malformed lattices (zero dimensions, wrong universe
count, non-positive pitch); a deck with no cell left; check effort below the op's defaults.

# OpenMC reader (W3c-CSG-OPENMC)

`src/openmc/` (`mod.rs` walk and lattices, `surfaces.rs`, `region.rs`, `xform.rs`) reads
`geometry.xml` and an optional `materials.xml` with `roxmltree` (DTDs refused, element depth capped
at 64 by a pre-scan, 8 M nodes, 256 MiB of text). Every field is accepted as an attribute or as a
child element. Errors carry the 1-based line (`Parse { line, col }`, `Unsupported { line }`,
`Invalid("line N: ...")`).

## Confirmed against OpenMC itself (ruling C7)

`tests/fixtures/openmc/gen.py` builds small models, writes `geometry.xml` and `materials.xml`, and
asks OpenMC 0.16.0 (the `openmc` conda env's python) what is where on exactly those files:
`openmc.lib.find_cell` at 600 random points, and chords (each ray stepped at 0.002 and every cell
change bisected to about 1e-12). No nuclear data is configured on this machine, so the models use
materials with a one-group dummy multi-group library written on the fly (`openmc.MGXSLibrary`,
`energy_mode = multi-group`); `openmc.lib.init` then accepts the committed XML (the committed
`materials.xml` lacks only the `<cross_sections>` line the oracle run adds). The answers are
committed as `oracle.json`; `tests/openmc_oracle.rs` compares the import against them (material at
every point; chord per material to 2e-9) and needs no OpenMC. Mutating the lattice row order or
the rotation transpose in the reader fails five of the eleven oracle tests.

| Item | Convention (confirmed by fixture) |
|---|---|
| Surface sense | negative side is `f < 0` for every type; cones are the double cone, quadrics keep `f` as written, `plane A B C D` is `Ax+By+Cz-D` (`surfaces`) |
| Cylinder, cone, torus coefficients | `x-cylinder y0 z0 r`, `y-cylinder x0 z0 r`, `z-cylinder x0 y0 r`; the general `cylinder` and `cone` types are not OpenMC geometry.xml types (OpenMC 0.16.0 Python writes a quadric, its C++ layer has no such type) and are refused as Unsupported; `x-cone x0 y0 z0 R^2`; tori `x0 y0 z0 A B C` with `B == C` (`surfaces`) |
| Region grammar | `~` over juxtaposition (and) over `\|`; `+n` and `n` are the positive side; parentheses nest (`grammar`) |
| Lattice `universes` text | layers `k` from the lowest `z` up; within a layer rows from the highest `y` down; each row left to right in `+x` (`lattice2d`, `lattice3d`) |
| Lattice element | box from `lower_left + [i,j,k] * pitch`; the universe's origin is the element centre; a 2-D lattice has no `z` extent (`lattice2d`, `lattice3d`) |
| `outer` | a homogeneous `outer` fills the container outside the array box (`lattice2d`, `lattice_xf`; these cannot see an origin). A structured `outer` is tiled at the lattice pitch by OpenMC (`outer_structured`, OpenMC 0.16.0 `find_cell`), which the reader does not reproduce: Unsupported |
| `translation t`, `rotation M` | OpenMC's `u = M (r - t)` from cell frame `r` to the filled universe frame `u`; the IR transform is `r = M^T u + t` (`rotations`, `translated`) |
| `rotation` angles | three angles `(phi, theta, psi)` in degrees give `M = Rz(-psi) Ry(-theta) Rx(-phi)`; nine entries are `M` row-major (`rotations`: single-axis, three-axis, negative angles, the matrix form) |
| Nested fills | the transforms compose outer to inner as in OpenMC (`nested`) |
| Lattice in a transformed cell | the lattice frame is the cell frame after the transform (`lattice_xf`) |
| Root universe | the one universe no cell, lattice element or `outer` fills (OpenMC's Python API writes `universe="1"` for its root, so `0` is not assumed) |
| Void | `material="void"` (a cell with no material and no fill is an error) |

To rerun: `gen.py [--out DIR] [MODEL ...]` with that python; it writes `<model>/geometry.xml`,
`materials.xml` and `oracle.json` and leaves nothing else behind (OpenMC runs in a temporary
directory, one subprocess per model). Ray origins are nudged 5e-3 into the model because a start on
a surface is ambiguous in every code.

## Names and materials

Region names follow C9 (`names.rs`): the material `name` of `materials.xml` when non-empty and
unique, else `material_<id>`; `void` for void cells; `ImportOptions::material_names` renames. Without
`materials.xml` every material id a cell uses becomes an unnamed material. Densities, nuclides,
`temperature` and `volume` are dropped and listed in `CsgIr::dropped`.

## Refused (`Unsupported` with element and line)

`hex_lattice`, `dagmc_universe`, other geometry elements, periodic boundaries, a `material` list,
tori with `B != C` or `A <= B`, general `cylinder`/`cone` surfaces, a lattice `outer` with position-dependent cells, unknown surface types, a lattice used as an element or as the
`outer` of another lattice (the IR holds universes there), rotation matrices that are not proper
orthonormal. Invalid: duplicate ids, wrong coefficient or `universes` counts, a cell with both or
neither of `fill` and `material`, translation or rotation on a material cell, bad regions, zero or
several root universes.

## Differences from the draft (`drafts/csg-import.md`)

* Acceptance 17: an oversize pin (r = 1.2 in a pitch 2 lattice) is not an overlap. OpenMC looks a
  point up only inside its own element, so the spill is clipped; the importer does the same
  (`openmc_reader.rs`, `oversize_pins_are_clipped_...`).
* Acceptance 19: the fallback region name is `material_<id>` (C9), not `m<id>`.
* Multi-row lattices and multi-axis rotations are supported, not refused: both are confirmed.

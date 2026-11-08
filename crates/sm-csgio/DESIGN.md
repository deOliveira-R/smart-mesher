# sm-csgio design (W3c-CSG-CORE)

OpenMC is the only target (ruling C6): there is no `mcnp` module and nothing in the core is
shaped after MCNP. The core is frozen after merge; `src/openmc/` (W3c-CSG-OPENMC) fills the IR.

## Pipeline

`openmc::import(geometry, materials) -> CsgIr` (stub) then `ImportedCsg::instantiate` (flatten, no
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
| `outer` universe | placed with the lattice frame origin, fills container minus array box |
| Region names (C9) | material `name` if non-empty, unique, not `void`, not another material's fallback; else `material_<id>`; no material is `void`; override map wins; two materials with one name is an error |
| Boundary flags | vacuum: `Respond(Vacuum)`; reflective: `Reflect` (warning `import.reflective_curved` unless planar); white: `Respond(White)`; others: default vacuum |

The OpenMC reader must confirm (against OpenMC itself, ruling C7) the lattice row order, the
rotation convention and the `outer` frame, and refuse with `Unsupported` what it cannot confirm.

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

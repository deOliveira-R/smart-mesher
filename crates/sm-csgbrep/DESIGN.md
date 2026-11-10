# sm-csgbrep

CSG deck to B-rep import, checks and oracle (W4 tasks CSG-*): reads an OpenMC-style CSG deck and builds the B-rep model, with the checks and the oracle comparison that validate it. It is a layer-3 crate under the server and must not depend on `sm-server` (the test-only dev-dependency in `tests/mesh_roundtrip.rs` is the one exception). The `csgbrep` plugin is registered in `sm_server::standard()` and registers `model.import_openmc`.

## `model.import_openmc` (W4-CSG-CORE)

One op, one history node (`docs/design/w4/csgbrief.md`). `{geometry, materials?, options?}`; both files are `sm_cadio::Source` values (`{sha256}` read through `StepInputs`, or `{inline}`), resolved by a local copy of the 25-line loader with `csg.*` error codes.

Pipeline (all geometry inside one `topo_txn`, so one `commit_topology`):

1. `sm_csgio::openmc::import` reads the deck into the IR.
2. `deck::place` flattens universes and lattices into cells over canonical global surfaces. It mirrors `sm-csgio`'s `instantiate` (sm-csgio's `ImportedCsg` keeps its surface store private, and the core is frozen) but keeps regions as expressions, so there is no DNF term cap. Surfaces are placed with `SurfaceDef::transformed`, planes and cylinders are canonicalised (a cylinder shifted along its axis is the same surface), a quadric that classifies as a plane or cylinder is replaced by it, and everything else is `csg.unsupported` naming the deck surface (sphere, cone, torus, elliptic: W4-BOOL-TOOLS).
3. `partition::build` cuts a start box (the hull of the cell bounds; ten percent larger on every side that is not an axis-aligned deck plane, so a curved extreme is never tangent to a box face) with `sm_boolean::webcut` (`merge: true`, so every cut face is shared and the result is conformal). A piece is cut by a surface only while some cell that can still own it changes value with that surface (three-valued evaluation of the cell expressions on the piece's sign vector); an untouched piece reports its side, so the sign is exact either way. Planes are cut before cylinders (canonical order), so a lattice splits into boxes first.
4. Each finished piece is classified by evaluating the cell expressions: one owner keeps it, none drops it (beyond a one-sided boundary-flagged surface, or outside the bounds: silently; inside: `import.gap` warning with the volume), two or more are an overlap (`csg.overlap` by default; `first_wins` and `report` are options).
5. Pieces of one cell that share a face are united (`boolean::Unite`, breadth-first in ascending order), so the `outer` ring of a lattice is one volume and a cell cut only by planes other cells use stays one volume. A cell that stays disconnected is several volumes in one block.
6. Blocks (`fem.block_add`, numbered from 1 in region-name order, named after the region), sidesets (`fem.sideset_add`: `vacuum` 1, `reflective` 2, `white` 3, on the exterior faces that lie on flagged surfaces), then in one `model_txn` the lattice frames `lat<deck id>` (parents first) and a region tag per volume inside lattice elements, carrying the deck indices (ruling D9).

`TypedOp::apply` of `BlockAdd` and `SidesetAdd` is called with the import's own context (disambiguated from the `Op` blanket impl). The roles they record, `block` and `sideset`, are declared `Many`; the result roles are `deck_volumes` and `deck_bodies` (ascending). `volumes` and `bodies` are not produced.

Diagnostics: `import.partition` (info: kernel cuts, webcut calls, pieces, unions, volumes; work counters, no clock), `import.regions`, `import.gap`, `import.overlap`, `import.reflective_curved`, `import.lattice_rotated`, `import.lattice_multi_placement`.

## Known limits and deviations

- **No element reuse yet (D4).** Every element of a lattice is cut: the kernel cuts are `3 N + (n - 1) + (n - 1) n` for an `n x n` pin lattice of 3 circles (1155 for 17 x 17), not the cuts of one element per distinct universe. Copying a prototype breaks conformality wherever a neighbour is not an identical element (an `outer` ring, a different universe): merge pairs faces whose boundaries differ are only reported. Reuse needs an imprint step after the copies (or cutting the neighbours by the grid planes); a3 decides.
- **Seam turn.** Cylinder tools are cut with their x reference turned by 1 rad about the axis (the surface is unchanged, the canonical form used for ordering and dedup is not). With the default frame the real PWR gap (fuel 0.4096, clad 0.418 to 0.475; clearance 0.0084) fails with `cannot find a point inside a fragment`, the W4-REGIONS-CUT finding; with the turn it imports (`the_real_pwr_gap_imports`, the 17 x 17 test). The cause of the default-frame failure is still open (W4-BOOL-TOOLS).
- **Tangent pin.** A pin tangent to its element box (line contact) is a typed `csg.kernel` error naming the deck surface (`the cut curves branch`), pinned in `tests/import.rs`.
- **Frame names follow the deck lattice id** (`lat<id>`), so renumbering a lattice id moves the state hash; renumbering surfaces, cells, universes and text order does not (`tests/determinism.rs`).
- **Placed-surface dedup is by exact bits** after canonicalisation, not by tolerance; coincident surfaces that differ by round-off are the CHECK task's job.
- A deck whose cells have no finite bounds needs `options.bounds`.

## Quadric classifier

`quadric.rs` (W4-CSG-CLASSIFY): the ten coefficients are divided by a power of two near the largest before the Jacobi sweeps, so tolerances hold at any magnitude. Its `classify_with_tol` rustdoc lists the two fail-safe limits (`k_zero` relative to the translation terms only; radius/offset floor about 3e-5).

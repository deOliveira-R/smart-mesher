# W3c CSG-IMPORT: MCNP and OpenMC CSG importers (O7, Q4)

> Planning draft (W3c), corrected by the planning critique. Changes against the original draft are marked **Critique delta**. `docs/design/w3c/plan.md` wins over this file.
>
> **Feeds plan tasks:** W3c-CSG-CORE (Scope 1-7, firm), W3c-CSG-OPENMC (Scope 8-12, firm, after the reference-deck answer, plan C7), W3c-CSG-MCNP (Scope 13-17, last sub-wave or stretch, gated on plan C7).

Original size: L (about 3500 lines with tests). Splits into CSG-CORE (S/M, first) then CSG-OPENMC (M) and CSG-MCNP (M) in parallel; the B-rep route is a separate W4 task of size L. Original owners: new crate `crates/sm-csgio` (all of this task); orchestrator: workspace Cargo.lock (one XML dependency), `tools/ci.sh` line, `docs/PARITY.md` row, and the CSG-IMAGE decision (Question 2). Depends: sm-rt-api (frozen, no change), sm-geom-api (frozen, no change), sm-rt public exports `AnalyticTrackerBuilder`, `check_overlaps` (no sm-rt edit needed), W3b-INT merged (d7ad022).

**Critique delta (scope).** O7 puts the importers in W3c. CSG-CORE and CSG-OPENMC are firm; CSG-MCNP is the last sub-wave or stretch, gated on the reference decks (Q-REFDECKS, plan C7). If MCNP lands in W3c, TRC, tori (TX/TY/TZ) and `*TR` are reasonable cuts to `Unsupported`.

## Goal
Read an MCNP input deck or an OpenMC `geometry.xml`/`materials.xml` pair and produce an sm-rt analytic CSG world: cells named by material, universes and rectangular lattices instantiated, surfaces and boundary flags carried over, and **overlapping decks rejected with a typed error** (Q4, O17). The result tracks exactly through `AnalyticTracker` and can be compared with the mesh tracker on the same geometry. No cross sections enter (Q2): a cell's only physics datum is its material name.

Out of scope for W3c (waits for W4): CSG to B-rep model volumes for meshing, and a `.smrt` image of a CSG world (see the questions).

## Owner
New crate `crates/sm-csgio` (L3 I/O, next to sm-cadio). It depends on `sm-core`, `sm-geom-api`, `sm-rt-api`, `sm-rt` (for `AnalyticTrackerBuilder`, `check_overlaps`) and one XML crate. It does not touch sm-rt, sm-rt-api, sm-server, sm-cli or sm-protocol. Reason for a new crate and not a module in sm-rt: `sm-rt-capi` links sm-rt, and a deck parser and an XML dependency do not belong in the C ABI library. sm-cadio is B-rep/STEP I/O and sm-cadio already has a dev-only cycle with sm-server.

Ownership notes for the orchestrator: `Cargo.toml` members is `crates/*`, so the crate is picked up by the glob; the Cargo.lock change, the `tools/ci.sh` line and the PARITY row (section B Import: tick "MCNP/OpenMC CSG", with the subset) are orchestrator edits.

**Critique delta (shared files).** The XML crate is `roxmltree` (orchestrator decision, added to Cargo.lock in sub-wave 0; no user question). CSG-CORE pre-declares `pub mod openmc;` and `pub mod mcnp;` in `lib.rs` (empty modules returning `Unsupported`) and the `--format mcnp|openmc` dispatch in `src/bin/smcsg.rs`, so CSG-OPENMC and CSG-MCNP edit only their own directories and never collide in shared files. Dev-dependencies `sm-model` and `sm-testkit` are listed in CSG-CORE's `Cargo.toml` (acceptance 20).

## Findings (what exists, what is missing)
1. **The tracking target already exists and needs no contract change.** `CsgLeafInput { geom: &dyn GeomStore, cells: &[CsgCellDef], bounds, complement }` (`crates/sm-rt-api/src/csg.rs:129`); a cell is `CsgCellDef { region: CsgRegion (a DNF: union of intersections of signed half-spaces), name, volume }` (`csg.rs:105`, `csg.rs:25`). Cells must be pairwise disjoint, with no first-wins rule (`csg.rs:5-10`, ruling Q4). Several cells may carry one name, so "one region per material" (R9) is free.
2. **Surfaces.** `SurfaceDef` has Plane, Cylinder, Cone, Sphere, Torus, Implicit (general quadric), EllipticCylinder, EllipticCone (`crates/sm-geom-api/src/store.rs:35-55`). The analytic kernel handles plane, sphere, cylinder, cone, ring torus and implicit quadrics (`crates/sm-rt/src/analytic/kernel.rs:30-80`). MCNP `SQ`/`GQ` and OpenMC `quadric` map to `Quadric::from_local` or raw coefficients (`crates/sm-geom-api/src/implicit.rs:196-250`), keeping the sign of the written function. This matters: MCNP defines sense by the sign of f as written, so negating all SQ coefficients flips inside and outside. The kernel's cone is a double cone and the bridge adds the apex plane (`bridge.rs:177-185`); MCNP requires the cell to be bounded by the apex plane when a sheet is given, so the importer emits the double cone and validates, not selects, the sheet. Elliptic cones and non-circular tori are not representable: typed `Unsupported`.
3. **Flattening is the only hosting mode for CSG today.** `WorldTracker` has no `Csg` leaf support (`grep Csg crates/sm-rt/src/world.rs` is empty), so `CsgWorld::analytic_tracker` flattens (`crates/sm-rt/src/bridge.rs:582-592`, `flatten` at `bridge.rs:505`, `place_leaf` at `bridge.rs:535` moves surfaces with `SurfaceDef::transformed` and flips the half-space when the sense reverses, dedup by definition through `add_def` at `bridge.rs:141`, cap `MAX_FLAT_CELLS = 2^21`). The importer does the same: universes and lattices are instantiated by recursive placement into **one flat leaf**. A hierarchical `WorldSpec` with CSG leaves would have no tracker. Consequence: a lattice of N elements costs N copies of its universe cells; cold locate is O(cells) (backlog item in notes-w3b3.md, cell-box BVH deferred). Fine for pin cells and small assemblies, and a known limit for 17x17 full cores.
4. **Clipping is the importer's job.** `WorldSpec` has no clip (the bridge refuses clipped instances, `bridge.rs:380`). MCNP and OpenMC universe cells are usually infinite and are clipped by the cell they fill and, in a lattice, by the element box. So every instantiated cell's region is `universe cell AND container region AND element box`, with the DNF products capped. Leaf bounds (required when any cell is unbounded, `crates/sm-rt/src/analytic/leaf.rs:303-318`) are computed from the box of the finite surfaces.
5. **Overlap machinery is reusable as is.** `sm_rt::check_overlaps(&HierInput, n_points, n_rays, seed)` (`crates/sm-rt/src/analytic/check.rs:64`) returns `csg.overlap` (point plus the two local cells), `csg.gap`, `csg.unbounded`; samples depend only on `(seed, leaf hash, index)`, so findings are the same for 1 and N threads. It reports local cell indices; the importer keeps its own `CellSource { deck_cell_id, universe_path, lattice_index, material }` per flat cell (as `CellInfo` does in the bridge) and translates, so the error names deck cells ("cells 1 and 2 overlap at (x, y, z)"). Track-time overlap is `LostCause::Overlap` (`crates/sm-rt-api/src/error.rs:33-36`). The check cannot prove absence (Q4): "rejected" means "found by sampling". Defaults for imports are higher than the op's (100 000 points, 2 000 chords, seed 0), and the options cannot go below the op's defaults.
6. **Graveyard.** `ComplementPolicy::Outside` means leaving the cells leaves the model. An MCNP `imp:n=0` shell, or the region outside an OpenMC vacuum boundary, must not be a tracked cell, but a sampled check without it would report false gaps. Plan: parse once, build two worlds from the same flat IR: a check world that keeps imp:n=0 cells under the reserved name `<outside>` (so gaps and overlaps are judged correctly) and the tracking world without them.
7. **Boundaries.** `BoundarySpec { default, rules: [(BoundarySelector::Entity(surface id), BoundaryDef)] }` (`crates/sm-rt-api/src/boundary.rs:226`); the analytic tracker uses `FaceId == SurfaceId` and applies `Reflect` / `Respond(White|Vacuum)` on named surfaces (`crates/sm-rt/src/analytic/tracker.rs:33-36`). MCNP `*` (reflective) and `+` (white) surface prefixes and OpenMC `boundary="vacuum|reflective|white"` map to rules on the store surface id. `periodic` needs `BoundaryDef::Wrap` with a mate and a transform; refused in W3c. Reflection on non-planar surfaces is only exact on planes (ResponseKind::Reflective doc), so the importer warns for a reflective sphere or cylinder.
8. **Parity and rulings.** PARITY lists "MCNP/OpenMC CSG" as unticked (`docs/PARITY.md:46`); O7 puts it in W3c; Q4 and O17 require rejecting overlapping decks; O12 asks for user-supplied reference shapes. The w3b report carries it as "not scheduled" (w3b-report.md:112).
9. **B-rep for meshing is much larger.** It needs bounded primitives (planes are infinite), region-expression to boolean-tree conversion, imprint and merge, and the model ops (`model.create_*`, subtract, imprint, merge, blocks by material). The tractable W4 shape is "deck to `.smesh` script" for the primitive-only subset (RPP, RCC, SPH, boxes of planes), so history, replay and blocks come for free. Not in this task.

## Scope
Shared core (task CSG-CORE, written first and then frozen for the two format tasks):
1. `src/ir.rs`: format-neutral IR. `Surf { id, def: SurfaceDef, flag: Vacuum|Reflective|White }` (transform already applied), `Expr` (`Half(surf, sign)`, `And`, `Or`, `Not`), `Cell { id, region: Expr, material: Option<MatId>, universe, fill: Fill::{None, Universe{id, xf}, Lattice{id, xf}} }`, `Lattice::Rect { origin_cell_frame, pitch, dims, fill_ids, outer }`. All maps are `BTreeMap`.
2. `src/dnf.rs`: `Expr` to `CsgRegion` by De Morgan and distribution. Drops terms that contain a surface in both senses, removes duplicate half-spaces and duplicate terms, deterministic order. Cap `MAX_TERMS = 4096` per cell, error `ImportError::TooManyTerms { cell, terms }`.
3. `src/instantiate.rs`: depth-first placement of the universe tree with `Transform` composition (cycle detection: `UniverseCycle`; depth cap 64; cell cap `MAX_FLAT_CELLS`), surfaces moved with `SurfaceDef::transformed` and deduplicated by definition in one `MemoryStore`; lattice elements clipped by their box; container regions ANDed in.
   - **Critique delta (canonical order):** after instantiation the flat leaf is put in canonical order: store surfaces sorted by their definition bytes (and renumbered in that order), half-spaces inside a term and terms inside a DNF sorted by (surface index, sign), and flat cells sorted by (name, DNF). `content_hash` is computed on that canonical form, which is what makes acceptance 22 hold.
4. `src/world.rs`: `pub struct ImportedCsg` (owns store, flat cells, `CellSource` table, `BoundarySpec`, bounds, `ImportReport`). Methods: `tracker(&self, &BuildOptions) -> Result<(AnalyticTracker, BuildReport), RtError>`, `region_names()`, `content_hash()` (by value: store definitions, cell DNFs, names, boundaries), `hier_input()` for tests.
5. `src/check.rs`: `ImportOptions { check: CheckOptions { samples (default 100000), rays (default 2000), seed (default 0) }, material_names, max_cells }`, and the rejection: any `csg.overlap` returns `ImportError::Overlap { findings: Vec<OverlapFinding { point, a: CellSource, b: CellSource }> }` (at most 16, as the check reports); gaps and `csg.unbounded` go to `ImportReport::warnings` (see Q-GAPS).
6. `src/error.rs`: `ImportError` (`thiserror`, `#[non_exhaustive]`): `Parse { line, col, msg }`, `Unsupported { line, what }` (always names the card or element and the line), `UnknownSurface`, `UnknownMaterial`, `UnknownUniverse`, `UnknownLattice`, `UniverseCycle`, `TooManyTerms`, `TooManyCells`, `Overlap`, `Build(RtError)`. No `unwrap` on input paths; no panics on hostile bytes (inputs capped at 256 MiB; XML nesting depth and entity expansion limited by the XML crate, depth asserted by a test).
7. `src/bin/smcsg.rs`: `smcsg check <deck> [--format mcnp|openmc] [--samples N --rays N --seed S] [--names 1=fuel,...] [--strict-gaps] [--json]`, and `smcsg info <deck>` (counts of surfaces by kind, cells, materials, universes, lattices, flat cells, bounds, dropped data). Exit code 0 clean, 1 overlap (or gap with `--strict-gaps`), 2 parse or unsupported. This gives a CLI without touching sm-cli or sm-protocol (both frozen or other owners'); a `session`/`rt.*` op for imports is not part of W3c.
   - **Critique delta:** CSG-CORE writes the `mcnp`/`openmc` dispatch with both arms returning `Unsupported` until the format tasks fill their modules.

OpenMC (CSG-OPENMC, `src/openmc/`):
8. XML reader over the chosen XML crate (**Critique delta:** `roxmltree`); accept attributes and child-element forms of `<surface>`, `<cell>`, `<lattice>`.
9. Surfaces: `plane` (A B C D), `x-plane`, `y-plane`, `z-plane`, `x-cylinder`/`y-cylinder`/`z-cylinder`, general `cylinder`, `sphere`, `x-cone`/`y-cone`/`z-cone` (R2), general `cone`, `quadric` (A..K), `x-torus`/`y-torus`/`z-torus` only with b == c (circular, else `Unsupported`). OpenMC senses: negative is f < 0, same as `Sign::Negative`.
10. Region grammar: signed integers, `~` complement, space intersection, `|` union, parentheses; precedence complement over intersection over union. Cells: `id`, `material` (an id or `void`; a list is `Unsupported`), `universe` (default 0), `fill` (universe or lattice id), `translation`, `rotation` (3 angles in degrees or 9 matrix entries; convention pinned by test and Q-REFDECKS), `temperature`/`volume` ignored and reported. Root universe 0 must exist.
11. Rect lattices: `dimension` (2 or 3), `lower_left`, `pitch`, `universes` (row order top to bottom in y, layers bottom to top in z, pinned by test), `outer`. Universes are placed at the element centre (this matches `Lattice::element_center` conventions in sm-rt-api, `world.rs:74`). `hex_lattice`: `Unsupported`.
12. `materials.xml` (optional): ids only; `m<id>` names; `name` used with `use_names` (see Q-NAMES).

MCNP (CSG-MCNP, `src/mcnp/`):
13. Card reader: title, three blocks separated by blank lines, `c` comment lines, `$` inline comments, 5-space and `&` continuations, case-insensitive, `n R` repeats, `message:` block skipped. Data cards ignored but listed in the report: `mode`, `nps`, `kcode`, `sdef`, tallies, `imp` (read), `m<n>` (read for ids only), `tr<n>`/`*tr<n>` (read), `vol`, `tmp`, `dm`, `print`, and others.
14. Surfaces: P, PX, PY, PZ (sense: positive is Ax+By+Cz-D > 0), SO, S, SX, SY, SZ, C/X, C/Y, C/Z, CX, CY, CZ, K/X, K/Y, K/Z, KX, KY, KZ (double cone plus apex-plane validation), SQ, GQ, TX, TY, TZ (circular only); prefixes `*` (reflective) and `+` (white). Surface-card transformation `n` (TRn): displacement plus rotation (3-vector displacement with 9 matrix entries; partial matrix forms refused; `m = ±1` handled); `*TR` means angles are direction-cosine angles in degrees. Periodic surfaces (negative id) refused. **Critique delta:** TX/TY/TZ and `*TR` may be cut to `Unsupported` if MCNP lands in W3c.
15. Macrobodies: RPP, SPH, RCC, BOX (orthogonal edge vectors), TRC. Each expands to planes and quadrics numbered `n.1...n.k` so cell cards can reference faces (`-n.2`); `-n` is the intersection of the faces' negative sides, `n` the union (De Morgan). RHP/HEX, WED, ELL, REC, ARB are `Unsupported` with the card line. **Critique delta:** TRC may be cut to `Unsupported` likewise.
16. Cells: `j m d geom` and `j 0 geom`, operators space, `:`, `#n`, `#(...)`, `(...)`; parameters `u`, `fill n`, `fill n (TRn | dx dy dz | 12 numbers)`, `*fill`, `trcl`, `*trcl`, `lat=1` (axis-aligned plane pairs only: pitch from plane separation; element [i,j,k] direction by surface order, pinned by test and Q-REFDECKS) with `fill i1:i2 j1:j2 k1:k2` arrays (index i fastest, `nR` repeats). The universe of a lattice element sits at the element [0,0,0] frame; the importer wraps by a translation of minus the centre of that element (MCNP's universe origin is the frame of the lattice cell, sm-rt's is the element centre). `imp:n=0` (or all particles zero) marks the exterior (finding 6); `lat=2`, `like n but` (see Q-SUBSET), `trcl` on lattice cells: `Unsupported`.
17. Materials: cell material number n gives region name `m<n>`; `0` gives `void`; negative density sign and density values are dropped and reported.

**Critique delta (conventions policy, applies to 10, 11, 14, 16):** any convention not confirmed by the manual or by a user deck (Q-REFDECKS) returns `Unsupported`, and the test asserts the `Unsupported`. A guess is never pinned.

Not in scope: hierarchical (non-flat) output, hex lattices, periodic and Wrap boundaries, B-rep volumes, `.smrt` image, session or protocol ops, tallies, sources, density or composition.

## Contract changes / ACRs
None. Everything needed is public: `CsgCellDef`, `CsgRegion`, `CsgLeafInput`, `HierInput`, `WorldSpec`, `BoundarySpec`, `MemoryStore`, `SurfaceDef::transformed`, `Quadric`, `AnalyticTrackerBuilder::build_concrete`, `check_overlaps` (all re-exported at crate root, `crates/sm-rt/src/lib.rs:44-47`). `CsgCellDef` is `#[non_exhaustive]` but has `new`/`with_volume`, which is enough. If the user picks a CSG section in `.smrt` (Q-OUTPUT), that is an additive sm-rt task (CSG-IMAGE), not an ACR on a frozen crate unless `CacheImage` section tags need a new tag (additive).

## Acceptance
Decks are inline strings in `crates/sm-csgio/tests/`. Tolerances are 1e-12 absolute on chord lengths unless stated. Chords are traced with `Tracker::trace`/`step` from the built `AnalyticTracker` along axis-parallel rays, summed per region name.

**Critique delta (task split of the acceptance).** CSG-CORE: 3, 4 (with a hand-built IR of Deck A), 5, 12 (core errors: cycle, unknown universe, `TooManyTerms`, `max_cells`), 21, 22, 23 (core parts). CSG-OPENMC: 13-19, 20 (OpenMC half), 21-23 for its decks. CSG-MCNP: 1-12, 20 (MCNP half), 21-23 for its decks. Acceptance 13's MCNP-vs-OpenMC pairing runs in whichever format task merges second.

**Deck A (MCNP, sphere in a box).**
```
sphere in box
1 1 -10.0 -1                          imp:n=1
2 2 -1.0   1 2 -3 4 -5 6 -7           imp:n=1
3 0        -2:3:-4:5:-6:7             imp:n=0

1 SO 1.0
2 PX -2
3 PX 2
4 PY -2
5 PY 2
6 PZ -2
7 PZ 2

m1 92235 1
m2 1001 2 8016 1
```
1. Deck A parses and has no findings; regions are `m1`, `m2` (graveyard is not a tracked cell). Along +x at y=0.6, z=0 from x=-2: `m1` chord = 2 sqrt(1 - 0.36) = 1.6, `m2` chord = 4 - 1.6 = 2.4. At y=0, `m1` = 2, `m2` = 2. At y=1.5 the chord is all `m2` = 4. Leaving at x=2 ends as a boundary exit (vacuum), not `Lost`.
2. Deck A written with cell 2 as `2 2 -1.0 -2:3:... ` replaced by the complement form `2 2 -1.0 #1 2 -3 4 -5 6 -7` gives an identical `content_hash` of the flat leaf apart from nothing: the two decks produce the same chords (acceptance 1 values) and the same cell regions after DNF. (Tests the `#n` operator.)
   - **Critique delta (restated):** Deck A with cell 2 written as `2 2 -1.0 #1 2 -3 4 -5 6 -7` (the `#1` form) and Deck A as written (the explicit form `1 2 -3 4 -5 6 -7`) give, for every deck cell, the same DNF after canonicalisation (Scope 3 delta), and the same chords as acceptance 1 to 1e-12.
3. Volume cross-check (deterministic grid, not exact): chords of a 400x400 midpoint grid of rays along z over the box give `m1` volume within 1e-3 relative of 4 pi / 3 and `m2` within 1e-3 of 64 - 4 pi / 3. State the tolerance in the test; the chord check in item 1 is the exact test.
4. **Overlap rejection.** Deck A with cell 2 changed to `2 2 -1.0 2 -3 4 -5 6 -7` (forgot `1`) is rejected: `ImportError::Overlap`, at least one finding whose point has `|p| < 1` (inside the lens, which is the whole sphere) and whose two cells are deck cells 1 and 2. Same findings for `RAYON_NUM_THREADS=1` and default. The sphere is 6.5 percent of the box, so 1e5 samples find it with certainty at seed 0.
5. Gap is a warning: Deck A with the `+z` plane of cell 2 moved to `PZ 1` and cell 3 unchanged leaves the slab 1 < z < 2 in no cell. Import succeeds with an `import.gap` warning whose point is in that slab (not rejected, see Q-GAPS); `--strict-gaps` returns exit code 1.
6. Sense by surface kind: for each of P, PX, SO, S, SX, C/Z, CZ, K/Z (with a bounding apex plane), SQ, GQ and circular TZ, a point inside and a point outside the primitive is classified with the right sign through the built tracker (`locate`). `SQ` with all coefficients negated has inside and outside swapped (f as written).
7. Torus: `TZ 0 0 0 2.0 0.5 0.5` as the only fuel cell inside a box: along +x at y=0, z=0 from x=-3, `m1` chords are [1.5, 2.5] and [-2.5, -1.5], each of length 1.0; along +x at z=0.5 (tangent to the tube top) there is no event (tangent roots give no event, as for the analytic kernel). **Critique delta:** if tori are cut (Scope 14 delta), this item asserts `Unsupported` naming the `TZ` card and line.
8. Macrobodies: `1 RPP -1 1 -2 2 -3 3` as the fuel cell in a bigger RPP: chord along y at x=0, z=0 is 4.0; the face reference `-1.1` ... `1.6` expand to 6 planes, a cell using only `1.2` (the `+x` face; verify numbering against the manual, Q-REFDECKS) is bounded accordingly; RCC `0 0 0 0 0 2 0.5`: chord along x at z=1, y=0 is 1.0, along z at r=0.3 is 2.0. SPH and TRC chord at the axis equal 2R and the analytic frustum length. `RHP` returns `Unsupported` naming the card and line.
   - **Critique delta:** if the macrobody face numbering is not confirmed by the manual or a user deck, a cell card that references a face (`n.m`) returns `Unsupported` and the test asserts that; whole-body references (`-n`, `n`) stay tested by chords. If TRC is cut, its chord check becomes an `Unsupported` assertion.
9. Transformations: `*TR1 0 0 3 ...` is not used for single-axis checks. `5 1 PZ 0` with `TR1 0 0 3` is the plane z = 3 (chord of a cell bounded by it is shifted by exactly 3). A `TR2` that is a 90 degree rotation about z (`*TR2 0 0 0 0 90 ... `, direction cosines in degrees, pinned in Q-REFDECKS) turns a `PX 1` into the plane y = 1 (or -1, as pinned) and `Unsupported` is returned for a partial matrix form.
   - **Critique delta:** "as pinned" means confirmed by the manual or a user deck; otherwise any rotation in `TRn`/`*TRn` returns `Unsupported` (pure displacement stays supported) and the test asserts it.
10. **Universe, fill and lattice (Deck C).** **Critique delta:** the original deck was wrong as written (universe 3 had no cell, and the `fill=` array had a stray `=` form). Corrected deck:
```
lattice
10 1 -10  -1                 u=1 imp:n=1
11 2 -1    1                 u=1 imp:n=1
12 2 -1   -12                u=3 imp:n=1
13 0       2 -3 4 -5 6 -7    u=2 lat=1 imp:n=1
           fill 0:1 0:0 0:0  1 3
14 0       -8 9 -10 11 6 -7  fill 2 imp:n=1
15 0       8:-9:10:-11:-6:7  imp:n=0

1 CZ 0.4
2 PX -1
3 PX 1
4 PY -1
5 PY 1
6 PZ -1
7 PZ 1
8 PX 3
9 PX -1
10 PY 1
11 PY -1
12 SO 50

m1 92235 1
m2 1001 2 8016 1
```
   Universe 1 is the pin (fuel `-1`, moderator `1`, both clipped by the element). Universe 3 is a single moderator cell (`-12`, a sphere that covers any element, clipped to the element box), so the `3` in the fill array is the pure-moderator element. Cell 13 is the lattice cell (element [0,0,0] is x in [-1, 1], y in [-1, 1], z in [-1, 1]; the `fill` continuation line is indented 5+ spaces); cell 14 is the container (x in [-1, 3]) filled with the lattice universe 2.
   Expected: lattice element [0,0,0] is the cell itself (x in [-1, 1], pin at the origin), element [1,0,0] is x in [1, 3] filled with universe 3 (moderator only). Along +x at y=0.2 from x=-1: `m1` chord = 2 sqrt(0.16 - 0.04) = 0.69282032302755 (only in the first element) and `m2` = 4 - that. Swapping the fill array to `3 1` moves the pin to x in [1, 3], so the first element is moderator: this pins the +i direction. All cells fit the flat leaf; no overlap finding; the element planes are shared (store holds 11 plane/cylinder surfaces plus no duplicates).
   - **Critique delta:** the surface count is restated as "no two store surfaces have equal definitions" (surfaces 9, 10, 11 coincide with 2, 5, 4 by definition and are deduplicated; the count depends on whether unused clipped surfaces are pruned, which the test does not pin). If the lattice index direction is not confirmed (Q-REFDECKS), `lat=1` returns `Unsupported` and the chord part of this item waits for the confirmation.
11. `trcl`/`fill` displacement: `fill=2 (2 0 0)` shifts the filling universe by +2 in x (universe origin at (2,0,0) in the container frame): the pin of Deck C's cell 10 appears at x = 2. Pinned together with Q-REFDECKS; if unconfirmed, the importer returns `Unsupported` for any FILL transformation other than a pure translation.
12. Rejections, each a typed error naming the card and the line: `LAT=2`, `RHP`, `ARB`, periodic surface (negative id), `like n but` (unless Q-SUBSET adds it), a cyclic universe (`fill` of itself), `fill` of an unknown universe, a cell with an unknown surface (`UnknownSurface`), a `#7` complement of a missing cell, a deck whose complement DNF exceeds `MAX_TERMS` (a cell `#(a:b:c:...)` of 13 two-term cells multiplied out), a flat world over `max_cells`. Parse errors give line and column. Fuzz-style test: truncated and random-byte inputs never panic (a fixed-seed corpus of 500 mutations of Deck A and Deck C).

**Deck O (OpenMC, same geometry as Deck A).**
```xml
<geometry>
  <cell id="1" material="1" region="-1" universe="0"/>
  <cell id="2" material="2" region="1 2 -3 4 -5 6 -7" universe="0"/>
  <surface id="1" type="sphere" coeffs="0 0 0 1"/>
  <surface id="2" type="x-plane" coeffs="-2" boundary="vacuum"/>
  <surface id="3" type="x-plane" coeffs="2"  boundary="vacuum"/>
  <surface id="4" type="y-plane" coeffs="-2" boundary="vacuum"/>
  <surface id="5" type="y-plane" coeffs="2"  boundary="vacuum"/>
  <surface id="6" type="z-plane" coeffs="-2" boundary="vacuum"/>
  <surface id="7" type="z-plane" coeffs="2"  boundary="vacuum"/>
</geometry>
```
13. Deck O has the same chords as Deck A (acceptance 1 values, equal to 1e-12), with regions `m1`, `m2`; the chords of the MCNP and OpenMC imports of the same geometry agree pairwise to 1e-12 on 1e3 fixed-seed rays (both via `ImportedCsg::tracker`).
14. Region grammar: `region="~(-1) 2 -3 ..."`, `region="(-1 | 8) ..."`, nested parentheses and `~` over a union give the same chords as the hand-expanded form; precedence is checked with a case where wrong precedence changes a chord (`-1 | 2 -3` is `-1` or (`2` and `-3`)).
15. Overlap rejection (OpenMC): cell 2 region changed to `2 -3 4 -5 6 -7` (no `1`) gives `ImportError::Overlap` with cells 1 and 2 and a point with `|p| < 1`, the same as acceptance 4; identical for 1 and N threads.
16. Boundaries: surfaces with `boundary="reflective"` on x-planes become `Reflect`/`Respond(Reflective)` rules on those surface ids; a ray along +x from the centre reflects exactly at x=2 and re-crosses the sphere (chord sum 2 on every pass); `white` gives `Respond(White)`; a reflective sphere imports with a warning (reflection is exact only on planes). `periodic` returns `Unsupported`.
17. Universe, rect lattice, `outer`, `rotation`: a 2x1 `rect_lattice` (`dimension 2 1`, `lower_left -1 -1`, `pitch 2 2`) whose `universes` text `1 2` holds a pin universe 1 (cylinder r=0.4, moderator) and a moderator-only universe 2 has the pin in the first element (x in [-1, 1]); changing the text to `2 1` moves it to the second. For a 2x2 lattice with a pin in only the top-left, the row order is pinned (first row is highest y; confirm against the manual or a user deck, Q-REFDECKS). `rotation="0 0 90"` on a filled cell turns a pin offset (0.5, 0) into (0, 0.5) (single-axis, convention-insensitive); a three-axis rotation is pinned against a hand-computed matrix. `outer` fills outside the array inside the container. Universe cells that extend past an element are clipped at the element wall: a pin universe whose moderator cell is infinite produces no overlap with its neighbour, and a deliberately oversize fuel cylinder (r=1.2 in a pitch 2 lattice) is rejected as overlap (neighbouring cylinders intersect).
   - **Critique delta:** if the row order or the three-axis rotation order is not confirmed by the manual or a user deck, multi-row lattices and multi-axis rotations return `Unsupported` and the test asserts it (1-row lattices and single-axis rotations stay supported).
18. `hex_lattice`, `dagmc_universe`, a `material` list and `periodic` surfaces each return `Unsupported` with the element name.
19. Materials: with `materials.xml` absent, cells use `m<id>`; with `material_names` {1: "fuel", 2: "water"} regions are `fuel`, `water`; an unknown material id gives `UnknownMaterial`; `material="void"` gives region `void`.

**Cross checks with the Model bridge and determinism.**
20. Agreement with the bridged model: the pin cell and a 3x3 lattice from `sm_testkit::lattice` (`pin_assembly`) via `csg_world` versus the same geometry written as an OpenMC deck and an MCNP deck: region chord tables on 1e4 fixed-seed rays agree to 1e-11 relative, region names aside.
   - **Critique delta:** the path is `sm_testkit::topo::assembly::pin_assembly` (not `sm_testkit::lattice`). The test needs `sm-model` and `sm-testkit` as dev-dependencies of sm-csgio (declared by CSG-CORE).
21. Determinism: import each deck twice, with `RAYON_NUM_THREADS=1` and default; `ImportedCsg::content_hash`, the finding list of the check, and the chord tables are bit-identical. No `HashMap` iteration reaches output (`BTreeMap` everywhere), seed is explicit.
22. Property test (`sm_testkit::strategies::config(n)`): a random deck generator produces nested boxes and cylinders with random permutation of cell order and of surface ids (renumbering); the flat leaf `content_hash` is invariant under surface renumbering and under reordering of independent cells' text, and chords are unchanged.
   - **Critique delta:** this holds because of the canonical order in Scope 3 (store surfaces sorted by definition bytes, flat cells by (name, DNF)); the test also asserts chords unchanged.
23. Docs and style: rustdoc on every public item; `cargo clippy -p sm-csgio -- -D warnings`; `unsafe` forbidden (`#![forbid(unsafe_code)]`); `crates/sm-csgio/DESIGN.md` with the supported subset table, the convention table (lattice index direction, FILL transform, rotation order, row order) and the refusal list; `smcsg` tests on exit codes (0, 1, 2) and a `--json` golden for Deck A's overlap variant.

Public decks: the MCNP and OpenMC benchmark decks of public suites (ICSBEP, OpenMC's own examples) are not committed unless their licence is checked first; if the user supplies decks (Q-REFDECKS) they are used locally and referenced by hash only (like `--model` STEP files, Q9).

## Gates / dependencies
- Needs nothing unmerged: sm-rt analytic leaf, `check_overlaps` and the bridge are in main (W3b-3, `40c36c0`).
- Order: CSG-CORE (item 1-7, with an OpenMC-free test harness and Deck A's IR built by hand) then CSG-OPENMC and CSG-MCNP in parallel in separate directories (`src/openmc/`, `src/mcnp/`); the core is frozen after CSG-CORE merges. Q-REFDECKS must be answered before the two format tasks pin conventions.
  - **Critique delta:** CSG-MCNP is the last sub-wave or stretch; CSG-OPENMC goes first.
- Definition of done per CLAUDE.md: `cargo build`, `cargo test -p sm-csgio`, `cargo clippy -p sm-csgio -- -D warnings`, determinism test, property tests with `sm_testkit::strategies::config(n)`, rustdoc on all public items, tests against analytic results (all of the above).
- Orchestrator: Cargo.lock for the XML crate, `tools/ci.sh` line, PARITY row, optional CSG-IMAGE if Q-OUTPUT is answered otherwise.

## Hash impact
None. A new crate with no ops, no protocol or MCP surface, no layer, no registration in `standard()`: no state hash, no golden, no snapshot and no existing milestone hash (W2 milestone/fork, w3a A/B, W3b `h_model`/`h_auto`/`h_mesh`) can move. No change to pave, tri, sweep or any scheme output. The new `ImportedCsg::content_hash` is a new, crate-local hash. If CSG-IMAGE is later added, `.smrt` of mesh worlds must stay byte-identical (additive section tag only).

## Size
L overall. CSG-CORE S/M (about 1000 lines), CSG-OPENMC M (about 1000), CSG-MCNP M (about 1500, the card reader, macrobodies and lattice conventions dominate). Tests add about as much again.

## Risks
1. **Conventions are easy to get wrong silently**: MCNP lattice index direction from surface order, FILL transformation direction (universe points u appear at u + O), `*TR` angle form, and the MCNP rule that the universe origin is the lattice cell frame (sm-rt uses the element centre); OpenMC rect `universes` row order and `rotation` angle order. Mitigation: single-axis asymmetric decks with analytic chords, a refuse-when-unsure policy (`Unsupported` instead of a guess), and user reference decks (Q-REFDECKS).
2. **Sampling cannot prove disjointness.** A thin overlap sliver can be missed (Q4 accepts this); the default sample counts are higher than the op's, the report states "no overlap found in N points and M chords at seed S", and `smcsg check` can raise them. Do not write a test that pins a miss (seed-dependent).
3. **DNF blow-up** from `#n` and from macrobody complements; capped at `MAX_TERMS`, a typed error says which cell.
4. **Flat leaf size and speed.** Lattice copies are materialised; cold locate is O(cells) (backlog); a 17x17 assembly is fine, a full core with fine pin structure may be slow. Hierarchical hosting of CSG leaves needs a `WorldTracker` change (sm-rt, not in this task).
5. **Double cone and reflective curved surfaces.** MCNP sheet selection is validated, not enforced; reflection on non-planar surfaces is inexact (warning).
6. **Coincident or tangent surfaces** in real decks (pin touching the cell wall) are normal in MCNP and rely on the kernel's tolerance; tested with a tangent cylinder (r = 1 in a pitch-2 element) where the chord at the tangency is zero and no overlap is reported.
7. **External XML dependency** (Q-XML) adds a supply-chain item; mitigated by pinning and by the hostile-input tests.
8. **Scope creep toward B-rep.** The importer must stay a world builder; B-rep volumes come from a W4 script generator on top of `ImportedCsg`'s IR, which is why the IR keeps surfaces, expressions and cell sources, not only the flat leaf.

## Questions
- Q-SUBSET Which input subset should W3c support? Proposed MCNP: cells with `#n`, `#(...)`, `:` union and `n.m` face references; surfaces P/PX/PY/PZ, SO/S/SX/SY/SZ, C/X C/Y C/Z, CX/CY/CZ, K/X..KZ, SQ, GQ, circular TX/TY/TZ; macrobodies RPP, SPH, RCC, BOX, TRC (RHP, WED, ELL, REC, ARB are refused with a typed error); `u`, `fill` (with TRn or a displacement), `trcl`, `lat=1` rectangular lattices, `TRn`/`*TRn`, `imp`, `m<n>`; `nR` repeats in fill arrays. Proposed OpenMC: geometry.xml and materials.xml, all analytic surface types incl. circular tori and general quadric, region strings with `~ | ( )`, `fill`, `translation`, `rotation`, universes, rect lattices with `outer`. Hex lattices (LAT=2, `hex_lattice`), periodic boundaries, `dagmc_universe`, distributed materials and `like n but` are refused as `Unsupported`. Is that the right subset, or do your decks need something on the refused list? (rec: Accept the proposed subset. Add `like n but` (textual copy, about 60 lines) if your decks use it. Keep hex lattices out: sm-rt has no hex lattice tracking yet (Q3 stretch, not scheduled).) → plan C6.
- Q-OUTPUT The importers produce a world for tracking only (analytic CSG, flattened to one leaf). The `.smrt` handover file (O1) cannot hold it: `AnalyticTracker` has no cache image (`crates/sm-rt/src/analytic/tracker.rs:37`) and the C ABI opens only `.smrt` files. So in W3c an imported deck can be tracked and overlap-checked in Rust and through a CLI, not opened by a C/Fortran transport code. Do you want a CSG section in `.smrt` (task CSG-IMAGE, sm-rt plus sm-rt-capi, size M-L) in W3c, or is in-process use enough for now? B-rep volumes for meshing are W4 either way. (rec: In-process plus CLI only in W3c. Your transport codes already read MCNP/OpenMC natively, so the immediate value is overlap checking and the analytic-versus-mesh benchmark twin. Schedule CSG-IMAGE for W4 only if you want to hand CSG decks to the transport code through sm-rt. Put the B-rep route in W4 as 'deck to .smesh script' (primitive-only first), reusing the existing model ops.) → plan C8.
- Q-NAMES Region (material) names. Proposed: `m<id>` for both formats, void cells named `void`, one region per material (R9), user override with `ImportOptions::material_names` and `--names 1=fuel,2=water`. OpenMC `name=` attributes would be used only with an opt-in flag, because they may be empty or duplicated. Is that right? (rec: Yes. Id-based names pair deterministically with the transport code's own cross sections (Q2). Density and composition are dropped and listed in the report only as 'dropped'.) → plan C9 (merged with Q-GAPS).
- Q-XML May sm-csgio depend on an external XML crate (`roxmltree`)? **Critique delta:** a Cargo.lock decision for the orchestrator: take roxmltree, pinned to an exact version at least two weeks old. Dropped from the user list.
- Q-REFDECKS The index and transform conventions are the main correctness risk. Can you supply (O12-style) two or three small decks of your own (one MCNP lattice, one OpenMC lattice with rotation) with positions you know independently? (rec: Yes, ask for them before CSG-MCNP and CSG-OPENMC start. Until then any convention not confirmed by the manual or your decks is refused with `Unsupported` instead of guessed.) → plan C7.
- Q-GAPS Overlaps reject the deck (Q4). What about gaps? (rec: Report gaps as warnings with the point, never reject. `smcsg check --strict-gaps` turns them into exit code 1.) → plan C9.

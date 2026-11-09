# W4 CSGBREP: OpenMC CSG to B-rep volumes, quadrics, lattices, overlaps, and the mesh round trip

> Planning draft (W4). Area: csgbrep. Read-only study at d2f4803. Line numbers are against that commit. The Opus planning critique corrected this draft. Each change from the Sonnet draft is marked **Critique delta**.

## Goal

Take an OpenMC `geometry.xml` (+ `materials.xml`) and turn it into B-rep volumes in the model. The user can then mesh it (polyhedral for the transport code, `.smrt` for sm-rt) with the normal history, fork and replay. Then prove the chain `deck -> B-rep -> mesh -> .smrt -> tracking` against OpenMC itself.

**Critique delta (goal):** the chain must end in the user's two products, so it is `deck -> B-rep -> mesh -> {polyhedral .smsh/VTU via mesh.to_poly, .smrt}`. No acceptance item in the draft exported a polyhedral mesh. CSGBREP-ORACLE now has a row for it.

## Findings (what is true today, with file:line)

1. **The importer stops at a tracker.** sm-csgio flattens a deck to analytic CSG cells and checks it by sampling (`crates/sm-csgio/src/world.rs:197` `import`, `:265` `check`). PARITY says it plainly: "The result is a tracker, not a B-rep: B-rep import of CSG is open" (`docs/PARITY.md:46`), and `CSG to B-rep` is an open box (`docs/PARITY.md:47`). Ruling C8 sent it to W4 (`docs/design/w3c/rulings.md:16`; `docs/design/w3c/w3c-report.md:136`).
2. **The IR already holds what a B-rep builder needs.** Surfaces are kernel `SurfaceDef`s with boundary flags. Cell regions are `Expr` trees (`ir.rs:115`). Fills are a universe or a lattice with a `Transform` (`ir.rs:144`). Lattices are rectangular only (`ir.rs:190`). The W3c draft kept the IR (not only the flat leaf) on purpose for this task (`docs/design/w3c/drafts/csg-import.md:189`, risk 8). It also named the tractable shape: a script generator over the IR (`:32`, item 9).
   **Critique delta:** item 9 is at `csg-import.md:32`, not `:31`. sm-csgio's own DESIGN also says "The core is frozen after merge" (`crates/sm-csgio/DESIGN.md:3-4`). So any IR change (hex lattices) goes through the orchestrator, even though sm-csgio is not an `*-api` crate.
3. **Reader coverage.** Planes, x/y/z cylinders, sphere, x/y/z cones (double), quadric, and circular tori (`crates/sm-csgio/src/openmc/surfaces.rs:44-98`; elliptic tori refused). Refused with a typed error: `hex_lattice`, `dagmc_universe`, periodic boundaries, a structured lattice `outer`, a lattice as an element of another lattice (`crates/sm-csgio/DESIGN.md:119-126`). The conventions (sense, lattice row order, rotation transpose, `outer`) are confirmed against OpenMC 0.16.0 and committed as `tests/fixtures/openmc/*/oracle.json` (DESIGN.md:79-110).
   **Critique delta:** the surface arms run `surfaces.rs:44-98` (`:88-98` is the torus arm only). This matters for the task: the reader refuses OpenMC's general `cylinder` and `cone` and tells the user to write a `quadric` (`surfaces.rs:78`; DESIGN.md:95). So every tilted or off-axis cylinder or cone in a real deck arrives as an `Implicit` quadric.
4. **Kernel surface menu.** `SurfaceDef` has Plane, Cylinder, Cone, Sphere, Torus, Nurbs, Implicit (general quadric, torus quartic), EllipticCylinder, EllipticCone (`crates/sm-geom-api/src/store.rs:35-56`). The bridge in the other direction (model to CSG) already maps planes, spheres, cylinders, tori, implicit quadrics and one cone nappe. It states its assumption: a volume equals the intersection of its face half-spaces (`crates/sm-rt/src/bridge.rs:1-30`).
   **Critique delta (missed, decisive for D3):** `Implicit` has no parametrization ("An implicit-only surface ... No parametrization", `store.rs:48-49`). `ParametricSurface` is implemented only for Plane, Cylinder, Cone, Sphere, Torus, NURBS and the two elliptic types (`crates/sm-geom-api/src/analytic.rs:161-414`, `elliptic.rs:97,161`, `nurbs.rs:509`). Every surface mesher works through pcurves and `(u, v)`, so a B-rep face on an `Implicit` carrier cannot be meshed. The kernel has no quadric classifier: `implicit.rs:196-250` has the constructors `from_local`, `plane`, `sphere`, `cylinder` and `cone`, but no inverse. Also missed: a volume may carry `Carrier::Csg(CsgCell)` (`crates/sm-topo-api/src/types.rs:278`), and the bridge takes such a volume "as given" (`bridge.rs:8-9`).
5. **Op menu to build volumes.** Primitives: brick, cylinder (z axis only, `crates/sm-model/src/prim.rs:113-123`), sphere, frustum, prism, pyramid, torus, sphere part. Transforms: move, rotate, scale, reflect, move_to, rotate_about and `model.copy` (`crates/sm-model/src/lib.rs:35-50`). Booleans: unite, subtract, intersect, imprint, merge, webcut plane / surface / cylinder (`crates/sm-boolean/src/lib.rs:57-65`). `WebcutSurface` takes a planar face only (struct `crates/sm-boolean/src/ops.rs:529-546`, refusal in `apply` `:548-562`).
   **Critique delta:** the draft said no op bounds a body by "an offset or tilted cylinder". That is wrong: `model.webcut_cylinder` takes any `axis` and `center` (`ops.rs:566-600`). The library `webcut` already takes `CutTool::Surface { surface: SurfaceDef, window }` (`crates/sm-boolean/src/webcut.rs:49-57`), but it refuses every tool except Plane and Cylinder (`webcut.rs:131-132`). So "a half-space of any surface, bounded by a window" is a webcut that already exists in shape and only needs more tool types. That is a sm-boolean job, not a new sm-model op. Webcut with a body tool is an open PARITY box (`docs/PARITY.md:36`), and `WebcutOptions::consume_tool` is already reserved for it (`webcut.rs:69-70`).
6. **Boolean limits that CSG decks hit.** Tangent contact along a line or at points is `TangentContact` and is rolled back (`crates/sm-boolean/DESIGN.md:84-92`). Near-tangent curved operands with clearance below about 1e-2 are rejected with `ResultInvalid` (`:96-102`). Pin-in-pin with equal radii, a fuel/clad gap of zero, and cylinders tangent to lattice planes are everyday CSG.
   **Critique delta:** the draft said "W3c already met this class on the round pin (PARITY `:72`)". PARITY:72 is the `mesh.auto` row and says nothing about tangency, so the claim is removed. The csg-import draft did test a tangent pin (r = 1 in a pitch-2 element), but for the *tracker* only (`csg-import.md:187`, risk 6).
7. **Import has a precedent.** `model.import_step` takes a content-addressed `Source` (`{sha256}` or `{inline}`), so replay needs no file. It builds its cells inside one `apply` with an `IMPL_VERSION` (`crates/sm-cadio/src/op.rs:72-140`). A CSG import op can follow it.
8. **Lattices as copies work.** A 3x3 lattice and the 3x3 pin cell were built by two `model.copy` steps, conformal and checked (`docs/PARITY.md:33`, milestone_w3b M1, `milestone_w3c_pin`). PARITY `:72` shows that a through pin is all hex via sweeps of pave sources and an enclosed pin is all tet. O-grid hex is deferred (`docs/design/w3c/w3c-report.md:133`).
   **Critique delta (missed):** `model.copy` is a deep copy, cell by cell (`crates/sm-model/src/copy.rs:1-10`, `MAX_COPIES = 4096` at `:25`), so volumes and topology are O(elements). The frozen model API also has real instancing: `Prototype`, and `Lattice` with `Layout::Cartesian` and `Layout::Hex { rings, pitch, orient, nz }` (`crates/sm-model-api/src/instancing.rs:118-135`), plus `add_prototype` / `add_lattice` (`:661,675`). The mesh side has an `InstancedView` with flatten and weld (`crates/sm-mesh-api/src/instance.rs:1-13`). But no op creates any of them (no `const NAME` mentions lattice or prototype anywhere), and sm-boolean refuses instanced volumes (`crates/sm-boolean/src/ops.rs:40-52`).
9. **Gap/overlap today is sampled.** The defaults are 100 000 points and 2 000 chords, and the DESIGN says "sampling cannot prove disjointness; a thin overlap sliver can be missed" (`crates/sm-csgio/DESIGN.md:25-34,58`). A B-rep has exact volumes, so it can do better.
10. **Round trip pieces exist.** `.smrt` export takes region names from blocks and warns when volumes are in no block (`w3c-report.md:43-44`; PARITY `:103`). Regions are one per block (`docs/design/w3/backlog.md:102`). The oracle data and a model-vs-deck cross test exist (`crates/sm-csgio/tests/openmc_cross.rs:66`, `the_model_bridge_and_the_openmc_deck_agree_on_the_pin_assembly`).
    **Critique delta:** the oracle chord tolerance is 2e-9 (`crates/sm-csgio/tests/openmc_oracle.rs:88`). There are 11 committed fixtures. They already cover quadric, x/y/z cones and x/y/z tori (`surfaces`, 34 cells) and rotated or translated fills (`rotations`, `translated`, `lattice_xf`, `nested`).
11. **Backlog items that touch this area** (take with W4): `gen.py` `openmc.reset_auto_ids()` and SIGTERM exit (`docs/design/w3/backlog.md:138,149`); analytic stepping and cold locate O(cells) (`:105`); hex lattices are "explicit user request only" (`:112`); per-volume `.smrt` region tag, trigger "first request" (`:102`); merge face-pair scan (`:95`).
12. **Critique delta (new finding): sub-op expansion is not available outside sm-op-api.** `ApplyCtx::apply_nested` is `pub(crate)` (`crates/sm-op-api/src/op.rs:619-648`), and only `core.batch` uses it. `core.batch` has no control flow and refuses nesting (`crates/sm-op-api/src/batch.rs:13-23`). So an op in a new crate cannot "expand to a list of ordinary ops in its `apply`" without a change to a frozen contract.

## Design

### D1. Where it lives: a new crate, one import op built like `model.import_step`

New crate `crates/sm-csgbrep` (L3). It depends on the sm-csgio IR, sm-boolean, sm-model and sm-layer-api, and not the other way round, so sm-csgio keeps no model dependency. ~~It turns a `CsgIr` into a list of ordinary ops ... expands to that list in its `apply`.~~

**Critique delta:** that mechanism does not exist (finding 12). There are three honest options:
- (a) **One op that calls library functions inside** (`model.import_openmc {source, materials_source, options}`, following `model.import_step`, finding 7). It calls `sm_boolean::webcut` / `boolean` / `merge` and the fem functions inside one transaction. No contract change. History is one node. The hash depends on the start state, the deck bytes, the options and `IMPL_VERSION`. Fork, replay, MCP, CLI and Python work as they do for STEP.
- (b) **A planner that emits a `core.batch`** of ordinary ops. History then holds real ops for W5 "edit one cell". But a batch has no control flow and refers to outputs by index (`$i.role[k]`). Which webcut role a fragment lands in depends on the geometry, so the planner must dry-run the kernel to emit concrete references. Doable, but twice the work and fragile.
- (c) An `api_change_request` to make `apply_nested` public. This is the orchestrator's call and touches a frozen crate.

Recommendation: (a) for W4. It is what STEP import does and costs nothing in contracts. In W5, regeneration re-runs the import with an override map rather than editing sub-ops. The choice goes to the user as Q6 because it decides what W5 can edit. The new crate also needs a root `Cargo.toml` member and plugin registration in `crates/sm-server/src/standard.rs`. Both are orchestrator edits (CSGBRIEF-0).

Why better than Cubit: Cubit has no CSG import, so users rebuild CSG by hand. Here an unmodified OpenMC model becomes a forkable timeline node with C9 region names as blocks.

### D2. Build by space partition (webcut arrangement) and classify, not by per-cell DNF booleans

~~For each flat cell ... intersect/subtract primitive bodies per DNF term, unite the terms, then imprint + merge all cells.~~

**Critique delta (design):** the per-term boolean route has four defects at d2f4803:
- (1) Finite primitives must overhang the clip brick, and caps that land on brick faces are the co-planar cases the kernel handles worst.
- (2) `unite` of disjoint terms fails with "bodies do not touch" (`sm-boolean/DESIGN.md:67-71`), and disconnected cells are normal (for example a moderator split by a baffle).
- (3) "Plane literals only shrink the brick" holds only for axis planes, and `type="plane"` is in the fixtures (`surfaces.rs:48`).
- (4) Every cell pays a boolean chain plus a global imprint + merge, and the overlap check needs a further O(pairs) boolean pass.

A better design is the classic CSG-to-B-rep partition:

1. For each distinct universe, start from one box: its clip (lattice element box, filling cell, or `bounds`).
2. Webcut the pieces by the universe's surfaces in sm-csgio's canonical surface order (DESIGN.md:20-24), with `merge: true`. Every cut face is shared, so the result is conformal by construction and needs no imprint/merge pass.
3. BSP rule, to keep the fragment count down: cut a piece by surface S only if the cells that can still contain the piece (given the signs already known for it) disagree on S. Each piece carries its sign vector, which is an exact `CsgCell`.
4. Classify each piece by evaluating every candidate cell's region on the sign vector (no point sampling). The regions come from `ImportedCsg::cells()` (`world.rs:349`) and are already DNF (`CsgCellDef`, `crates/sm-rt-api/src/csg.rs:105`; DESIGN.md steps 2-3). So the 4096-term cap (`dnf.rs:6`) stays as it is today, and evaluating a DNF on a sign vector is cheap. 0 owners = gap, 1 = that cell, 2 or more = overlap (D5).
5. Unite the pieces of one cell that share a face (this removes internal faces). A cell that stays disconnected becomes several volumes in one block.

Exact carriers:
- Today: x/y/z cylinders and any-axis cylinders through `CutTool::Surface(Cylinder)`, and planes of any normal.
- New tool types (D3): spheres, tori and cones.
- Double cones: cut at the apex plane first (the bridge does the same, `bridge.rs:11-13`).
- General quadrics: through the classifier in D3.

Primitives keep exact analytic surfaces (no tessellation, no fit).

Optional, decided by CSGBREP-CORE: give each piece `Carrier::Csg` with its sign vector (finding 4). The bridge then reads the B-rep exactly, with no "volume = intersection of its face half-spaces" assumption, and `topo.validate` checks that each half-space has an implicit form (`crates/sm-topo-api/src/validate.rs:333-341`). This works only for pieces that are not united in step 5.

### D3. Kernel work: more webcut tools, and a quadric classifier (not `model.create_halfspace`)

~~Add `model.create_halfspace {surface, side, bounds}` (owner sm-model) ... with the `Implicit` carrier that the bridge and the boolean ops already understand (finding 4-5; `tests/quadric_ops.rs`).~~

**Critique delta:** wrong owner and wrong carrier.
- A bounded half-space body is a webcut of the bounds box that keeps one side. That code lives in sm-boolean (`webcut.rs:41-57`), not in sm-model, which has no cutting machinery.
- `tests/quadric_ops.rs` tests only cylinders and spheres; `grep Implicit crates/sm-boolean/src crates/sm-boolean/tests` is empty.
- SSI routes Plane x Quadric through `implicit_def` (`crates/sm-geom/src/intersect/ssi.rs:331`) and every other pair to `march::intersect_general` (`:341`).
- An `Implicit` face cannot be meshed (finding 4).

Replacement:

- **Quadric classifier** (in `crates/sm-csgbrep`, or in sm-geom, which is not frozen). It eigen-decomposes the 3x3 part of the 10 coefficients, recovers the frame, and returns the exact parametric `SurfaceDef`: Plane, Sphere, Cylinder (any axis), Cone (double, as two nappes), EllipticCylinder or EllipticCone. In the first sub-wave, ellipsoids, paraboloids and hyperboloids are refused with a typed `Unsupported` naming the deck surface (later: exact rational NURBS). The sign convention follows the written `f` (DESIGN.md:94). Classification uses a relative tolerance on the eigenvalues and is pure arithmetic, so it is deterministic.
- **Webcut tool types** (sm-boolean): extend `CutTool::Surface` to Sphere, Cone (one nappe with its apex plane), Torus, EllipticCylinder and EllipticCone. Add the body-tool form for closed tools that lie fully inside a target (a sphere or torus that meets no target face). This also ticks PARITY:36 "tool body".

### D4. Lattices: build each distinct universe once, then copy

- Rectangular lattice with one distinct universe: build the element once, run `model.copy` with a count and a pitch (as PARITY `:33`), then clip the container. ~~The 17x17 assembly is 289 copies, not 289 builds. Cost is O(distinct universes), not O(elements).~~
  **Critique delta:** `model.copy` deep-copies (finding 8), so volumes, topology and memory are O(elements). Only the *webcut/boolean* work is O(distinct universes). The merge after copying is also O(elements), with the face-box sweep of `crates/sm-boolean/src/merge.rs:388-400` (backlog `:95`). The acceptance counter must count kernel cuts, not copies.
- Mixed universes: one build per distinct universe, with copies placed from the lattice map. Universes used under a rotation get a `rotate` op (the IR transform is `r = M^T u + t`, DESIGN.md:100).
- Homogeneous `outer`: container minus the array box, one volume. A structured `outer` stays refused, as in the tracker.
- Hex lattices: the reader still refuses them (`backlog.md:112`). The design leaves room for them: a hex element is a prism with 6 planes (`model.create_prism` exists), and rings follow OpenMC order. Not scheduled; see Questions.
  **Critique delta:** the model side is readier than the reader. `Layout::Hex { rings, pitch, orient, nz }` is already in the frozen model API (`instancing.rs:126-135`). Real instancing (one prototype and a model `Lattice`) would make a 17x17 O(distinct universes) in memory and keep the lattice in `.smrt`. But no op creates instancing, and booleans refuse instanced volumes (finding 8). So W4 uses copies, and instancing is a separate later task (W5 candidate) owned by sm-model plus sm-boolean.

### D5. Overlap and gap: exact, by volume, before meshing

**Critique delta:** with D2, the overlap and gap checks fall out of classification instead of needing a separate pairwise boolean pass:

- **Overlap:** a piece owned by two or more cells. Report the deck cell ids (from sm-csgio's `CellSource`) and the piece's exact volume (`model.measure`). The default policy is `error`, matching the tracker import, which already rejects overlaps (`crates/sm-csgio/DESIGN.md:32-33`; csg-import Q4/O17). The options are `first_wins` (cell order) and `report` (continue, keep the piece in the first cell, warn).
- **Gap:** a piece owned by no cell that lies inside the build box and not beyond a vacuum, reflective or white surface (same rule as the tracker's check leaf, DESIGN.md:29-31). Report each gap piece's volume and box. ~~World box volume minus the sum of cell volumes~~: that subtraction loses about `V_world * 1e-12 / V_gap` in relative accuracy (1e-6 for a 1e-3 slab in a 1e3 box), so it cannot meet a 1e-12 acceptance.
- **Coincident surfaces:** reuse sm-csgio's surface dedup (planes normalised, definition bytes, DESIGN.md:20-24) so that equal deck surfaces become one kernel surface before any cut. **Critique delta:** that dedup runs on the IR before placement. Some surfaces coincide only after the transform: equal-radius pins in two universes, or a universe plane lying on a lattice plane. These need a second dedup on the placed surfaces, at the model's linear tolerance, before cutting. This is the main defence against `TangentContact`. Dedup does not remove exact line tangency between a cylinder and a plane (a pin touching its element box). That case stays a typed error naming the deck cells and surfaces until a kernel tangency task exists.

~~Why better than Cubit: Cubit finds overlaps only on request and by facets (`check`/`overlap` on geometry is exact volume too, but gaps are not a first-class concept).~~ **Critique delta:** the unsourced claims about Cubit are removed. What this design gives: the import fails or reports, naming deck ids, in one pass; the volumes are exact; and an exact partition replaces the sampled tracker check, which "cannot prove disjointness" (DESIGN.md:58).

### D6. Region names and boundaries

C9 names (`names.rs`) become blocks (one block per region name), so the `.smrt` regions are the material names and there is no `smrt-region-names` warning. Boundary flags become sidesets: vacuum, reflective and white are three named sidesets on the faces that lie on those deck surfaces. Reflective or white on a curved surface keeps the existing `import.reflective_curved` warning. Only the existing `fem.block_add` / `fem.sideset_add` (`crates/sm-fem/src/ops.rs:84,270`) are used. They have no public library function today, so how the import reaches them inside one op is settled in CSGBRIEF-0.

### D7. The round trip and the oracle

`deck -> B-rep -> mesh -> .smrt` is checked against `oracle.json` (OpenMC `find_cell`, chords) in layers:

1. **Exactness layer (no meshing error):** B-rep volumes equal analytic volumes (pi r^2 h, 4/3 pi r^3, box products). Cell membership by the kernel point-locate (`locate_point`, `crates/sm-boolean/src/lib.rs:40`) equals OpenMC at the 600 oracle points per model. Points within linear tolerance of a surface are excluded, using the same 5e-3 nudge as `gen.py`.
   **Critique delta:** volumes come from Gauss quadrature, except whole spheres (`crates/sm-model/src/queries.rs:150-155`). 1e-12 relative holds for untouched primitives (`crates/sm-model/tests/prim2.rs:25`). After cuts, use 1e-9, as for the W3c pin (`w3c-report.md:40`).
2. **Mesh layer (discretisation error):** ~~Planar-only models must match to 1e-9 (the existing chord tolerance, 2e-9). Curved models: halving h four times gives chord error ratio near 4.~~
   **Critique delta:**
   - Planar models match OpenMC chords to 2e-9, the existing tolerance. One number, not two.
   - For curved models, a single chord's error depends on where the ray meets the polygon, so an order measured on chords is noisy. There is a better and exact check. When the circle edges are meshed by equal intervals (the through-pin sweep of a pave source), the mesh boundary is the inscribed regular n-gon prism. Its block volume is `n r^2 sin(2 pi / n) / 2 * H`, and its chords have closed forms. Assert the `.smrt` chords and block volumes against that polygon geometry to 2e-9. This is the backlog's own suggestion for the pin (`backlog.md:131`).
   - Bound the difference to OpenMC's circle chords by `C h^2`, with `C` measured once per fixture and committed. A plain sagitta bound `r (1 - cos(pi / n))` does not hold: the chord error at a crossing grows like sagitta / sin(incidence), and the fixtures have oblique rays (for example `gen.py:133`).
   - The test sets the circle intervals explicitly (`mesh.intervals`), so `n` is fixed and the closed form is known.
   - If a convergence order is kept, assert it on per-material volume, not on chords.
3. **Critique delta (polyhedral product):** the same meshed model goes through `mesh.to_poly` to `.smsh` and VTU. The per-block sum of cell volumes equals the tet/hex mesh's to 1e-12, and `mesh.validate` reports 0 violations.

Compared with DAGMC/XDG: there is no facet-gap watertightness question, because the mesh is conformal and the hash is stable. The same deck can also be tracked exactly by the analytic tracker that already exists (`ImportedCsg::tracker`, `world.rs:319`). So one test has one deck and three trackers (analytic, mesh, OpenMC).

## Tasks

Sizes: S under 400 lines with tests, M under 1500, L over.

### CSGBRIEF-0 (orchestrator, S): contract and wiring check
Owner: orchestrator.

Scope:
- Confirm the import op needs no frozen-crate change under D1 (a), and record the D1 choice (a/b/c) after Q6.
- Add `crates/sm-csgbrep` to the root `Cargo.toml` and register its plugin in `crates/sm-server/src/standard.rs`.
- Decide whether the second `Source` (materials) is a second field of the same `Source` type (no change) or needs an archive type (then an `api_change_request`).
- Check the library entry points D1 (a) needs. sm-boolean has `pub fn webcut` (`webcut.rs:112`), `boolean` (`boolean.rs:155`) and `merge` (`merge.rs:571`). But `model.copy` (`crates/sm-model/src/copy.rs`) and `fem.block_add` / `fem.sideset_add` (`crates/sm-fem/src/ops.rs:84,270`) have no public non-op function. The ops' layer bookkeeping (`model_txn` lineage, colours, groups, `sm-boolean/DESIGN.md:59-63`) must not be skipped. Either the owners of those crates add small library functions in this wave (they are not frozen), or the import calls their `TypedOp::apply` with its own context. If neither is clean, fall back to D1 (b) or (c).

Acceptance: written answers in the wave plan; the empty crate builds and `smesh ops` lists its plugin. Hash impact: none. Depends: none.
**Critique delta:** added the plugin registration and workspace member (orchestrator-owned files), the D1 decision and the library entry-point check.

### WEBCUT-PROBE (S): what the kernel cuts today (was QUADRIC-PROBE)
Owner: `crates/sm-boolean/tests/` only (new test file) plus a short report.

Scope:
1. A brick webcut by a tilted, offset cylinder (`model.webcut_cylinder`) and by a non-axis plane, keeping each side.
2. A brick subtract/intersect with the existing sphere, frustum and torus primitives, including a primitive fully inside the brick.
3. The same with an `Implicit` quadric face, wherever a path exists.
4. The two kernel behaviours D2 relies on:
   - Webcut of one piece whose face is shared (`merge: true`) with an uncut neighbour: the neighbour stays unchanged and the shared face is split conformally. `webcut.rs:147` refuses a body that holds other cells, so each piece must be its own body.
   - `unite` of two pieces that share a merged face gives one volume with that face removed.

Acceptance: volumes equal analytic values to 1e-9 relative (tilted cylinder through a box: pi r^2 L for an axis crossing two opposite faces; sphere 4/3 pi r^3; torus 2 pi^2 R r^2; frustum 1/3 pi h (R^2 + R r + r^2)). Each case that fails prints its typed error (`TangentContact`, `ResultInvalid`, `Unsupported`), never a panic. The report lists pass/fail per case. Hash impact: none (tests). Depends: none. Decides WEBCUT-TOOLS' size.
**Critique delta:** the probe now measures webcut and the primitives the D2 design uses, and it gates D2. If item 4 fails, CSGBREP-CORE falls back to the draft's per-cell boolean route for the first sub-wave (intersect/subtract overhanging primitives, then imprint + merge, as for the W3c pin), with the known limits listed in the D2 critique. `Implicit` is probed for information only, since D3 maps quadrics to parametric surfaces. The draft's ellipsoid and paraboloid cases move out of the first sub-wave.

### WEBCUT-TOOLS (M, L if the probe fails closed tools): sphere, cone, torus, elliptic and body tools (replaces HALFSPACE-OP)
Owner: `crates/sm-boolean/`.

Scope:
- Extend `CutTool::Surface` (`webcut.rs:131-132`) to Sphere, Cone (one nappe plus the apex plane), Torus, EllipticCylinder and EllipticCone.
- A closed tool fully inside a target splits it into an inside and an outside piece.
- Extend `model.webcut_surface` to take a non-planar face.
- Expose a volume interior point (today `faceclass::interior_point` is `pub(crate)`, `crates/sm-boolean/src/faceclass.rs:199`).

Acceptance:
- For each tool type, a brick cut gives two volumes whose sum equals the brick to 1e-9, and the inside piece equals the analytic volume to 1e-9 (sphere in a brick, torus in a brick, cone frustum `1/3 pi h (R^2 + R r + r^2)`, elliptic cylinder `pi a b h`).
- `topo.validate` is clean on both pieces.
- Both pieces share the cut face (one `TopoFace`, two uses).
- Determinism test: two runs, and `RAYON_NUM_THREADS=1` vs default, give equal hashes.
- Property test with `sm_testkit::strategies::config(n)` over random axes and offsets.
- PARITY:36 "tool body" is ticked.

Hash impact: none on existing ops (new tool types only). Plane and cylinder behaviour in `webcut.rs` is unchanged, checked by the existing webcut tests. Depends: WEBCUT-PROBE.
**Critique delta:** the draft's "Euler characteristic 2" is wrong for the outside of a through cylinder (genus 1, `V - E + F = 0`). It is replaced by `topo.validate` and volume checks.

### QUADRIC-CLASSIFY (S): OpenMC quadric to parametric `SurfaceDef`
Owner: `crates/sm-csgbrep/` (module `quadric.rs`). Scope: the D3 classifier.

Acceptance:
- For random frames (`strategies::config(n)`), `Quadric::from_local` of a sphere, cylinder, cone, elliptic cylinder and elliptic cone round-trips to the same type, with frame and radii to 1e-12 relative and the same sign of `f` at 100 random points.
- Ellipsoids, paraboloids and hyperboloids return `Unsupported` naming the deck surface id and line.
- A plane written as a quadric becomes a Plane.

Hash impact: none. Depends: none.
**Critique delta:** new task. Without it, every off-axis cylinder in a real deck (written as `quadric`, finding 3) cannot be meshed.

### CSGBREP-CORE (L): the partition builder and the op
Owner: new `crates/sm-csgbrep/`. Scope: D1 (a), D2, D4 (single-universe and mixed rectangular lattices) and D6. The `model.import_openmc` op has a `Source` for geometry and materials, an `IMPL_VERSION`, and declared output roles (volumes and bodies, like `IMPORT_OUTPUTS` in `crates/sm-cadio/src/op.rs:23`). It uses sm-csgio's canonical surfaces and flat cell sources. Its refusal list (elliptic torus, structured `outer`, hex, non-classifiable quadric) mirrors the reader.

Acceptance:
- Deck A (`pin` fixture: fuel / void gap / clad / moderator in a box): four volumes. The fuel, gap and clad volumes equal pi r^2 h and its annulus differences to 1e-9. One block per region name. After `mesh.auto`, `mesh.validate` reports 0 violations (as in the W3c pin milestone).
- Deck B (3x3 and 17x17 pin lattices): the volume count is 4 x N plus the container, as specified per fixture. The number of kernel webcuts equals the count for one element per distinct universe (a counter in the op's diagnostics, no wall clock). The 17x17 imports within `MAX_COPIES`.
- Deck C (`rotations`, `translated`, `lattice_xf`): volume centroids equal the analytic placement to 1e-9.
- Every committed oracle fixture either imports or returns a typed `Unsupported` naming the element and line. `outer_structured` is the expected refusal.
- In process: the same deck run twice, and with `RAYON_NUM_THREADS=1`, gives equal state hashes. Deck renumbering and text reordering give the same state hash (reusing sm-csgio's `canonical.rs` property).
- Disconnected cells become several volumes in one block (a fixture with a moderator split by a plane).

Hash impact: new op, new golden state hashes for the new fixtures only. No existing hash moves.

Depends:
- CSGBRIEF-0.
- WEBCUT-PROBE (its item 4 picks the D2 partition or the boolean fallback).
- QUADRIC-CLASSIFY.
- WEBCUT-TOOLS, for sphere/cone/torus decks only. The plane-and-cylinder path covers `pin`, `lattice2d`, `lattice3d` and `deck_o` and ships without it.

**Critique delta:**
- 1e-12 changed to 1e-9 after cuts.
- "CLI, MCP, in-process and Python give the same hash" moved to CSGBREP-CLI, which builds those front ends.
- The counter counts cuts, not copies.
- The `gen.py` item moved to ORACLE, which owns the fixtures.

### CSGBREP-CHECK (M): exact overlap and gap
Owner: `crates/sm-csgbrep/`. Scope: D5 with the three overlap policies and the gap report, placed-surface dedup before cutting, and typed diagnostics through `delta.diagnostics`.

Acceptance:
- Two equal spheres (radius R, centre distance d) that overlap: the reported overlap volume equals the lens `pi (4R + d)(2R - d)^2 / 12` to 1e-9 relative. This needs the sphere tools from WEBCUT-TOOLS, so a planar fixture with two boxes overlapping by a slab runs first (slab volume to 1e-9).
- A deck with a missing slab 1e-3 thick in a box of size 10: one gap piece, whose volume equals the slab to 1e-9 relative (measured on the piece, not by subtraction).
- A planar overlap sliver 1e-4 thick (above `linear`, far below the sampler's spacing) is found, with its exact volume. ~~1e-6 sliver; assert the tracker check misses it.~~
- Equal radii in two universes placed coaxially, and a zero fuel/clad gap, import with no `TangentContact`. A pin tangent to its element box returns a typed error naming both deck surfaces.

Hash impact: none beyond new fixtures. Depends: CSGBREP-CORE.
**Critique delta:**
- A 1e-6 sliver is at the model's linear tolerance (`sm-boolean/DESIGN.md:97-98`, `linear = 1e-6`) and would be welded or rejected, so the sliver is now 1e-4 and planar. Curved slivers below 1e-2 are near-tangent and fail by design.
- Asserting that the sampler misses the sliver pins a seed-dependent miss, which the W3c csg-import plan forbade (`csg-import.md:183`, risk 2).
- The tangent-to-plane case is a typed error, not a pass: dedup cannot remove a cylinder-plane line contact.

### CSGBREP-ORACLE (M): round trip against OpenMC
Owner: `crates/sm-csgbrep/tests/` and `crates/sm-csgio/tests/fixtures/openmc/`. The fixture generator belongs to sm-csgio, so the orchestrator assigns that directory to this task for the wave.

Scope: D7 layers 1-3 on the committed `oracle.json`. `gen.py` gets `reset_auto_ids()` and SIGTERM exit 143 (backlog `:138,149`). ~~Add decks for quadric, cone, torus, lattice with rotated fill.~~ Add one deck with an off-axis cylinder written as `quadric`, and one with a disconnected cell. No OpenMC is needed at test time.

Acceptance:
- Membership at the 600 points equals OpenMC on every fixture that imports.
- Planar decks: per-material chord through the `.smrt` to 2e-9.
- Curved decks with equal-interval circle meshing (intervals set explicitly): `.smrt` chords and block volumes equal the inscribed-polygon geometry to 2e-9, and differ from OpenMC by at most the committed `C h^2` per fixture.
- `pin` and `lattice2d` through `mesh.to_poly`: `.smsh` and VTU are written, per-block volume sums equal the source mesh's to 1e-12, and `mesh.validate` reports 0 violations.
- `gen.py` run on a subset gives byte-identical `geometry.xml` for those models.
- Mutating the lattice row order or the rotation transpose in the reader fails at least five of the tests (as `the_fixtures_are_not_a_vacuous_oracle`, `openmc_oracle.rs:176`).

Hash impact: none. Depends: CSGBREP-CORE.
**Critique delta:** `surfaces` already has a quadric, cones and tori, and three fixtures cover rotated fills (finding 10). The polyhedral export row is new. The curved-mesh acceptance is now exact against the polygon instead of a noisy chord order.

### CSGBREP-CLI (S): front ends
Owner: `crates/sm-cli/` (a `smesh import-openmc` sugar over the op), `crates/sm-mcp/` (tool listing), `crates/sm-py/` (binding).

Scope: expose the op, and show gap/overlap diagnostics in text mode. This also closes the backlog item on printing `delta.diagnostics` (`backlog.md:142`).

Acceptance:
- Text and JSON outputs both carry the diagnostics.
- Exit code 1 on an overlap by default, 0 with `--overlap report`.
- The `pin` deck through CLI, MCP, in-process and Python gives one state hash.

Hash impact: none. Depends: CSGBREP-CHECK.
**Critique delta:** the four-front-end hash row moved here from CORE.

### CSGBREP-HEX (M-L): hex lattices (later)
Owner: `crates/sm-csgio/` (reader and IR `HexLattice`; the orchestrator approves the IR change because the core is frozen, DESIGN.md:3-4), then `crates/sm-csgbrep/`.

Scope: `hex_lattice` in both orientations, ring order as in OpenMC, homogeneous `outer`.

Acceptance:
- New oracle fixtures (`gen.py`) at 600 points.
- A 7-ring lattice gives 127 element prisms, each with cross-section area `sqrt(3)/2 p^2` (p the flat-to-flat pitch) to 1e-9.
- The B-rep volume sum equals the analytic sum to 1e-9.
- sm-csgio `content_hash` is unchanged for every existing fixture (golden test).

Hash impact: the IR gains a variant, and old decks' hashes must not move. Depends: CSGBREP-CORE, and an explicit user request (Q2).
**Critique delta:**
- The draft's per-element area `3 sqrt(3)/2 p^2 / 3` equals `sqrt(3)/2 p^2` only when p is the flat-to-flat pitch. It is now stated once, with p defined.
- 7 rings is `3*7*6+1 = 127` elements.
- Size raised to M-L (reader, IR, flatten, tracker and generator).

### CSGBREP-TRACK-PERF (M, optional, later): cold locate and analytic stepping
Owner: `crates/sm-rt/` only. Scope: a cell-box BVH for cold locate in the analytic tracker, and O(cells) stepping, per backlog `:105`. Acceptance: output bit-identical to today on the existing oracles; the speedup is measured in tests by a counter, not by time. Hash impact: none. Depends: none; trigger as in the backlog.

## Priority (for the user's goals: polyhedral meshes and a ray tracer that beats DAGMC/XDG)

- **Now (first W4 sub-wave):** first CSGBRIEF-0, WEBCUT-PROBE and QUADRIC-CLASSIFY (cheap, in parallel). Then CSGBREP-CORE on the plane-and-cylinder path (after the probe decides D2), CSGBREP-CHECK and CSGBREP-ORACLE.
  Reason: this is the shortest path from a real OpenMC model to a polyhedral mesh and a `.smrt`. The oracle test is what lets the ray-tracing benchmark claim correctness on the user's own models, and the DAGMC/XDG comparison needs the same geometry on both sides. Pin cells and assemblies (planes and z-cylinders) need no new kernel tool, so start there.
- **Now, second step:** WEBCUT-TOOLS (spheres, cones, tori, body tools) as soon as the probe reports, and CSGBREP-CLI with the others.
- **Later W4 sub-wave:** CSGBREP-HEX (on request); CSGBREP-TRACK-PERF (on the benchmark trigger).
- **W5+:**
  - model instancing for lattices (finding 8);
  - edit-one-cell-and-regenerate;
  - DAGMC universe import (needs mesh-based geometry, `PARITY.md:47`);
  - periodic boundaries (needs `--model` periodic faces, `w3c-report.md:131`);
  - ellipsoid and paraboloid faces as exact rational NURBS.

  An exact curved-face tracker on meshes (no sagitta error in `.smrt`) is the real lever for beating DAGMC, but it belongs to sm-rt, not this area.

**Critique delta (priority):** the order is right for the user's goals. What changes is that a pin-cell or assembly deck reaches a polyhedral mesh with no new kernel work. WEBCUT-TOOLS is therefore not on the critical path and must not block CORE.

## Risks

1. **Kernel robustness on CSG decks.** Tangent and near-tangent contacts are rejected (`sm-boolean/DESIGN.md:84-102`). Mitigation: placed-surface dedup, WEBCUT-PROBE, and a typed error naming the deck cells and surfaces. A pin tangent to its element box is the likely first failure, and it stays a typed error in W4.
2. **Fragment count of the partition.** Infinite deck surfaces can cut cells they do not bound. Mitigation: partition each universe inside its clip box, and apply the BSP rule (cut only where candidate cells disagree). Print the piece count per fixture in the test output.
3. **O(elements) after copying.** `model.copy` is deep, and the merge and validate passes scale with the element count. A full core (tens of thousands of pins) may hit `MAX_COPIES = 4096` per copy op and the merge face sweep (backlog `:95`). Mitigation: 17x17 is the W4 bar; full cores wait for instancing.
4. **Quadric classification tolerance.** A deck quadric that is a cylinder up to round-off must classify as a cylinder; with a slightly unequal pair of eigenvalues it would become an elliptic cylinder. Mitigation: a relative eigenvalue tolerance, and the refusal message prints the eigenvalues.
5. **The mesh round trip is not exact on curves.** The mesh tracker sees polygons and OpenMC sees circles. The plan checks exactly against the polygon and bounds the difference to OpenMC by a committed `C h^2` per fixture. Say this plainly in the report.
6. **Oracle drift.** The oracle is OpenMC 0.16.0 with a dummy multi-group library. Keep the 5e-3 nudge and the committed `oracle.json`.
7. **Scope creep to MCNP/DAGMC/tallies.** Out of scope: MCNP is not a target (C6), there are no cross sections, and densities are dropped (`CsgIr::dropped`).
8. **A second source of truth.** The B-rep and the analytic tracker both describe the same deck, so the cross test (finding 10) must stay green on every change. With `Carrier::Csg` on the pieces (D2, optional), the bridge reads the B-rep exactly.

## Questions for the user

1. ~~**Overlap default.**~~ **Critique delta:** this is not a new decision. The tracker import already rejects overlaps (DESIGN.md:32-33, from csg-import Q4/O17). The B-rep import keeps `error` as the default, with `first_wins` and `report` as options. Removed from the questions.
2. **Hex lattices.** Do your decks use `hex_lattice`? Recommendation: leave hex lattices for the later sub-wave unless you say yes now. The typed refusal tells you the line when you hit one. If yes, it moves to Now (size M-L: reader, IR and generator; the model side already has a hex layout).
3. **Which decks first?** Recommendation: name two real models (a pin cell and an assembly, or a small core slice) so that acceptance also uses your geometry. C7 has us write our own models but does not forbid yours.
4. ~~**Mesh-size policy of the import.**~~ **Critique delta:** nobody would choose otherwise, so it is the default: the import makes geometry, blocks and sidesets only, and meshing is a following `mesh.auto`. Removed from the questions.
5. **Region granularity in `.smrt`.** Today `.smrt` regions are one per block (`backlog.md:102`), so all fuel pins are one region, "fuel". Per-pin tallies or depletion need one region per lattice element (for example `fuel[3,5]`). Do you want per-element regions in `.smrt` now? Recommendation: yes, if your transport code tallies per pin. It is the backlog's "first request" trigger and a small sm-rt/sm-meshio change, and the import keeps one volume per placed cell either way.
   **Critique delta:** reworded. How volumes are named is an implementation detail; the real decision is the `.smrt` region granularity, which is the backlog item at `:102`.
6. **Critique delta (new): one history node or many?** The import can be one op, like STEP import: one timeline node, and regeneration re-runs it with changed options. Or it can be a batch of ordinary ops that a later edit can point at, which needs either a planner that dry-runs the kernel or a change to the frozen op contract. Recommendation: one op now; revisit with W5 editing.

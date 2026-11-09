# HEXPIN: all-hex round geometry, hex-tet transitions, multisweep, volume submap, `hex_only` and `tet`

> Planning draft (W4), area "hexpin". Base: `main` at d2f4803, read-only. Rulings C1 and C5 (`docs/design/w3c/rulings.md:9,13`) deferred these items to W4. Standing principle: Cubit is a bar to surpass; OpenMC is the oracle for geometry; MCNP is not a target. Claims about Cubit behaviour that `docs/research/cubit-schemes.md` does not cover are marked "not verified".
>
> **Critique delta (method):** every file:line below was checked at d2f4803. The decomposition claims were also run as a spike with the existing debug binary `target/debug/smesh` (built 2026-11-07, before the W3c merge). The scripts are in the critic's scratchpad (`spike/cyl.smesh`, `cyl2.smesh`, `cyl3.smesh`). The DECOMP task must re-run the spike at d2f4803 as its first step.

## Goal
The user's round pin cell (cylinder in a 1.26 x 1.26 x 1.0 brick, plus the 3x3 lattice) meshes all-hex in both forms (through pin and enclosed pin), with a regular O-grid that is 4-fold symmetric when the circle count allows it. `mesh.auto` has the strategies `hex_only` and `tet`. A hex region can meet a tet region through pyramids. Many-to-one sweeps and volume submap close the two open P1 rows. Everything is deterministic and replays from stored assignments.

**Critique delta:** the 4-fold symmetry holds only when the circle count n is a multiple of 4 (n % 8 == 0 for corners exactly on the diagonals, R6). The goal now says so.

## Findings (what is true today)

### F1. The through pin is all-hex, but the disk and annulus are paved
- The default strategy lists are fixed: surface `[map, submap, pave, trimesh]`, volume `[map, sweep, tetmesh]` (`crates/sm-mesher/src/auto.rs:161-168`). There is no circle scheme (PARITY `circle`, `hole`: `docs/PARITY.md:66`, unchecked). **Critique delta:** the PARITY line was cited as `:72`; it is `:66`.
- A disk or an annulus therefore gets `pave`. Map and submap refuse a disk (fewer than four corners, `crates/sm-scheme-map/src/lib.rs:79-84`) and an annulus (one loop only, `map/src/lib.rs:69-73`). Pave's design note says: "disks and annuli have irregular nodes at the centre (minimum 0.5-0.8, mean 0.93-0.97)" (`crates/sm-scheme-pave/DESIGN.md:209-211`). Pave is constrained Delaunay, then frontal rows, then perfect matching (`crates/sm-scheme-pave/src/lib.rs:3-6`). Nothing in it enforces 90-degree rotation symmetry.
- Through pin, `tools/w3c_pin.smesh`: 18 volumes, 432 hexes (24 per volume). The count is asserted in the script and in `crates/sm-cli/tests/milestone_w3c_pin.rs:342-343`. Hashes are pinned in `tools/platform/expected.txt:54-63` (C12).

### F2. The pin milestone cannot see the linear-hex volume error
- The test compares only the sum of the mesh volume with 9 x 1.26^2 (`milestone_w3c_pin.rs:340-341`). Fuel plus moderator always sums to the brick, whatever the polygon is.
- With nodes on the circle, a linear n-gon under-fills the disk by `1 - sin(2*pi/n)/(2*pi/n)`: 9.97% at n=8, 2.55% at n=16, 0.64% at n=32 (checked). Size 0.3 on r=0.4 gives n of about 8 (circumference 2.51 / 0.3). For a transport code the fuel volume is a first-order quantity. (n is an estimate; the real interval count must be read from `mesh.plan`.)
- **Critique delta:** this error also reaches ray tracing. The `.smrt` export builds a mesh tracker from the volume mesh (`crates/sm-server/src/mesh_export.rs:158-175`, `BuildInput::Mesh(MeshInput::new(host, ...))`). The pin's `.smrt` therefore tracks through the n-gon fuel, not the circle. The exact cylinder exists only in the analytic CSG tracker (`crates/sm-rt/src/lib.rs:10-11`). This raises the priority of Q2.

### F3. The enclosed pin (H=0.8) is all-tet by ruling C1
- `tools/w3c_pin_enclosed.smesh` asserts 733 tets. The milestone checks two `mesh.auto_tet` warnings (`milestone_w3c_pin.rs:617-621`). **Critique delta:** the warning count is asserted in the test, not in the script.
- The moderator is not a prism: its cavity does not reach the brick faces. Sweep finds no source/target pair.
- The warning text tells the user to "use a strategy without tets" (`auto.rs:591-592`), but no such strategy exists (the registry holds only `default`: `auto.rs:424-433`).
- The model ops needed to cut the moderator already exist: `model.webcut_plane`, `model.webcut_cylinder` (`crates/sm-boolean/src/ops.rs:493,593`, `docs/PARITY.md:35`). `webcut_cylinder` cuts with an infinite cylinder (`ops.rs:571-573`).
- **Critique delta (spike, overturns the draft):** a cylinder cut alone is **not** enough.
  - Cutting the enclosed moderator with the r=0.4 cylinder works, even though the tool coincides with the cavity wall. It gives 4 volumes: the ring (1.08495), two 0.1-thick caps (0.050265 each) and the fuel (0.402124). The sum is exactly 1.5876.
  - But the ring's inner wall is now three faces stacked along z: cap band, fuel wall, cap band (Face#17, #9, #18, each with its own seam). Its inner column therefore has 3 edges, while the outer brick columns have 1 edge. Sweep refuses this with "columns have different lengths" (`crates/sm-scheme-sweep/src/discover.rs:252-254`). The ring is not "the same as the through pin".
  - With that model, `mesh.auto default` **fails outright** and does not even fall back to tets: "no applicable scheme for Surface(Face#13) … overridden by imposition from Volume(Volume#5): Caps(1026)". Face#13 is the brick bottom with the hole. Volume#5 imposes `QUAD | BLOCK_STRUCTURED` on it (1026 = bits 1 and 10, `crates/sm-scheme-api/src/caps.rs:22,47`), and no scheme can meet that on a two-loop face. A partly decomposed model is worse than no decomposition today; see R8.
  - Adding two plane cuts of the outer piece at the cavity end planes (z = -0.4 and 0.4) gives 6 volumes: three square-annulus slabs, two caps and the fuel. With today's `default` the result is `{Hex: 80}`, `mesh.validate` reports 0 violations, and the mesh volume is 1.5876000000000015.
  - **So the enclosed pin can be all-hex today with `default` plus three ops.** No new scheme is needed for C1; only a way to find these cuts (D2). The O-grid improves quality and symmetry; it does not enable all-hex.

### F4. Sweep is strictly 1-to-1, and the source/target are one face each
- `SweepParams.source/target` are `Option<[EntityId; 1]>` (`crates/sm-scheme-sweep/src/params.rs:46-52`).
- `prefilter` requires equal vertex counts and refuses with "many-to-one sweeps are not supported" (`discover.rs:109-122`). A `Target::Group` is refused with "multisweep is not supported" (`scheme.rs:193-195`).
- `find_columns` tries pairs of faces, lowest id first (`scheme.rs:58-103`). The linking faces must be strips (one loop, at least two column coedges: `discover.rs:316-335`, added in W3c). All columns must have the same number of edges (`discover.rs:252-254`). **Critique delta:** this rule is what blocks the cylinder-only cut (F3); it is added here.
- Sweep already handles a composite stack of merged volumes (`Columns.members`, `discover.rs:30-37`). Polygon sources of five or more sides give prismatic polyhedra (`lib.rs:5-6`). Linking faces are required `QUAD | BLOCK_STRUCTURED` Hard (`scheme.rs:231-239`). The source face has no capability requirement beyond what its own scheme gives.
- The target is always a copy of the source (`TargetMode::Project` is parsed but rejected: `params.rs:17-20`). Many-to-one needs the single side to be a union of copies. `CopyConstraint` (`crates/sm-scheme-api/src/copy.rs`) is one source to one target, and it is a frozen contract.

### F5. Volume `map` and submap
- Volume `map` needs exactly six faces, one loop each (`crates/sm-scheme-mapvol/src/lib.rs:48-60`).
- Surface `submap` exists (L, U, staircase as rectangular blocks: `crates/sm-scheme-submap/src/lib.rs:1-6`, `lattice.rs`, `blocks.rs`). There is no volume submap (`docs/PARITY.md:71`, unchecked together with multisweep and copy).

### F6. The tet scheme requires triangles on every face, and pyramids exist only as a cell type
- `tetmesh` imposes `Caps::TRI` Hard on every boundary face (`crates/sm-scheme-tet/src/scheme.rs:183-187`). A hex neighbour imposes QUAD on the shared face, so the planner reports a conflict and the replan loop falls back. Nothing in the pipeline can produce a pyramid.
- The pyramid exists elsewhere in the stack:
  - `Caps::PYRAMID` (`crates/sm-scheme-api/src/caps.rs:34`) and `CellShape::Pyramid` (`crates/sm-mesh-api/src/shape.rs:12,33`).
  - `Pyramid5` in sm-fem (`crates/sm-fem/src/element.rs:25-26`); `Pyramid13` is unsupported (`:46-48`).
  - Signed scaled Jacobian in sm-quality (`crates/sm-quality/src/driver.rs:432-435`).
  - The writers: Gmsh, VTU, OpenFOAM, Exodus and DAGMC (`sm-meshio/src/{gmsh/write.rs, vtu.rs, foam/build.rs, dagmc/write.rs}`, `sm-exodus/src/tables.rs`).
- The planner's capability need is "all of" (`crates/sm-scheme-api/src/planner.rs:134`: `Need::Caps(c) => !caps.contains(*c)`), plus `Need::Forbid` (`:135`). An imposition has a firmness (`require_caps(child, caps, firmness)`, `problem.rs:796`).
- **Critique delta:** what a Soft TRI does is already in the code. `resolve` retries without the Soft needs when the pass that honours them fails with `NoApplicableScheme` or `ImpositionConflict` (`planner.rs:1310-1322`; `Firmness::Soft` is "kept unless it conflicts with a hard request", `firm.rs:18-19`). So a Soft TRI from the tet volume loses to a Hard QUAD from a hex neighbour in the same planning pass, without a replan. R3 shrinks to a generation-time question; see R3.
- `sm-poly` converts only an all-tet mesh (barycentric dual, `crates/sm-poly/src/lib.rs:1-2`). A mixed hex/pyramid/tet mesh is already a valid polyhedral mesh for a finite-volume code, but it cannot go through `mesh.to_poly`.
- **Critique delta:** today's hex-tet transition degrades the hex side. B6 (`crates/sm-server/tests/auto_select.rs`, `b6_auto_next_to_a_hard_tet_neighbour_falls_back_to_a_conformal_scheme`; PARITY:72) reaches a conformal fill only with a wedge layer, so the hex neighbour stops being hex. That is the reason pyramids are needed.

### F7. The strategy machinery already supports list-driven strategies
- `DefaultStrategy::with_lists(AutoLists)` and `with_tet_scheme` exist (`auto.rs:317-332`). `StrategyRegistry::default()` inserts one strategy (`auto.rs:424-433`). The stored op records the strategy name and the explicit assignments (`auto.rs:597-605`). Replay applies the assignments and calls no strategy (`auto.rs:491-499`), so adding strategies cannot change old histories.
- **Critique delta:** `DefaultStrategy::name()` returns the constant `DEFAULT_STRATEGY`, and `description()` is a fixed string (`auto.rs:360-367`). A second list-driven strategy therefore needs a name and description field (or a wrapper type); `with_lists` alone is not enough. This is small and stays inside sm-mesher.
- `hex_only` is `AutoLists` without `tetmesh`. `tet` is `AutoLists` with `[tetmesh]` and a triangle surface list.
- The tet warning is keyed on the scheme name `self.tet` (`auto.rs:397-409`, emitted at `:582-596`). `tet_reasons` is never stored (`serde(skip)`, `auto.rs:55-58`). For `tet` the warning would be noise.

### F8. Interval parity is expressible; a "multiple of 4" is not
- The interval constraints are `Fixed`, `Goal`, `Range`, `Linear` (signed `i32` coefficients) and `EvenSum` (`problem.rs:72-116`; builder `even` at `:995-1002`). The enum is `non_exhaustive` in a frozen crate. **Critique delta:** `Goal` and `Range` were missing from the list. The signed coefficients were checked: they are what makes the D1 annulus row `n - m1 - m2 - m3 - m4 = 0` expressible.
- An O-grid needs the circle count to split into four arcs `a, b, a, b`, so `n = 2(a+b)`, and EvenSum is enough. A divisible-by-4 or by-8 rule is not needed for validity, only for symmetry (D1, R6). A `Linear` row has no auxiliary integer variable, so "n % 4 == 0" cannot be stated. It holds on the pin anyway because `n = m1+m2+m3+m4` and the brick makes opposite sides equal.

## Design

### D1. O-grid scheme crate (`sm-scheme-ogrid`): disk, annulus, ring
One new surface-scheme crate with a pure kernel and a thin adapter, like `sm-scheme-pave`.
- **`ogrid` on a disk** (one loop, one circular edge with one vertex; `create_cylinder` topology, `crates/sm-model/src/prim.rs:116-120`): a central block of `a x b` quads plus four collar blocks of `k` radial layers. Each collar block is a structured `a x k` or `b x k` grid; the central block is a map block. Quad count: `a*b + 2*k*(a+b) = a*b + k*n`.
  - Corners are chosen as indices into the circle's boundary nodes, not as B-rep vertices. A circle has one vertex, so Cubit-style submap/map cannot place corners; this scheme does not need vertices. With `n = 2(a+b)`: `a = b` when `n % 4 == 0`, otherwise `|a - b| = 1`.
  - The phase of the four corners relative to the seam vertex is a parameter. Default: 45 degrees from the seam if n is a multiple of 8, otherwise the nearest node. Partitioning the curve would help only here (virtual topology, a W4 sibling task); it is not required.
- **`ogrid` on an annulus with a 4-sided outer loop** (the pin moderator face): four trapezoid blocks between the circle and the square, with radial lines from the square corners. Side `i` carries `m_i` intervals and the circle `n = m1 + m2 + m3 + m4`, which is one `Linear` Eq row with coefficients `+1, -1, -1, -1, -1` (F8). The circle's `EvenSum` is implied. **Critique delta:** the draft required "n/4 = m per side". That is not needed; each block only needs its arc count to equal its side count. Equal sides come from the neighbours (the brick's opposite sides).
- **`ogrid` on an annulus between two circles** (clad, gap): a structured `n x k` ring, periodic in the angular direction. `map` cannot do this today (it needs one loop, `map/src/lib.rs:69-73`).
- **`collar` on a face with circular holes** (deferred, see the delta): `k` layers of quads around each circle, then the remainder meshed by another scheme. **Critique delta:** a scheme cannot call another *scheme* on a sub-region; there is no virtual sub-face. It can call the pure kernel `sm_scheme_pave::pave_region` (`pave/src/lib.rs:14`) on the collar's outer boundary. Scope `collar` as "collar plus a `pave_region` remainder". It is not needed for the user's pins (each pin cell is its own face), so it moves to the later sub-wave.
- **Radial grading** `ratio` (first-layer thickness), so fuel/gap/clad rings can be graded for a transport code that wants thin outer cells.
- **Critique delta, an alternative to offer: a `polar` mode.** Reactor-physics transport meshes are usually rings x azimuthal sectors with a polygonal centre, the shape of MOC flat-source regions. Sweep already turns a source polygon of five or more sides into a prismatic polyhedron (`sweep/src/lib.rs:5-6`). A `polar` mode of the same kernel would mesh a disk as `k` rings of `n` quads around one central n-gon. Swept, that gives n-gon prisms in the centre and hexes in the rings.
  - Strengths: it is exactly symmetric for every even n (not only n % 4 == 0), has no valence-3 nodes, and uses fewer cells.
  - Limits: it is for FV/polyhedral codes only. FE exporters cannot take the polygon prism.
  - Cost: small (a periodic structured ring plus one polygon), so it fits in OGRID-K. Which mode the user's code wants is Q6.

Why this beats Cubit and the current pave:
1. All interior nodes have four neighbours except exactly four of valence 3 at the central block corners. Pave leaves irregular nodes at the centre (F1).
2. When n % 4 == 0, the mesh is invariant under the 90-degree rotation and under the reflections that map the pin cell onto itself, so the discrete transport solution keeps the lattice symmetry. Delaunay-based pave does not guarantee this.
3. The rings align with material interfaces and give a place for radial grading. Concentric rings are the structure of a polyhedral/FV transport mesh in a pin.
4. Corners on boundary nodes need no virtual topology and no multiple-of-4 rule; the design needs only `EvenSum` and one `Linear` row (F8).
5. The scheme applies only when the face is exactly a disk or an annulus, so general faces still fall to pave. Scores today: map 100 (`map/src/lib.rs:89`), submap 80 (`submap/src/lib.rs:102`), pave 40 (`pave/src/lib.rs:141`), trimesh 30 (`tri/src/scheme.rs:120`). Map and submap refuse disks and annuli (F1), so any `ogrid` score above 40 wins on them. 70 is fine. **Critique delta:** the draft cited the scores from a doc comment (`tri/src/scheme.rs:21`); they are now cited where they are set.

Literature: the five-block O-grid is the standard blocking topology for round cross-sections (Thompson, Soni, Weatherill, *Handbook of Grid Generation*, 1999, section on block-structured grids). The elliptical mapping of a square to a disk avoids blocks but is singular at the square corners (Jacobian determinant zero at (+-1, +-1)), so it is not used. Where the central block is a square (`a = b`, phase 45 degrees), each collar quad has a 135-degree angle at the central block corner, so its scaled Jacobian there is sin(135 deg) = 0.7071 before smoothing. This is the number the quality test pins. **Critique delta:** the condition `a = b` with corners on the diagonals was added; otherwise the angle differs. Cubit's own `circle` scheme is not described in `docs/research/cubit-schemes.md`; any comparison with it is not verified.

### D2. Round-geometry decomposition is a suggestion that becomes ordinary ops
The best design keeps replay simple: `mesh.auto` stores explicit assignments and replay calls no strategy (F7). Decomposition changes the B-rep, so it must also be explicit.
- A pure planner (`sm-decomp`) takes a volume that no hex scheme accepts and returns cuts, each given as the exact arguments of an existing op (`model.webcut_cylinder`, `model.webcut_plane`). First version, axis-aligned:
  1. Find cavity cylinders whose axis is the sweep axis.
  2. Cut the host by the extended cylinder.
  3. Cut the outer piece by the plane of **every cavity end face** (not "if needed").
  - Result for the enclosed pin: 6 volumes (three square-annulus slabs, two disk caps, the fuel), all 1-to-1 sweeps. The spike verified this (F3).
  - **Critique delta:** the draft's cut set (cylinder only, "then cut the host's caps … if needed") leaves a ring whose columns differ in length. `default` then fails outright (F3). The plane cuts are always needed when a cavity ends inside the host.
- **Critique delta (CutSpec shape):** the cuts are a sequence, and the later cuts target pieces that do not exist until the earlier ones run. In the spike, cut 2 targets `c.positive` and cut 3 targets `p1.positive`. So `CutSpec` must be an **ordered list** whose targets are either existing ids or references `{step: i, role: "positive" | "negative"}` to an earlier step's output role (the `WEBCUT_OUTPUTS` roles, `ops.rs:458`). Alternatively the query returns the cuts as a command-language snippet. The QUERY JSON schema depends on this choice; pick the role-reference form, which is the same in MCP, CLI and Python.
- A read-only query (`mesh.decompose_plan`, in the style of `mesh.plan`, `crates/sm-mesher/src/plan_query.rs:750`) returns the list. The user or a script applies it with the normal ops plus imprint and merge, and the timeline records them. `hex_only` uses it in its error text: "volume 7 cannot be swept; suggested cuts: ...".
- No hidden decomposition inside `mesh.auto`. The history shows exactly what was cut.
- Better long-term alternative (not now): let sweep accept columns with intermediate vertices. The shorter columns would get layer rows so that each intermediate vertex lands on a layer. That would make the cylinder-only cut sweepable and would avoid the plane cuts for every axial zone (fuel, plenum, end plugs). It is a sweep change, which MSWEEP owns.
- The general automatic multisweep decomposition of Shepherd, Mitchell, Knupp and White ("Methods for multisweep automation", 9th IMR, 2000) is out of scope. The first version covers the axisymmetric reactor features in the user's models (pins, guide tubes, plena, axial blankets).

### D3. `hex_only` and `tet` strategies (C5)
- `hex_only`: surface list `[map, ogrid, submap, pave]`, volume list `[map, sweep]`. There is no tet fallback. A volume without a hex scheme fails with the per-candidate reasons the scratch plan already collects (`auto.rs:244-260`, `plan_with_blame` `:181-249`) and with the D2 cuts. On faces without circles, its assignments equal `default`'s. **Critique delta:** on disk and annulus faces they differ (`ogrid` against `pave`) unless `ogrid` also enters `default` (Q1). The draft's "same as `default`" claim is false otherwise.
- `tet`: surface list `[trimesh]`, volume list `[tetmesh]`. It does not emit `mesh.auto_tet` (the warning is for a fallback; here tets are the request).
- `default` is unchanged except for the question of `ogrid` in its list (Q1).
- **Critique delta:** both new strategies need their own `name()` and `description()` (F7). The `mesh.auto_tet` hint at `auto.rs:591-592` should name `hex_only`.

### D4. Many-to-one and many-to-many sweep (multisweep) on the existing machinery
- The params become `source: Vec<EntityId>`, `target: Vec<EntityId>`; the old `[id]` JSON still parses. Columns are discovered per face set; every source vertex still needs exactly one onward edge.
- Many-to-many: one volume whose bottom and top are each split into the same set of faces by imprinted edges (for example a disk and an annulus on each side, with no internal face). The set-aware discovery pairs the faces; generation per pair is the existing code, plus one shared layer-count constraint. **Critique delta:** the draft described two imprinted volumes, which are two plain 1-to-1 sweeps already. The new case is one volume with several faces on each side.
- True many-to-one (one source face and several target faces, or the reverse): the single-side mesh must contain the projection of the many-side boundary. The design meshes the many-side faces first; the single side is then the union of their copies. This needs a union copy in the frozen API (`copy.rs`) and is an ACR candidate (R2). The alternative without the ACR is to require an imprint on the single side (a virtual partition from the W4 virtual-topology task).

### D5. Hex-tet transition by pyramids
- A quad face between a hex volume and a tet volume gets one pyramid per quad, apex inside the tet volume. The tet scheme then treats the four triangles of each pyramid as frozen boundary (Owen, Canann and Saigal, "Pyramid elements for maintaining tetrahedra to hexahedra conformability", ASME AMD-Vol. 220, 1997; the same idea appears in Owen and Saigal, "H-Morph", IJNME 2000).
- Apex position: the centroid of the quad plus `h` along the averaged unit normal, with `h = 0.5..0.8` of the mean edge (tuned in a spike). The apex is rejected if the pyramid inverts or leaves the volume; the replan loop then falls back (existing behaviour).
- The tet scheme takes `transition: "pyramid" | "none"`, default `none`, so every existing hash stays. With `pyramid`, TRI is a Soft requirement on the faces. The planner already lets a Hard QUAD from a neighbour win (F6, `planner.rs:1310-1322`), and a face shared only with this volume stays triangles.
- Conformal: the hex quad face and the pyramid base are the same four nodes. FE meshes stay conforming (a split quad would leave a hanging diagonal).
- Why this beats today's transition: B6 keeps conformity by giving up the hex side for wedges (F6). Pyramids keep the hexes.
- Polyhedral consequence: pyramids are polyhedral cells for an FV code, but `mesh.to_poly` still needs all tets (F6). A later task can dualize only the tet region; that is not in this area.

### D6. Volume submap
- Generalise the 2D lattice/blocks code to a 3D polycube: a union of boxes on an integer lattice, validated exactly and decomposed by cut planes. Each box becomes a map block; the interfaces are shared quad grids. It needs 3D interval rows `sum(+i) = sum(-i)` per axis, the same shape as the 2D rows in `submap/src/lib.rs`. These are `Linear` rows with ±1 coefficients (F8).
- It is the least valuable item for pins and assemblies, which are all sweepable after D2. It is kept for P1 completeness.

## Tasks

Ownership: one worker per row. New crates, their registration in `crates/sm-server/src/standard.rs:23-33` and the root `Cargo.toml` are the orchestrator's.

| Task | Owner | Size | Depends |
|---|---|---|---|
| W4-HEX-STRAT | `crates/sm-mesher/` | S | none |
| W4-HEX-DECOMP | new `crates/sm-decomp/` | M | none (spike first) |
| W4-HEX-QUERY | `crates/sm-mesher/` (query `mesh.decompose_plan`) | S | DECOMP, STRAT (same crate, sequential) |
| W4-HEX-OGRID-K | new `crates/sm-scheme-ogrid/` (kernel) | M | none |
| W4-HEX-OGRID-S | `crates/sm-scheme-ogrid/` (adapter, registration, scores) | M | OGRID-K |
| W4-HEX-PIN-INT | `crates/sm-cli/tests/`, `tools/` (new files) | M | STRAT, QUERY, OGRID-S |
| W4-HEX-VOLFIT | `crates/sm-mesher/` (op), only if Q2 says yes | S | PIN-INT |
| W4-HEX-MSWEEP | `crates/sm-scheme-sweep/` | L | ACR (R2) for true many-to-one |
| W4-HEX-PYR | `crates/sm-scheme-tet/` | L | spike (R3) |
| W4-HEX-VSUBMAP | `crates/sm-scheme-mapvol/` (+ reuse of submap lattice code) | L | none |

**Critique delta:** STRAT and QUERY both own `crates/sm-mesher/`, so they run in sequence (QUERY after STRAT), not in parallel. VOLFIT is new (Q2).

### W4-HEX-STRAT (S)
- **Scope:** add a name and description to `DefaultStrategy` (F7), register `hex_only` and `tet` (D3) in `StrategyRegistry::default()` (`auto.rs:424-433`), and add a no-warning setting for `tet`. Update the `auto.rs:591-592` hint to name `hex_only`. Add a per-volume mesh volume to `mesh.info` (`volume_by_owner: BTreeMap<EntityId, f64>`, sorted, deterministic; `info.rs:49` has only the total). **Critique delta:** the per-volume volume is new. PIN-INT acceptance 1 and OGRID-S acceptance 1 cannot be checked from the script without it.
- **Acceptance:**
  1. `mesh.auto_strategies` (`auto.rs:609-652`) lists the three names, ascending, with descriptions. **Critique delta:** the query is `mesh.auto_strategies`, not `mesh.strategies`.
  2. Brick (B1) and L-slab (B2): `hex_only` assignments equal `default`'s (compare assignment lists, not state hashes; the stored op carries the strategy name, `auto.rs:598`). Through pin and B5: equal except `ogrid` replaces `pave` on disk and annulus faces, while OGRID-S is not merged or Q1 is "no".
  3. Enclosed pin with `hex_only`: `mesh.auto` fails, naming the moderator volume id, with `tetmesh` absent from the tried list and the per-candidate reasons; the timeline is unchanged (the op is rejected).
  4. `tet` on a brick: all volumes `tetmesh`, all faces `trimesh`, all cells Tet, zero `mesh.auto_tet` warnings, mesh volume equal to the brick to 1e-12.
  5. Replay of a recorded `hex_only` history reproduces the state hash with the strategy removed from the registry. (This repeats the W3b M2 proof for a new name; it is cheap and kept.)
  6. `mesh.info.volume_by_owner` sums to `volume` to 1e-12 on the through pin, and is equal for 1 and N threads.
- **Hash impact:** none on state. The `mesh.info` reply gains a field, so the MCP golden `crates/sm-mcp/tests/golden/tools.standard.txt` changes if it records output schemas (orchestrator updates it).

### W4-HEX-DECOMP (M)
- **Scope:**
  - Step 0, the spike: re-run `cyl.smesh` and `cyl3.smesh` (F3) at d2f4803 and record the result in the task report.
  - Then the pure planner `plan_cuts(topology view, geometry, volume) -> Vec<CutStep>` (D2). Each `CutStep` serializes as the arguments of `model.webcut_cylinder` or `model.webcut_plane`, with targets that are ids or `{step, role}` references. Axis-aligned cavity cylinders first.
- **Acceptance:**
  1. Enclosed pin: applying the steps, then imprint and merge, gives 6 volumes (three square-annulus slabs, two caps, fuel). `mesh.auto default` then gives all Hex with 0 `mesh.validate` violations. The spike gave `{Hex: 80}` at size 0.3; the test pins the count measured at d2f4803.
  2. Volume is preserved to 1e-12 across the cuts (sum of `model.measure` before and after; the spike gives 1.5876 to 4e-15).
  3. A volume that already sweeps returns an empty list, so `hex_only` never edits a sweepable model.
  4. Two cavities at different heights, a stepped axial stack and an off-axis cavity: either the right cuts or an explicit "no decomposition found" with a reason, never a wrong cut. "Right" means acceptance 1 holds for the result.
  5. The output is deterministic, steps in a fixed order (cylinders by axis position, then planes ascending along the axis).
- **Hash impact:** none.

### W4-HEX-QUERY (S)
- **Scope:** the `mesh.decompose_plan` query in `sm-mesher` (read-only, like `mesh.plan`), and the `hex_only` error text using it.
- **Acceptance:** the query on the enclosed pin returns the same steps as DECOMP. Calling it leaves the state hash unchanged. Applying the returned steps in order through the CLI, MCP and Python gives equal state hashes. The JSON schema appears in the MCP tool golden (`crates/sm-mcp/tests/golden/tools.standard.txt`, updated by the orchestrator).
- **Hash impact:** none.

### W4-HEX-OGRID-K (M)
- **Scope:** pure functions `ogrid_disk(n, a, b, k, ratio, phase)`, `ogrid_annulus_quad(...)`, `ring(n, k, ...)` and `polar_disk(n, k, ratio)` (D1 alternative); no topology types. `collar` is deferred (D1).
- **Acceptance (exact or analytic):**
  1. Disk, n in {8, 12, 16, 20, 40}: quad count equals `a*b + k*n`; Euler characteristic V - E + F = 1. **Critique delta:** the draft's broken formula text ("hmm per block sum") is replaced.
  2. Exactly four interior nodes of valence 3, none of valence 5 or more.
  3. Total area equals the inscribed polygon area `(n/2) r^2 sin(2 pi / n)` to 1e-12 relative (nodes on the circle).
  4. For n % 4 == 0 and phase 45 degrees, the node set is invariant under the 90-degree rotation and both axis reflections, to 1e-12.
  5. With `a = b` and phase 45 degrees, the minimum scaled Jacobian at a central block corner equals sin(135 deg) within 1e-9 before smoothing. After the kernel's smoothing it is at least 0.7 (the measured value replaces 0.7 after the spike).
  6. Annulus with a square outer loop: area equals `L^2 - (n/2) r^2 sin(2 pi / n)`. No inverted quad for radius ratios 0.1..0.9 (property test with `sm_testkit::strategies::config(n)`).
  7. `polar_disk`: one n-gon plus `k*n` quads; area as in 3; symmetric under every reflection of the regular n-gon for even n.
  8. Output hash identical for `RAYON_NUM_THREADS=1` and default (determinism rule).
- **Hash impact:** none (new crate).

### W4-HEX-OGRID-S (M)
- **Scope:** `Scheme` impls `ogrid` (params `mode: "ogrid" | "polar"`, `rings`, `ratio`, `phase`), the register function, and applicability only for exact disks and annuli (score 70). Caps `QUAD | STRUCTURED | BLOCK_STRUCTURED` for `ogrid` and `QUAD | POLY` for `polar`. Interval rows as in D1: `EvenSum` on the circle and one `Linear` row on the square annulus.
  - **Critique delta:** the draft gave "requirements QUAD | BLOCK_STRUCTURED so a sweep accepts the face". Sweep places that requirement only on linking faces (`sweep/src/scheme.rs:231-239`). A sweep source needs no capability, so this is a caps choice, not a requirement.
- **Acceptance:**
  1. A cylinder volume and a through-pin moderator mesh as hex through `sweep`, with an `ogrid` source and a `copy` target. The fuel mesh volume equals `h * (n/2) r^2 sin(2 pi / n)` to 1e-12 relative, with n read from `mesh.plan`. `mesh.validate` reports 0 violations, and the pin and the lattice are conformal at every shared circle.
  2. An odd interval request on the circle: the solver gives an even count (EvenSum), never an error.
  3. `mesh.plan` names `ogrid` for the fuel disk and the annulus. Pave stays chosen for a square with a rectangular hole (not applicable).
  4. Determinism test on the lattice: two runs, 1 vs N threads, hashes equal.
- **Hash impact:** none until the scheme is in a strategy list. If it enters `default` (Q1), the items in R1 move.

### W4-HEX-PIN-INT (M)
- **Scope:** new scripts `tools/w4_pin_ogrid.smesh` (through pin and lattice, `hex_only`) and `tools/w4_pin_enclosed_hex.smesh` (the 3 cut steps from `mesh.decompose_plan`, then `hex_only`), with tests in `crates/sm-cli/tests/milestone_w4_pin.rs`. The W3c pin scripts stay as they are until Q1 is answered.
- **Acceptance:**
  1. Through pin: all Hex. The cell count equals the O-grid formula for the intervals in `mesh.plan`. The fuel volume per pin, from `mesh.info.volume_by_owner`, equals `h (n/2) r^2 sin(2 pi / n)` to 1e-12 relative (or `pi r^2 h` to 1e-12 if VOLFIT is approved, Q2). This closes F2 for the new script.
  2. Enclosed pin: 6 volumes after the cuts, all Hex, no `mesh.auto_tet`, 0 `mesh.validate` violations, the same fuel-volume check. The `.smrt` and OpenFOAM exports have no warnings.
  3. In-process, CLI, MCP and Python hashes are equal (as in the W3c milestone).
  4. Both hashes are pinned in `tools/platform/expected.txt` (C12) on macOS arm64 and Linux CI.
  5. Mirror symmetry: the lattice mesh maps onto itself under the x and y reflections through the lattice centre (node-set test, 1e-12 absolute). This needs n % 4 == 0 on every circle; the script fixes the circle intervals so that it holds.
- **Hash impact:** new files only.

### W4-HEX-VOLFIT (S, only if Q2 says yes)
- **Scope:** an explicit, opt-in op `mesh.preserve_volume {mesh, curves | surfaces}`. It moves the nodes classified on the given circles and cylinder faces radially, from r to `r' = r sqrt((2 pi/n) / sin(2 pi/n))`, so that the linear n-gon has the area of the circle. Both sides of an interface move together, so the mesh stays conformal. The op is recorded in history, replays without a strategy, and is deterministic. MOOSE's reactor module has a `preserve_volumes` option on its concentric-circle generators for the same reason (not verified here).
- **Acceptance:**
  - Through pin: fuel volume `pi r^2 h` to 1e-12 relative, moderator `(1.26^2 - pi r^2) h`, total unchanged.
  - 0 `mesh.validate` violations and no inverted cell.
  - `.smrt` region volumes, checked by a ray-sampled volume estimate against OpenMC (conda env openmc), within the sampling error.
- **Hash impact:** none (new op, opt-in).

### W4-HEX-MSWEEP (L)
- **Scope:** `source`/`target` as lists (backward-compatible JSON), set-aware discovery, many-to-many on imprinted faces (D4), then true many-to-one with a union copy if the ACR is approved. Optional: columns with intermediate vertices (D2 alternative), if the user wants fewer cut steps.
- **Acceptance:**
  1. One slab volume whose bottom and top faces are each split into two faces by an imprinted line (no internal face): all hex. Cells and volume equal those of the same slab split into two volumes and swept 1-to-1: the same node set, volume to 1e-12.
  2. Existing `tests/bricks.rs`, `curved.rs` and `strip.rs` are unchanged. The census (`crates/sm-mesher/src/census.rs`) shows that no existing sweep choice changes (C3: any moved hash stops the task).
  3. True many-to-one: a block with a step on the top (one face on the bottom, two on top). Cells are conformal on the step edge, the volume is exact, all hex.
  4. The error text for an unsupported set names the face ids and the missing correspondence.
- **Hash impact:** none for 1-to-1 inputs; the census is the first step, as in CYL-SCHEMES.

### W4-HEX-PYR (L)
- **Scope:** the `transition` param on `tetmesh` (D5), the apex rule, the Soft TRI requirement, the frozen pyramid shell fed to `tetrahedralize`, and `Caps::PYRAMID` in the produced caps.
- **Acceptance:**
  1. A unit cube meshed as `a x a x a` hexes under a `tetmesh` box on its top face: the pyramid count equals `a^2`. Each pyramid volume equals `s^2 h / 3` to 1e-12 (planar square base of edge `s`, height `h`). The total volume equals the sum of the two boxes.
  2. `mesh.validate` reports 0 violations (every interface face is shared by two cells). The sm-quality signed pyramid scaled Jacobian is positive for every pyramid.
  3. With `transition: "none"` the plan is today's (plan hash equal), including the existing conflict message.
  4. Exports: Gmsh, VTU, OpenFOAM and Exodus round-trip the pyramid cells (existing writers; add a row each).
  5. A curved quad base (cylinder lateral face): the apex is not placed inside a neighbour; a failure falls through the replan loop, not a panic.
  6. B6 rerun with `transition: "pyramid"`: the hex neighbour keeps all its hexes (no wedge layer).
- **Hash impact:** none while the default is `none`.

### W4-HEX-VSUBMAP (L)
- **Scope:** D6. Polycube lattice, validation, block decomposition, 3D interval rows, generation as map blocks; `mapvol` keeps the six-face path.
- **Acceptance:** a cube minus a corner cube (not 1-to-1 sweepable):
  - 3 boxes; the cell count equals the sum of the products of intervals; the volume is exactly 7/8 of the cube.
  - All hex, conformal interfaces; every interior mesh edge has 4 hexes and the re-entrant boundary edge has 3.
  - Determinism test.
- **Hash impact:** none.

## Priority (for neutron transport and ray tracing)

- **Now (first W4 sub-wave), in this order:**
  1. DECOMP (with its spike), STRAT, QUERY. These alone close C1 (all-hex enclosed pin with today's pave sources) and C5.
  2. OGRID-K, OGRID-S: regular, symmetric rings for the transport code.
  3. PIN-INT.
  4. VOLFIT if Q2 approves it.
  - None of these changes a frozen contract or adds a cell type. All-hex is already polyhedral-compatible for an FV code.
- **Critique delta:** the draft said this work "does not touch the ray-tracing path". It does. The `.smrt` export is a mesh tracker over the volume mesh (F2), so its fuel volume is the n-gon's, and regular hex cells change the walk cost. This makes Q2 (fuel volume) relevant to both of the user's goals. It also means DECOMP is worth more than OGRID for C1 and should come first.
- **Later W4 sub-wave:** MSWEEP many-to-many (no ACR); then PYR, needed as soon as a model has a hex region next to a tet region (for example a pin with a complex grid spacer); then true many-to-one after the ACR decision; then `collar`.
- **W5+:** VSUBMAP (pins, assemblies and axial stacks are sweepable once D2 exists; polycubes are rare in reactor models), a general automatic multisweep decomposer, the sphere O-grid (P2), high-order elements.

## Risks

- **R1. Hash movement (C3, C12).** If `ogrid` enters `default`, these move:
  - `tools/platform/expected.txt:54-63` (`w3c_pin`: `h_auto`, `h_mesh`, `state`, `layer.*`, `cells`);
  - `milestone_w3c_pin.rs:342-343` (432 hexes);
  - `crates/sm-server/tests/auto_select.rs:296` (B5 plate with a hole, `{"Hex": 154}`; its outer loop is four-sided, so `ogrid` applies);
  - the `tools/w3c_pin_leftout.smesh` front-end comparison.
  - The enclosed pin row (`milestone_w3c_pin.rs:617-621`) does not move: its moderator still has no sweep under `default`.
  - **Critique delta:** this concrete list replaces "faces with circles inside the W3b milestone would change". No other `tools/*.smesh` creates a cylinder. The census must still prove that W2/W3a/W3b hashes stay, run first, as CYL-SCHEMES did.
- **R2. Frozen contract for many-to-one.** A union copy needs `crates/sm-scheme-api` (`copy.rs`). The task returns `blocked_on_api` with the request; many-to-many works without it.
- **R3. Pyramid planning and generation.** The planning half is settled by the code: a Soft TRI yields to a Hard QUAD in the same `resolve` (`planner.rs:1310-1322`). What remains is generation: can the tet scheme read which bounding faces were meshed as quads, and their nodes, from the earlier waves' patches through `GenCtx`, and give one volume mixed TRI/QUAD faces? If not, an ACR to `sm-scheme-api` is needed. The first PYR step is a half-day spike with one test; the result decides. **Critique delta:** the scope moved from the planner to generation.
- **R4. Pyramid quality and robustness.** A pyramid on a strongly curved quad can be thin, and the neighbour tets can be flat when `h` is small. The spike measures dihedral angles and scaled Jacobian before the apex rule is fixed.
- **R5. Linear-hex volume error (F2).** The O-grid does not remove it. It reaches both the transport mesh and the `.smrt` model. The fix is VOLFIT (Q2) or, later, high-order elements; curvature sizing only reduces it. The milestone row is honest only when it checks the fuel volume.
- **R6. Circle phase.** If the 45-degree corners do not fall on a node (n % 8 != 0), the radial lines to the four square corners are slightly skewed and the rotation symmetry is lost. This is acceptable: the scaled Jacobian is measured and gated, and PIN-INT fixes n. A node-placing `phase` on the curve scheme (`crates/sm-scheme-curve`, not frozen) removes the skew later.
- **R7. Decomposition is heuristic.** DECOMP covers axisymmetric cavities only and says so when it cannot decompose.
- **R8. A partial decomposition fails instead of falling back (new, observed in the spike).** After the cylinder cut alone, `mesh.auto default` errors on the two-loop moderator end face instead of falling back to tets (F3). DECOMP never emits a partial cut set. The behaviour itself is recorded for the backlog; it was not diagnosed further here.
- **R9. Spike binary age (new).** The spike used a binary built before the W3c merge (it has no W3c strip check). DECOMP step 0 repeats it at d2f4803 before anything is built on it.

## Questions for the user

1. **Does `ogrid` enter `default` now?** If yes, the through pin, the lattice and their `tools/platform/expected.txt` rows change on purpose (new count, new hashes), and so do B5's 154-hex assertion and the leftout fixture (R1). W2/W3a/W3b hashes stay. If no, only `hex_only` uses it and `default` stays as in W3c. *Recommendation:* yes, as one explicit, signed-off re-bless of the R1 list only. The pin is the user's own example, and its mesh is the thing being improved; the C3 census guards everything else.
2. **Volume fidelity of round regions with linear cells.** At about 8 intervals the mesh under-fills a fuel disk by about 10% (2.6% at 16, 0.6% at 32), in the transport mesh and in the `.smrt` model alike (F2). What relative fuel-volume error is acceptable? Options:
   - (a) accept it, and only refine the circle (n >= 32 for < 1%);
   - (b) the opt-in VOLFIT op, which makes each material's volume exact at any n by moving the circle nodes to the equal-area radius (still conformal, nodes no longer on the B-rep circle);
   - (c) wait for high-order elements.
   *Recommendation:* (b) as an opt-in now (task S), default off so no hash moves, plus the fuel-volume check in the new milestone. **Critique delta:** the draft argued that moving nodes off the circle breaks conformity. It does not: both sides share the moved nodes. What it gives up is that nodes lie on the geometry, which an FV transport code does not need. The choice is the user's.
3. **Cell shapes accepted by the transport code.** Does it take pyramids (5-face polyhedra) in a mixed mesh? Or only hex and tet (then a transition needs a different construction)? Or only polyhedra from a dual? *Recommendation:* pyramids; they are conforming for FE and plain polyhedra for FV.
4. **Decomposition as suggested ops.** Is it acceptable that `hex_only` fails with a cut list, and the user (or a script) applies it, instead of `mesh.auto` cutting the model itself? *Recommendation:* yes. It keeps the history explicit and replay free of strategies (the rule from W3b Q5). A CLI/MCP `--apply` convenience can call the same ops. The enclosed pin needs three cuts, not one (F3).
5. **ACR for true many-to-one sweep.** Approve a union-copy extension in `sm-scheme-api`, or restrict W4 to many-to-many with imprint? *Recommendation:* approve the ACR but schedule it in the later sub-wave; the pin work does not need it.
6. **(New) Pin cross-section shape for the transport code.** The choice is between an all-quad O-grid (five blocks, four valence-3 nodes, 4-fold symmetric only when n % 4 == 0, works for FE too) and a polar mesh (rings x sectors with one central n-gon prism, symmetric for every even n, fewer cells, FV/polyhedral only). *Recommendation:* build both modes in OGRID-K (small extra cost). Use `ogrid` in `hex_only` and in the milestone, and let the user's transport scripts choose `polar`.

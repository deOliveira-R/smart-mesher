# W4 planning draft: mesh modification (conversions, copy, refinement, boundary layers, high order)

> Planning draft for wave W4, area "meshops". Repo at `d2f4803`. Paths are relative to the repo root. Cubit behaviour is quoted only from `docs/PARITY.md` and `docs/research/cubit-schemes.md`. The Cubit manual pages for modification were never read (the PARITY.md header says so, `docs/PARITY.md:6`). "Better than Cubit" below therefore means "better than the PARITY list requires". It is not a claim about Cubit internals.
>
> Corrected by the planning critique. Changes against the original draft are marked **Critique delta**.

## Findings (what is true today)

**There is no modification crate.**
- `sm-modify` is in the architecture (`docs/ARCHITECTURE.md:116`: "refine, coarsen, pillow, collapse, remesh, align, mesh boolean/cleanup") but not in `crates/`. No op named `mesh.refine`, `mesh.htet`, `mesh.copy` etc. exists. The registered `mesh.*` ops are:
  - size, intervals, scheme, default_size, auto, run and delete (`crates/sm-mesher/src/lib.rs:9`);
  - `mesh.smooth` (sm-smooth) and `mesh.to_poly` (sm-poly);
  - the diagnostics (sm-diag);
  - the `mesh.exo`, `mesh.smsh` and `mesh.vtu` exports.
- PARITY C lists the work:
  - conversion "P1 THex, QTri, HTet, TQuad" (`docs/PARITY.md:75`);
  - element types "P1 HEX20/27, TET10, TRI6, QUAD8/9, WEDGE15, PYRAMID5/13" (`:79`);
  - "Modification (P2)" (`:88-89`);
  - "Boundary layers (P1)" (`:90`).
  
  PLAN lists them all in W4 (`docs/PLAN.md:74-75`). **Critique delta:** line numbers added.

**A pattern for derived meshes exists.**
- `mesh.to_poly` is the model (`crates/sm-poly/src/ops.rs:67-110`). It reads the source `MeshRead` and builds a new mesh through `MeshBuild`. It inserts the result as a new entry with `derived_from`, `keep_source` true by default, and `record_successors`.
- The scheme API says conversions are "IR -> IR mesh passes over `MeshRead -> MeshBuild`", not `MeshTransform`s (`crates/sm-scheme-api/src/scheme.rs:735-741`). `MeshTransform` is for topology-preserving edits only. The architecture line "Conversion schemes (THex, QTri, HTet) implement `MeshTransform`" (`docs/ARCHITECTURE.md:161-162`) is out of date. **Critique delta:** the authoritative statement is `crates/sm-scheme-api/DESIGN.md:173`. It says conversions "live in the mesh-ops crates", carry classification over, classify new nodes on the entity of the split face, edge or cell, and **re-verify the deck pairs on the result: the conversion must split paired deck faces identically**. The original draft missed that last requirement; see D2 and D3.
- `docs/research/cubit-schemes.md:18` records that THex must convert all merged volumes in one call. Our pass works on one mesh entry whose selection covers the merged volumes, so this holds by construction.

**Hierarchy and copy machinery.** **Critique delta:** the original heading said "exists in the API only". That was wrong: an implementation and a file format exist.
- The API (`crates/sm-mesh-api/src/hierarchy.rs`):
  - `LevelMap` is a fine-to-coarse cell map with an optional face map for CMFD surface currents (`:1-8, 20, 33-44`).
  - `ExtrusionMap` is at `:312`.
  - `MeshCatalog` and `CatalogEdit::link` are at `:481-515`.
  - The stale check compares ids and **connectivity** hashes only (`:132-146`, `validate.rs:1335-1339`). Moving points (a smooth) does not make a map stale.
- **Critique delta:** `crates/sm-mesh/src/catalog.rs:14-21` implements `MeshCatalog`/`CatalogEdit`, with level maps in a `BTreeMap`. `.smsh` already writes and reads level maps:
  - writer: `crates/sm-meshio/src/smsh/writer.rs:33,59,311-316`;
  - reader: `reader.rs:144,541-552`;
  - format: `format.rs:306`.
- The gap is only the state. The `meshes` layer (`crates/sm-layer-api/src/mesh.rs:16-41`, `MeshEntry` is `#[non_exhaustive]`) stores no link. Nothing outside sm-mesh-api, sm-mesh and the `.smsh` code names `LevelMap`. So no op can record a level link today (ACR-MOD1 below). The `mesh.smsh` export (`crates/sm-fem/src/export.rs:342`) passes no level map to the writer.
- `InstancedView`, `flatten` and `unfold` (`crates/sm-mesh-api/src/instance.rs:1-19, 710, 760`) place instances and weld across instances only. **Critique delta:** `flatten` carries cells, vertices, faces, classification and linear geometry. It does **not** carry sets, tags, the edge table or face curves: each becomes a `LossNote` (`instance.rs:15-19,160`). Mirroring or welding high-order cells is `Unsupported` (`:274`). No op uses `flatten` or `unfold`.
- `MeshBuild::set_cell_parent` and `set_cell_active` are at `crates/sm-mesh-api/src/build.rs:233-235`. `MeshRead::cell_parent` and `cell_active` are at `read.rs:190-192`. The doc says the second is for "a parent retained inactive after refinement". Face kind `NonConformal` exists "for hanging nodes, sliding meshes" (`index.rs:251`).
- **Critique delta (consumer audit).** The original said the consumers of `cell_active` were "not audited". They are, and most refuse:
  - sm-rt returns `Unsupported("mesh with inactive (refined) cells")` (`crates/sm-rt/src/build.rs:49-51`);
  - the Gmsh writer refuses with "extract the leaf mesh first" (`crates/sm-meshio/src/gmsh/write.rs:296-298`);
  - the OpenFOAM writer refuses with `InactiveCells` (`foam/build.rs:77-80`);
  - Exodus writes active cells only (`crates/sm-exodus/src/plan.rs:91-94`);
  - VTU adds an `active` array (`vtu.rs:328-329`);
  - sm-quality gives NaN for inactive cells (`crates/sm-quality/src/fv.rs:71,399`);
  - validate checks lineage (`validate.rs:767`).
- `model.copy` has no lineage link from copy to source beyond `Created` (`crates/sm-model/src/copy.rs:10`).
- **Critique delta (copy already exists in the mesher).** The original said the sweep's `target_mode: "copy"` (`crates/sm-scheme-sweep/src/fit.rs:183`) is the only mesh copy today. That is wrong. sm-scheme-api has a built-in copy generator at curve and surface level (`crates/sm-scheme-api/src/copy.rs:1-4, 304-308`):
  - `CopyConstraint {source, target, transform, vertex_map}`, where an empty `vertex_map` means the correspondence is found geometrically from the transform (`problem.rs:307-332`);
  - reasons `Deck`, `SweepTarget`, `User` and `Closure` (`problem.rs:277-293`);
  - planner copy classes run by the driver (`driver.rs:314-325`, `plan.rs:46-51,138`).
  
  No op produces a `User` copy, and there is no volume-level copy. PARITY lists copy as a curve, surface and volume *scheme* (`docs/PARITY.md:60,66,71`).

**High-order geometry is in the data model, not in any producer or consumer.**
- The API (`crates/sm-mesh-api/src/highorder.rs`):
  - degree 2 for every template shape, with bases Lagrange equispaced, GLL and Serendipity (`:17-35, 88-110`);
  - node order follows Gmsh (`:6-13`);
  - degree above 2 is unsupported (`node_count` returns `None`).
  
  **Critique delta:** Lagrange degree 2 gives PYRAMID14 and WEDGE18 (`node_count`: corners + edges + quad faces), not only the Serendipity 13 and 15. Edits exist: `add_extra_node`, `set_high_order`, `set_face_curve` (`build.rs:461-485`).
- `.smsh` stores and reads it (`crates/sm-meshio/src/smsh/writer.rs:225-238`, `reader.rs:445-466`). Its element-name table has HEX20, HEX27 and TET10 (`smsh/format.rs:247-252`).
- Everything else drops or refuses it:
  - Exodus: error `Unsupported("high-order cells")` (`crates/sm-exodus/src/plan.rs:96-98`).
  - VTU: writes the linear corners and leaves the extra nodes as unused points (`crates/sm-meshio/src/vtu.rs:15-16`).
  - Gmsh: loss item `msh.high_order_dropped`, Degraded (`gmsh/write.rs:325-335`).
  - OpenFOAM: `foam.high_order_dropped` (`foam/build.rs:425-427`).
  - The smoother refuses (`crates/sm-smooth/src/prep.rs:263,279,305`).
  - sm-rt never mentions high order (grep of `crates/sm-rt/src`), so it uses the linear mesh.
- `sm_fem::ElementType` names HEX20, HEX27, TET10, WEDGE15, PYRAMID13, QUAD8 and TRI6, each "not supported yet" (`crates/sm-fem/src/element.rs:34-54`). **Critique delta:** it has no QUAD9, WEDGE18 or PYRAMID14. Nothing generates a high-order mesh.

**Inputs a modification op needs are there.**
- Classification and sets carry through the builder (`MeshBuild::set_class`, `add_set`, `set_tag` in `crates/sm-mesh-api/src/build.rs:465-509`; sets in `sets.rs`). **Critique delta:** there is no `edit.rs`. Blocks and sidesets lower to mesh sets by volume or face colour (`crates/sm-fem/src/lower.rs`), so a converted mesh that keeps classification keeps its blocks.
- `mesh.flag_cells` stores offending cells as a group "so follow-up ops (smooth, refine) can target them" (`docs/design/w3/backlog.md:34`). `TagColumn` carries per-cell values (`read.rs:207-209`), the hook for a future error indicator.
- The polyhedral path takes tets only. `mesh.to_poly` refuses any other cell with `PolyError::NotTet` (`crates/sm-poly/src/prep.rs:237-238`; the message is at `ops.rs:59-62`). Sweep and map meshes (the W3c all-hex pin) cannot enter it today. **Critique delta:** neither can any mixed mesh, including the output of D2's polyhedral closure. This changes the HTet priority (see Priority and Q6).
- Hex-tet transition cells are a recorded W4 deferral (`docs/design/w3c/w3c-report.md:133`). **Critique delta:** the deferral is about meshing time. Per ruling C1 (`docs/design/w3c/rulings.md:9`), the enclosed pin is all-tet for now, and O-grid all-hex with a hex-tet transition goes to W4. B6 already reaches a conformal `{Tet: 160, Wedge: 32}` transition with a wedge layer (`docs/design/w3b/plan.md:310`, ruling C2), and `sm_testkit::meshes::mixed_transition` exists (`crates/sm-fem/tests/formats.rs:14`).
- **Critique delta:** median dual and agglomeration exist (`crates/sm-poly/src/agglom.rs:1-25`). Agglomeration is greedy merging of *small* cells, below a fraction of the median volume, at most 4 rounds (`options.rs:84-101`). It is a quality repair inside `dualize`. It emits no `LevelMap` and does not make a CMFD coarse level. The original claim that "coarsening in the user's sense is done" is withdrawn.
- **Critique delta:** geometry projection with parameter hints, curve-range checks and trimmed-face checks exists in `crates/sm-smooth/src/kernel.rs:247,282,384` (`curve.project`, `RejGeom`). It uses `sm_geom_api` `project` (`crates/sm-geom-api/src/surface.rs:89`). sm-poly's `TopologyResolver` (`crates/sm-poly/src/resolver.rs:43-53`) only resolves which model edge two faces share, and sm-poly does not depend on sm-geom-api (`crates/sm-poly/Cargo.toml:18-24`).

## Design

Principle: pick what serves polyhedral transport meshes and `.smrt` ray tracing, not Cubit's command list. Two facts drive it.
1. Our native cell is the polyhedron. So the hard part of Cubit-style modification, keeping a mesh conformal across a change (transition templates, pyramids, propagation), mostly disappears: a neighbour of a changed cell simply gains faces.
2. The user's solver wants hierarchies (CMFD, multigrid) and convergence studies, which Cubit's modification set does not model.

### D1. One crate, one pass shape
New crate `sm-modify` (L3, as in ARCHITECTURE). **Critique delta:** it depends on sm-mesh-api, sm-layer-api, sm-op-api, sm-model-api, sm-topo-api and sm-geom-api (the dependency set of sm-smooth, `crates/sm-smooth/Cargo.toml:13-21`), not on sm-poly's resolver. Every op is a pure `MeshRead -> MeshBuild` pass that:
- builds a new mesh entry (new ids, `derived_from`, `keep_source` true by default, `record_successors`), exactly like `mesh.to_poly`;
- keeps classification, sets (cells renamed through the cell map), tags and the edge table when it can, and reports every loss as a note with a code (the `LossNote` style of `instance.rs:160`);
- emits a `LevelMap` (child cell to parent cell) for conversions and refinements, **with a face map when one exists** (uniform refinement: every fine face between two different parents lies in exactly one parent face). **Critique delta:** without the face map the CMFD claim is half true. `LevelMap::new` already checks the face-map rules (`hierarchy.rs:33-38`);
- is deterministic: every choice depends on vertex and cell indices only, never on thread or hash order;
- **Critique delta (deck pairs):** splits every pair of periodic (deck) faces identically, as `sm-scheme-api/DESIGN.md:173` requires, and re-verifies the pairs on the result.

### D2. HTet with conformity by vertex ordering
Split hex, wedge and pyramid by the Dompierre rule (Dompierre, Labbe, Vallet, Camarero, "How to subdivide pyramids, prisms and hexahedra into tetrahedra", 8th IMR, 1999). Each quad face is cut along the diagonal through its lowest-ranked corner. Neighbouring cells agree on that diagonal without any propagation. Hex gives 5 or 6 tets, wedge 3, pyramid 2.
- **Critique delta (rank, not index):** the rule needs only a total order of vertices. Use rank = (rank of the vertex's deck root, vertex index). A vertex on the B side of a deck pair takes the rank of its A partner. The diagonals on paired faces then match without a separate pass, and the rule stays a total order. With plain vertex indices the two sides of a periodic pair generally get different diagonals, which `DESIGN.md:173` forbids.
- **Critique delta (Steiner cases):** the claim "always 5 or 6 tets" is checked, not assumed. Some quad-diagonal patterns of a hex cannot be tetrahedralised without an interior point. Whether the lowest-rank rule can produce them is settled by the exhaustive 8! test (acceptance 1). If it can, the design adds the centroid split for those cases and says so.
- **Critique delta (inversion):** a valid but distorted hex (non-planar or concave faces) can give a negative tet under a fixed diagonal choice. The op checks every tet's signed volume and refuses with a typed error naming the cell (`htet.inverted_tet`). It never writes an inverted tet.
- 2D: a quad splits along the diagonal through its lowest-ranked vertex (QTri 2-split). The 4-split (QTri, centroid fan) is a second mode.
- **Beyond Cubit (polyhedral closure).** The op takes an optional cell selection. A selected cell next to an unselected one cuts the shared quad into two triangles. The unselected neighbour gets one more face and its `cell_shape` becomes `Polyhedron` (a hex tag with 7 faces would fail `ShapeTag`/`Corners`, `validate.rs:48-51`). There are no transition pyramids and no "propagate to neighbours" rule. The mesh stays valid for the walker, Exodus NFACED and OpenFOAM. The option is `closure: "polyhedral" | "whole"` (whole = refuse unless the selection is closed).
- **Critique delta (what closure does and does not buy).** The original said closure lets "an all-hex sweep mesh have just its fuel region split to tets and then `mesh.to_poly` dualize that region only". That is false today: `to_poly` refuses any non-tet cell (`prep.rs:237-238`), so a closure result cannot enter it. The original also said closure "answers the hex-tet transition deferral for free". It does not: that deferral is about meshing time in `mesh.run` (C1), where the tet side must be meshed against a quad face. Closure is a post-hoc mechanism. If the transport path is to take hybrid meshes, the better design is outside this area: `to_poly` keeps non-tet cells as polyhedra and dualizes only the tet region. Each interface triangle is cut into its dual sub-faces and the neighbour becomes a polyhedron, the same closure idea applied inside sm-poly. That makes every W3c mixed mesh (B6 `{Tet, Wedge}`, hex pins next to tet moderator) a polyhedral transport mesh without splitting good hexes into tets. Dualizing tet-split hexes gives unsymmetric duals (see Risks). See X-1 and Q6.

### D3. Refinement as a hierarchy, not an in-place edit
`mesh.refine {mesh, cells?, levels}` creates a new independent mesh and a `LevelMap` (fine to coarse, with face map), linked in the catalog. It does not mark parents inactive inside one mesh.
- **Critique delta:** the reason is now the audit, not "not audited". sm-rt, Gmsh and OpenFOAM refuse inactive cells. Exodus silently drops parents. In-place marking would make the refined mesh unusable by the walker and two exporters until each one learns to extract leaves. Independent meshes are what `hierarchy.rs:1-8` was designed for. They give simple state hashes and multigrid and CMFD level pairs directly. `set_cell_parent` is still set on nothing: lineage lives in the `LevelMap`.
- Uniform rules:
  - tet 1 to 8 (Bey, "Tetrahedral grid refinement", Computing 55, 1995, with a consistent vertex ordering so that repeated refinement stays in at most three similarity classes; the shortest-diagonal variant is from Liu and Joe, Math. Comp. 65, 1996);
  - hex 1 to 8, wedge 1 to 8;
  - pyramid 1 to 6 pyramids + 4 tets (the standard split);
  - tri 1 to 4, quad 1 to 4.
- Polyhedral input is refused with an instruction: refine the tet source, then dualize again (the dual is derived, and `derived_from` records it). **Critique delta:** the re-dualized fine mesh is not nested in the coarse dual: dual cells of refined tets straddle coarse dual cells. So this route gives a convergence study but no `LevelMap` between the two polyhedral meshes. For a nested polyhedral hierarchy, see Q7.
- Local refinement: a selection (a `mesh.flag_cells` group or a tag-column threshold) refines those cells. A balance rule limits the level jump to 1 across a face. The neighbour's coarse face is split into the child faces, so the neighbour becomes a polyhedron. This is the OpenFOAM `hexRef8` approach. It is native for us and conformal without `NonConformal` faces. Cubit-style refine templates exist to avoid hanging nodes; we do not need them. **Critique delta:** it is not only face neighbours that change. Every face that contains a split edge gains a hanging vertex, so edge neighbours and the side faces of face neighbours become polygons with collinear vertices. `validate` accepts these: the side tets use the face centroid as apex, `validate.rs:1205-1220`. The citation "Jasak, Numerical Heat Transfer B 38, 2000" is a different paper (error estimation). Cite hexRef8 by the OpenFOAM source (`src/dynamicMesh/polyTopoChange/polyTopoChange/hexRef8`) instead.
- Boundary nodes: new edge and face-centre nodes on classified boundary entities are placed on the model entity. **Critique delta:** this uses the sm-smooth projection rule (`kernel.rs:247-282`): `project` with the parameter mean of the endpoints as hint, refused when it leaves the curve range or the trimmed face. sm-poly's resolver is not used. The projection is accepted only if no incident cell inverts (sm-quality validity). Otherwise the straight midpoint is kept and a note is emitted. This keeps volume and area close to the geometry, which matters to a transport calculation: a cylinder meshed by chords loses volume (the 6.1229 inscribed-prism value, `docs/design/w3/plan.md:491`).
- Coarsening: **Critique delta:** agglomeration does not produce a hierarchy (see Findings). A primal edge-collapse coarsener (Hoppe-style) and hex sheet coarsening are still not needed for the user goals. A CMFD coarse level is a real question (Q7).

### D4. Mesh copy
**Critique delta (design replaced).** The original design was a post-hoc `mesh.copy {mesh, transform | to_volume}` op over `flatten`. It needed a new sm-model lineage to find its target. It lost sets and tags (`instance.rs:15-19`). It left the shared faces between the copy and its non-copied neighbours to welding.

The better design reuses what the mesher already has: a **volume-level `copy` scheme** inside `mesh.run`. Today the copy generator works at curve and surface level, `CopyReason::User` already exists, and an empty `vertex_map` already means the correspondence is found from the transform (`problem.rs:307-332`, `copy.rs:1-4`). A volume copy (source volume + transform, target found by geometric correspondence) gives:
- bit-identical copies (`copy_patch` places nodes by the transform);
- conformal interfaces through the existing face and edge copy closure (`CopyReason::Closure`);
- one mesh entry with classification, so blocks and sidesets lower as usual;
- no dependency on a `model.copy` lineage;
- PARITY's volume "copy" scheme (`PARITY.md:71`).

This needs a volume `copy_patch` in sm-scheme-api (a frozen contract, so an ACR) and planner support. It belongs to the mesher/schemes area, not meshops (X-2). `InstancedView` stays for a later instance-aware `.smrt`. It is a ray-tracing feature, not a meshing one.

### D5. High order: elevate, validate, export
`mesh.elevate {mesh, degree: 2, basis, project: bool}` adds the degree-2 nodes that the API already describes for every shape (TRI6, QUAD8/9, TET10, HEX20/27, WEDGE15/18, PYRAMID13/14). Degree 3 and above stay unsupported until the API changes. **Critique delta:** basis choices whose shapes have no `ElementType` name (QUAD9, WEDGE18, PYRAMID14) need additive variants in sm-fem, or the op refuses them in v1. Recommendation: refuse them in v1 (Serendipity for wedge and pyramid, Lagrange for hex and quad only where the name exists, HEX27).
- `project: true` moves edge and face nodes on classified boundary entities onto the geometry (the D3 rule), then checks Jacobian positivity by Bezier bounds instead of sampling (Johnen, Remacle, Geuzaine, "Geometrical validity of curvilinear finite elements", J. Comput. Phys. 233, 2013). A cell that fails keeps straight edge nodes and is reported (note code `elevate.straight_kept`).
  - **Critique delta:** the bound with finite subdivision has three outcomes: valid, invalid, undecided. Undecided is treated as invalid, which is conservative.
  - **Critique delta:** pyramids need the rational treatment of Johnen and Geuzaine, "Geometrical validity of high-order triangles, quadrilaterals, tetrahedra, prisms, hexahedra and pyramids", J. Comput. Phys. 299, 2015. The 2013 paper does not cover them.
  - Cubit's high-order output has the same inversion risk with no bound-based check that we know of. We do not claim more.
- Consumers:
  - the Exodus writer accepts the Exodus node orders;
  - the Gmsh writer writes the Gmsh element types (our node order is already Gmsh, `highorder.rs:6-13`);
  - **Critique delta:** VTU writes the VTK *quadratic* types (QUADRATIC_TETRA 24, QUADRATIC_HEXAHEDRON 25, TRIQUADRATIC_HEXAHEDRON 29, QUADRATIC_WEDGE 26, QUADRATIC_PYRAMID 27, QUADRATIC_TRIANGLE 22, QUADRATIC_QUAD 23, BIQUADRATIC_QUAD 28). The original said VTK Lagrange cells; ParaView and meshio support the quadratic types better at degree 2, and the Lagrange cells only matter above degree 2, which we do not support;
  - `sm_fem::ElementType` drops "not supported yet" for the seven names;
  - `.smsh` already works.
  
  Node-order tables differ per format (HEX20 top-edge order and TET10 midside order are known pitfalls). meshio is already in the conformance environment (`docs/design/w3/w3a-report.md:11`) and is the cross-check.
- sm-smooth keeps refusing high-order meshes. The rule is smooth first, then elevate.

### D6. Boundary layers, pillow, THex, TQuad
- THex: 1 tet to 4 hexes (centroid, face centres, edge midpoints). TQuad: 1 tri to 3 quads. Always conformal (all faces are subdivided the same way), with poor angles. Cheap and exact; kept for PARITY and for 2D or hex-only solvers.
- Boundary layers: layered wedge/hex inflation by extruding the boundary mesh into the existing volume and re-attaching the interior (the interior is moved or remeshed). Useful for viscous CFD; serves none of the user goals. Extrusion layers with graded intervals already come out of sweep (`crates/sm-scheme-sweep/src/emit.rs:572-577` builds the hexes and wedges).
- Pillow (sheet insertion into doublets) repairs a hex mesh defect. W3c produced all-hex sweeps without doublets, so there is no case today.

## Tasks

Sizes are S/M/L. "Hash impact" is the effect on existing hashes. Every task adds ops, so the op and schema catalog goldens move once, in the registration step. That is not a re-bless of any mesh hash. Per the W3c rule (C3), any moved existing mesh or milestone hash stops the task.

### MOD-0 (orchestrator): skeleton and contract, S
- Owner: root `Cargo.toml`, `Cargo.lock`, the `crates/sm-modify/` skeleton (empty `lib.rs`, plugin stub), `crates/sm-layer-api`, the CI line, and wiring in the sm-server/sm-cli distribution.
- ACR-MOD1 (`sm-layer-api`, additive). **Critique delta:** no new method names; use the existing contract. `MeshLayer` implements `MeshCatalog` and `CatalogEdit::link` (`hierarchy.rs:481-515`), as `sm-mesh::Catalog` already does (`catalog.rs:14-21`):
  - links are stored in a `BTreeMap<(fine, coarse), LevelMap>`;
  - `link` enforces the existing stale-pair check;
  - `on_retired` (`mesh.rs:319`) drops links that name a retired mesh;
  - `insert_mesh`/`insert_view` return `Unsupported` (ops insert through `insert_mesh` in `mesh.rs:402`).
  
  **Hashing (decided here):** links are state that replay must reproduce, so they are hashed. They are hashed only when present: the digest appends a link section only if at least one link exists. With no links the digest is byte-identical to today's. Without ACR-MOD1, D3 cannot record the hierarchy. If it is rejected, refine ships without the link and a note says so (not recommended).
- Follow-up (sm-fem owner, S): **Critique delta:** `mesh.smsh` (`sm-fem/src/export.rs:342`) passes the links of the exported mesh to `SmshWriter::level_map` (`writer.rs:59`), so the hierarchy reaches disk.
- Acceptance:
  1. The workspace builds.
  2. A `LevelMap` link survives a timeline fork and replay with equal hashes.
  3. The layer state hash with no links equals today's on the W2, W3a, W3b and W3c milestone fixtures (no existing hash moves).
  4. **Critique delta:** a link is dropped when either mesh is retired.
  5. **Critique delta:** `link` refuses a map whose mesh was replaced with different connectivity.
- Hash impact: none on existing hashes. The layer schema listing changes.
- Depends: nothing.

### MOD-HTET (`crates/sm-modify/`): `mesh.htet`, `mesh.qtri`, M
- Scope: the D1 pass core (a reusable helper that carries classification, sets, tags, notes and the cell map); HTet for hex, wedge and pyramid by the lowest-rank rule with deck ranks; polyhedral closure; QTri 2-split and 4-split on 2D meshes.
- Acceptance (exact or analytic):
  1. Enumerate all 8! = 40320 rank orderings of a unit cube. Every result is 5 or 6 tets, the volume sum is 1 within 1e-14, and no tet has a negative volume. **Critique delta:** if some ordering needs a Steiner point, the test records which and the op uses the centroid split there. The result is pinned in the report, not assumed.
  2. Two cubes sharing a face, under 1000 relabelings drawn by `sm_testkit::strategies::config(n)`: both sides cut the shared quad along the same diagonal (face tables match), and `mesh.validate` reports 0 violations.
  3. Wedge gives 3 tets and pyramid gives 2, with volume sums equal to the cell volume to 1e-14 (wedge on the unit right triangle with height 1: 0.5; square pyramid with height 1: 1/3).
  4. A 4x4x4 hex block to tets: volume 64 exact, `mesh.validate` clean, sidesets keep their area (each boundary quad becomes 2 tris, same total area).
  5. Closure: a 2x1x1 block with only the left hex selected. The right cell has 7 faces and `cell_shape == Polyhedron`, the mesh validates, and the interface area is unchanged. `closure: "whole"` refuses with a typed error naming the cell.
  6. **Critique delta (replaces "passes through `mesh.to_poly`").** An all-hex input converted with no selection gives an all-tet mesh. That mesh passes through `mesh.to_poly` and the dual cell volumes sum to the volume. A closure (mixed) result is refused by `to_poly` with `NotTet` until X-1 lands; the test pins that refusal.
  7. QTri on a unit square and a parallelogram of area A: the 4-split gives 4 tris of area A/4 each; the 2-split gives 2 tris with total area A.
  8. **Critique delta (deck pairs).** A 2x2x2 hex block with a translation deck between x=0 and x=2, with vertex indices permuted so that plain-index Dompierre would disagree across the pair. After HTet, every pair of B-side triangles is the transform of the A-side pair (vertex map check), and the deck re-verification passes.
  9. **Critique delta (inversion).** A valid hex with one face folded (min scaled Jacobian > 0, some fixed diagonal giving a negative tet) is refused with `htet.inverted_tet` naming the cell.
  10. Determinism: two runs and 1 vs N threads give equal hashes. A property test with `sm_testkit::strategies::config(n)` over random cube relabelings.
- Hash impact: none on existing. New fixtures get new golden hashes.
- Depends: MOD-0.

### MOD-REFINE-U (`crates/sm-modify/`): uniform `mesh.refine` with `LevelMap`, M/L
- Scope: D3 uniform rules for tet, hex, wedge, pyramid, tri and quad; a per-level hierarchy with face maps; boundary-node projection with the inversion guard.
- Acceptance:
  1. Counts: an `n^3` hex block refined once has `(2n+1)^3` vertices and `8n^3` cells. The Euler characteristic of the boundary stays 2.
  2. Volumes: parallelepiped hex children are exactly V/8. Tet corner children are exactly V/8, and the four tets of the centre octahedron sum to V/2. The total is V to 1e-14.
  3. Tet similarity classes: refining a Kuhn (Freudenthal) tet three times gives at most three similarity classes, judged by sorted edge-length ratios to 1e-12 (Bey's result).
  4. `LevelMap` with face map: the map is total; there are 8 children per parent (hex, tet, wedge) and 10 per pyramid; the inverse CSR sizes check; `LevelMap::new` accepts the face map. **Critique delta (replaces "stale pair refused after a smooth"):** a smooth keeps connectivity, so the link stays valid after `mesh.smooth` of either mesh (asserted). Replacing either mesh with different connectivity, for example refining it again with `keep_source: false`, drops or refuses the link.
  5. Projection. **Critique delta (analytic, not measured rates):** an axial cylinder of radius 1 and height 2, swept with 16 circle intervals (the W3a `cylinder(1,2)` fixture of `docs/design/w3/plan.md:491`), refined k = 1, 2 times with projection on. Every new lateral node lies at r = 1 to 1e-12, and the volume equals the inscribed-prism value `(n/2) sin(2 pi/n) * 2` with `n = 16 * 2^k` to 1e-12.
  6. Sets and blocks survive. Block cell counts multiply by 8. Sidesets keep their areas, up to the projection change, which is computed per case from the same inscribed-polygon formula.
  7. Determinism and a property test. Refining the all-hex through pin (`tools/w3c_pin.smesh` milestone fixture) validates with 0 violations.
- Hash impact: none on existing; new goldens.
- Depends: MOD-0 (ACR-MOD1) and the MOD-HTET pass core.

### MOD-REFINE-L (`crates/sm-modify/`): local refinement with polyhedral hanging faces, L
- Scope: the D3 local rule (selection from a set or a tag threshold), 2:1 balance, neighbour polyhedra, planar sub-faces.
- Acceptance:
  1. Refine the centre hex of a 3x3x3 block. It gives 8 children. **Critique delta (corrected counts):**
     - the 6 face neighbours each have 9 faces (5 original + 4 split); their 4 side faces adjacent to the split face become 5-vertex polygons, and they are `Polyhedron`;
     - the 12 edge neighbours keep 6 faces, 2 of which become 5-vertex polygons (hanging edge midpoint), and they are `Polyhedron`;
     - the 8 corner neighbours are unchanged hexes;
     - the vertex count is `64 + 19`;
     - the volume is exact;
     - `mesh.validate` reports 0 violations and no face has kind `NonConformal`. The original named a check code `mesh.nonconformal_face`, which does not exist (`validate.rs:28-87`).
  2. With `balance: false`, a level jump greater than 1 across any face is a typed error. With balance on, no face has a jump above 1 (property test over random selections drawn by `sm_testkit::strategies::config(n)`).
  3. Every split face has the parent's planarity (a planar parent gives planar children to 1e-12).
  4. The `sm-rt` walker crosses the refined mesh: 10000 seeded rays, with chord sums equal to the box chord sums to 1e-12 (uses the existing `rt.check`/`rt.locate`, `crates/sm-rt/src/plugin.rs:276,333`).
  5. OpenFOAM `checkMesh` and Exodus export pass through the existing conformance (exporters own their quirks; our validator judges the mesh).
- Hash impact: none on existing.
- Depends: MOD-REFINE-U.

### MOD-COPY: moved out of this area
**Critique delta.** Replaced by X-2 (volume copy scheme in the mesher; see D4). No meshops task.

### MOD-ELEVATE (`crates/sm-modify/`, plus `sm-meshio`, `sm-exodus`, `sm-fem`): high order, L
- Scope: D5. The owners split: sm-modify for the op and the validity check; sm-exodus, sm-meshio (VTU, Gmsh) and sm-fem (`ElementType`) by their owners, in separate sub-tasks with their own W4 plan rows. **Critique delta:** this is four tasks, not one L. Size the sm-modify part M and each exporter part S.
- Acceptance:
  1. A straight-sided mesh elevated with `project: false` has every extra node at the exact edge midpoint (1e-15). Every cell volume equals its linear volume (Gauss quadrature of the Jacobian, 1e-13). The node count per shape equals `node_count` (TET10 10, HEX20 20, HEX27 27, WEDGE15 15, PYRAMID13 13, TRI6 6, QUAD8 8).
  2. Curved case. **Critique delta (analytic, not measured order):** the swept cylinder of MOD-REFINE-U item 5 with n circle intervals, elevated with `project: true`. Boundary edge nodes are at radius 1 to 1e-12. The volume equals `2 * [ (n/2) sin(2 pi/n) + n * (4/3) sin(pi/n) (1 - cos(pi/n)) ]` (inscribed polygon plus n parabolic segments) to 1e-12, for n = 16 and 32. The error against `2 pi` therefore falls as n^-4; the original estimate of "order about 3" was wrong.
  3. Validity: a deliberately inverted TET10 (an edge node pushed through the opposite face) is reported invalid. Property test over small random perturbations, drawn by `sm_testkit::strategies::config(n)`, with dense 5^3 Jacobian sampling as the oracle. **Critique delta:** the bound is never "valid" when a sample is negative. Cells with sampled min J/J_avg >= 1e-2 are classified valid within subdivision depth 6. Others may be "undecided", reported and counted.
  4. Exporters: Exodus writes the seven types in the Exodus node order. Gmsh writes the Gmsh types. VTU writes the VTK quadratic types. All three are re-read by meshio (conformance, `SM_CONFORMANCE=1`) with equal coordinates and connectivity after meshio's own permutation. `msh.high_order_dropped` and `foam.high_order_dropped` no longer appear for the Gmsh export of elevated cells; OpenFOAM keeps the note (it has no high order).
  5. The `.smsh` round trip (already supported) stays bit-equal.
- Hash impact: exporters' output hashes for linear meshes unchanged (no existing golden moves). `ElementType` serde names unchanged.
- Depends: MOD-0; the exporter changes need their crates' owners.

### MOD-CONV2 (`crates/sm-modify/`): `mesh.thex`, `mesh.tquad`, S
- Scope: THex (tet to 4 hexes) and TQuad (tri to 3 quads), D6.
- Acceptance:
  1. Each tet gives 4 hexes of volume exactly V/4 for any tet (affine map of the regular case, 1e-14).
  2. Each tri gives 3 quads of area A/3.
  3. The result is conformal on a 3x3x3 tet block and `mesh.validate` is clean.
  4. The scaled Jacobian of every hex is positive for a Kuhn tet.
  5. Blocks and sidesets carry over.
  6. **Critique delta:** deck pairs are split identically (THex subdivides every face the same way, so this holds by construction; the test checks it).
- Hash impact: none.
- Depends: the MOD-HTET core.

### Cross-area items (not owned by meshops; for the orchestrator)
- **X-1 (sm-poly): `mesh.to_poly` on mixed meshes.** **Critique delta, new.** Dualize the tet region. Keep non-tet cells as polyhedra, with each interface triangle cut into its dual sub-faces. M. This is what lets W3c's hybrid and hex meshes enter the polyhedral transport path, the user's first goal. See Q6.
- **X-2 (sm-scheme-api ACR + sm-mesher): volume `copy` scheme.** **Critique delta, new.** Replaces MOD-COPY (D4). M.

### MOD-BL, MOD-PILLOW, MOD-COARSEN: not planned in W4
Boundary layers (L), pillow (M), primal and sheet coarsening (L). Each has one line of scope in Priority. Their designs are not drafted here until a trigger fires.

## Priority (for the user's goals)

The goals are polyhedral meshes for the transport code and ray tracing that beats DAGMC/XDG. MCNP is not a target; OpenMC is the oracle.

| Item | Class | Reason |
|---|---|---|
| ACR-MOD1 (level-map storage) | now | A contract change that every hierarchy feature waits on. CMFD/multigrid is the stated use in `hierarchy.rs:1-8`. Most of the machinery exists (sm-mesh catalog, `.smsh` level maps), so the change is S. |
| X-1 `to_poly` on mixed meshes (sm-poly) | now, if Q6 says yes | **Critique delta, new.** The direct route for hybrid and hex meshes into the polyhedral transport path. |
| Uniform refine with `LevelMap` | now (W4a) | Convergence studies and a nested mesh hierarchy for the transport solver. Projection keeps curved-boundary volume analytic. |
| HTet with polyhedral closure, QTri | W4b | **Critique delta (was "now").** The original reason (feeding `to_poly`) does not hold (`prep.rs:237`). What remains: PARITY P1 and the XDG tet-walk trigger (`docs/design/w3/backlog.md:107`, `smtets_to_moab`, which needs tet inputs). It moves to "now" only if the XDG tet walk is scheduled in W4a. The pass core it would have provided is built by MOD-REFINE-U instead. |
| Local refine with hanging faces | W4b | Valuable (flag cells, then refine) but L. Needs uniform refine first and benefits from the `rt` locate checks. |
| X-2 volume copy scheme (mesher area) | W4b | **Critique delta (replaces "mesh copy by instancing").** Saves meshing time and gives lattice identity. No current case is slow (the 3x3 lattice meshes today). |
| High-order elements | W4c, minimal | No transport or ray-tracing goal depends on them: the walker uses linear geometry and the transport code takes linear polyhedra. It is PARITY P1 and the data model is done, so ship `elevate` with the exporters once. The loss notes (Gmsh, OpenFOAM) keep users informed until then. Curved-boundary accuracy for transport is better served by projected refinement (D3) than by high-order cells. |
| THex, TQuad | W4c | Cheap and exact, but poor quality and no user goal. |
| Boundary layers | W5+ | Viscous CFD only; sweep layers cover extrusions. Trigger: a CFD user or a request. |
| Pillow, hex sheet coarsening, primal coarsening | W5+ | No hex-defect case yet. **Critique delta:** agglomeration does *not* give a coarse level (see Findings); a CMFD coarse level is Q7. Primal coarsening comes with adaptive remeshing from solution fields (W5+ in PLAN). |

**Critique delta (ordering of W4a tasks).** With HTet moved out of W4a, MOD-REFINE-U owns the D1 pass core. Its dependency on "MOD-HTET pass core" becomes internal, and MOD-HTET in W4b depends on MOD-REFINE-U.

## Risks

- HTet quality: splitting a flat hex gives slivers (a poor scaled Jacobian for 6-tet cubes). Mitigation: report quality in the result; the user can smooth afterwards; polyhedral closure lets users split only what needs it.
- The median dual of a hex-derived tet mesh is not the hex mesh. The dual cell shapes depend on the diagonal pattern, so a structured region gets an unsymmetric dual. Keeping hex cells as polyhedra (X-1) avoids it.
- Local refinement turns hexes into polyhedra, so tools that expect hexes (some Exodus consumers) see `NFACED`. The exporters already write polyhedra.
- Projection onto geometry can invert thin cells. It is guarded by the inversion check and the straight-midpoint fallback.
- ACR-MOD1 is a frozen-contract edit. If it is refused, the hierarchy is lost and refine is only half useful.
- **Critique delta:** a `LevelMap` stays valid after a smooth of either mesh, because its check is connectivity only. A smoothed fine mesh can have cells whose centroids leave their parent. That is fine for restriction and prolongation by index, but should be documented.
- **Critique delta:** deck pairs. Any conversion or refinement that ignores deck ranks breaks periodic models silently. Acceptance item 8 of MOD-HTET and the deck re-verification guard this.
- Node orders for high order differ across Exodus, Gmsh and VTK. The meshio cross-check is the only external oracle, and it does not cover every order.
- The op and schema catalog goldens move once on registration. A worker must not re-bless any other hash.
- Cubit behaviours are not read from the manual. Parity claims on modification rest on the PARITY list only.

## Questions for the user

1. **Hierarchy storage (ACR-MOD1).** Should refinement levels be recorded as `LevelMap` links between independent meshes (my recommendation), or as in-place marking with `cell_active`? **Critique delta (evidence):** with in-place marking, the walker (`sm-rt/src/build.rs:49-51`), the Gmsh writer and the OpenFOAM writer refuse the mesh today, and Exodus drops the parents. In-place marking would mean re-opening those four consumers. Independent meshes need none of that and give CMFD and multigrid pairs directly.
2. **Hanging nodes.** For local refinement, is it acceptable for a coarse neighbour to become a polyhedron (my recommendation: yes; conformal, one rule)? Or must the output stay all-hex with transition templates (more work, more Cubit-like, no gain for your solver)?
3. **Mesh copy.** **Critique delta (question replaced).** Should copies be made by a volume `copy` scheme inside `mesh.run`, with the target found from the transform and no `model.copy` lineage needed? That is my recommendation (it reuses the existing curve and surface copy machinery and is PARITY's form). The alternative is a separate post-hoc `mesh.copy` op over instancing, which needs a lineage change in sm-model.
4. **High-order timing.** Do you need curved or high-order output for any current case? My recommendation: no for W4a/W4b; one task group in W4c with Exodus, Gmsh and VTU export, nothing beyond degree 2.
5. **Boundary layers.** Keep them out of W4 (my recommendation) unless a CFD workflow is planned. If one is, say so now and I will draft the design (extrusion with a moved interior versus remeshing the interior).
6. **Critique delta, new: hybrid meshes in the polyhedral path.** Should `mesh.to_poly` accept mixed meshes, keeping hexes and wedges as polyhedra and dualizing only the tet region (X-1, sm-poly)? Or should your transport meshes stay either all-dual or all-native? This decides whether HTet is a pipeline step or PARITY only.
7. **Critique delta, new: CMFD coarse level.** Does your transport code need a nested coarse mesh in W4, for example one coarse cell per pin or per model volume, made by agglomerating the fine polyhedral cells, with a `LevelMap` and face map? If yes, it is an M task after ACR-MOD1. It serves CMFD directly, which neither refinement nor today's small-cell agglomeration does.

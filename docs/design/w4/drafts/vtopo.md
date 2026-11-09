# W4 VTOPO: virtual topology, healing and defeaturing, facet import

> Planning draft (W4), area vtopo. Base `d2f4803`. Standing principle (user): Cubit is a bar to surpass, not to match; MCNP is not a target; OpenMC is the CSG oracle. Literature named here is from memory and marked "verify": check each reference before a worker cites it in code.
>
> **Critique delta (Opus, overall):** I checked every file:line claim at `d2f4803`. Most hold. Four acceptance items could not pass as written: T1.1 and T2.2 (webcut composite), T1.3 and T4.2 (exact hash after remove), T5.1 (volume after a gap merge). Finding 11 was wrong, so T3b and T4.1 are void. T8 is undersized. The `view()` plumbing touches 18 files, not one. Each fix is marked inline.

Goals served: (1) polyhedral meshes for the user's transport code, (2) ray tracing (sm-rt, `.smrt`) that beats DAGMC/XDG. Both start from dirty CAD, so this area decides how much of the user's real geometry gets through the front door.

## Findings (what is true today, with file:line)

### Virtual topology is a contract with no ops and no home in state

1. **The data structure exists and is complete for faces.** `Overlay` is a `TopologyView` over a host complex (`crates/sm-topo-api/src/overlay.rs:1-48`). It has composite faces (`:593`), edge partition (`:606`), face partition by a chord (`:619`), volume split (`:637`), plane cut (`:655`), `remove` (`:814`), `rebase` against a moved host (`:385`) and a replayable record, `VirtualOp` (`:115`) / `OverlayData` (`:188`).
   **Critique delta:** composite rules that the tasks must respect (`apply_composite`, `overlay.rs:858-958`):
   - All members must be **host** faces. A virtual member is refused as `Dead` (`:866`), so a partition child cannot join a composite.
   - All members must have **identical parent rows**: same volumes, sides and shells (`:871-875`).
   - No interior edge may touch a face outside the composite (`:891-893`).
   - `face_moments` of a partition child is `Unsupported` (`:1406`), and so is `face_central` (`:1426`).
2. **Nothing owns an overlay.** `Overlay::new` and `from_data` are called only from `sm-testkit/src/topo_conformance` (grep over `crates/*/src`). `ModelLayer` holds `topo`, `model`, `next_op`, `pending` (`crates/sm-layer-api/src/model.rs:44-51`). It has no overlay field, so nothing persists, hashes or replays an overlay. `sm-mesher`, `sm-engine` and `sm-server` never mention `Overlay`.
3. **Two different "composites".** `Model::composite` (`crates/sm-model-api/src/decomposition.rs:697`) joins volume *pieces* of the decomposition. Cubit's composite surface corresponds to `Overlay::composite_faces` (`overlay.rs:593`). `Model::split_realized` takes an `&mut Overlay` that the caller must supply (`decomposition.rs:801`), and no caller exists.
   **Critique delta:** "the second is not persisted" is only half true. `Decomposition` already **persists and hashes overlay ids**: `realized: CutId -> CutCells`, `face_cells` and `split_cells` (`decomposition.rs:245-260`, module docs `:18-24`). So a model with a realized split today holds persisted ids that point into an overlay nobody keeps. There must be **one** overlay: the one in ACR-V1 is the same instance `split_realized`/`unsplit_realized` write into. Otherwise the ids diverge on replay.
4. **No op exists.** `sm-model` registers primitives, transforms, copy, delete and measure (`crates/sm-model/src/lib.rs:35`ff). `sm-boolean` registers unite, subtract, intersect, imprint, merge, merge_check and webcut_* (`crates/sm-boolean/src/ops.rs:140-593`). There is no `model.composite`, `model.partition`, `model.collapse` or `model.heal`. The only virtual-ish data in a command is `EntityKind::Partition*` in the language kind table (`crates/sm-lang/src/kinds.rs:37-40`) and the planner.
   **Critique delta:** the planner accepts `PartitionFace` only in a **selection made only of faces** (`crates/sm-scheme-api/src/planner.rs:150-152`). In the general path, `:290-292` maps `TopoFace` to `Target::Surface` and `PartitionCell` to `Target::Volume`, but a `PartitionFace` falls to `continue`. A composite face picked together with a volume is silently dropped. The planner is frozen (`sm-scheme-api`), so the fix goes into ACR-V2.
   Also: there is **no fillet or chamfer op** anywhere (no hit in `sm-model`/`sm-boolean`). Fixtures with blends must be built from booleans or a committed STEP file (see T3, T12).
5. **Composite faces cannot be meshed.**
   - A composite has `Carrier::Composite(members)` (`overlay.rs:977`) and no surface.
   - `GenCtx::loop_uvs` returns `None` for any carrier other than `Surface` (`crates/sm-scheme-api/src/scheme.rs:494-497`).
   - `face_normal` falls back to the Newell normal of the outline (`crates/sm-scheme-api/src/geom.rs:150-160`).
   - `classify_point_on_face` on a virtual face is `Unsupported` (`overlay.rs:1434-1438`).
   - `validate_geometry` skips composites (`crates/sm-topo-api/src/validate.rs:308`).
   - Area and moments do work (sum over members, `overlay.rs:1391-1401`).

   **Critique delta:** on a **periodic** surface, `loop_uvs` also returns `None` for any coedge without a pcurve. Its docs say so (`scheme.rs:476-486`) and the code does it (`:538-540`). Virtual cells have no pcurves (`overlay.rs:31`), and reader faces have none except at poles (`crates/sm-cadio/DESIGN.md:160`). This is the same root cause as the W3c deferral "`--model` for imported periodic faces (sm-cadio leaves pcurves unset)" (`docs/design/w3c/w3c-report.md:131`). Also, a composite of the two halves of a cylinder's lateral face is a **band with no seam edge** (two ring loops). The `(u,v)` schemes cannot unroll it without a seam. Host cylinders always carry a seam used twice (`crates/sm-model/src/prim.rs:119-120,205-222`).
6. **Gaps in the primitive set.** The overlay has no composite *edges* (curves) and no collapse (edge or angle). `TopologyEdit` has no collapse primitive (`crates/sm-topo-api/src/edit.rs:31-355`). Partition limits are documented: no pcurves, no lineage of its own, and `face_moments` of a partition child and `volume_moments` of a virtual volume are `Unsupported` (`overlay.rs:30-35`).
7. **The reader's four-face split is not a composite case.** The STEP reader cuts a closed NURBS face into four faces by exact knot insertion, because the face sampler cannot tell `u=lo` from `u=hi` on a seam (`crates/sm-cadio/src/split.rs:1-23`, `cut` at `:257`; DESIGN.md "Not done / limits"). The pieces are four distinct `NurbsSurface` patches, not one handle. DESIGN.md names the real fix: honour seam pcurves in the sampler (ACR 1 and 2 in `crates/sm-cadio/W3_REPORT.md:154-190`).
   **Critique delta:** the original poster case, "a webcut half-cylinder pair", **cannot be composited**. A webcut makes two volumes, so the parent rows differ and `apply_composite` refuses (`overlay.rs:873-875`). The shared seam lines also bound the webcut's cut faces, which are outside the composite (`:891-893`). Also, `regularize` re-points co-surface fragments **inside one volume** and then `join_faces` them (`crates/sm-boolean/src/regularize.rs:1-8,231-235`). So after a regularized boolean, two faces on one carrier in one volume do not survive. They survive only where the host deliberately keeps them:
   - `unite {regularize:false}` (`ops.rs:77-78,116-133`)
   - imprint
   - imported STEP faces that share one `*_SURFACE` entity (the reader maps a STEP surface id to one handle, `crates/sm-cadio/src/decode.rs:540`).

   For the last group, an **exact host join** (regularize exposed as an op, new T0) is better than a virtual composite. It changes no geometry, needs no overlay, and the existing schemes mesh the result. Stage (a) composites keep value only where the host split must stay.

### Healing and defeaturing: tolerance machinery exists, repair does not

8. **Tolerance model.** There is one global `ModelTolerance { linear, angular, parametric, linear_max }` plus sparse per-entity overrides (`crates/sm-geom-api/src/tol.rs:29-38`). This is the ARCHITECTURE §9 Q2 proposal (`docs/ARCHITECTURE.md:197`). Per-entity override edits are `raise_tolerance` (`edit.rs:323`, impl `crates/sm-topo/src/ops.rs:1603`) and `scale_tolerances` (`edit.rs:281`).
9. **Gap closing exists as `merge`.**
   - `merge_check` is a read-only analysis (`crates/sm-boolean/src/merge.rs:505-520`). Its statuses are mergeable, near (above tolerance, below `linear_max`), overlapping, conflict and skipped (`merge.rs:1-9`, `:65`). It gives a hint that names the `tol` to pass (`merge.rs:189-190`).
   - `merge` runs vertices, then edges, then faces, in ascending pair order, and the lowest id survives (`merge.rs:3-4`).
   - The reader welds vertices within `weld_tol` (default 1e-6, `crates/sm-cadio/src/model.rs:156-171`) with a sorted grid (`crates/sm-cadio/src/weld.rs:30-40`). It warns on `step.edge_gap` and fails on `OpenShell` without repairing it (`crates/sm-cadio/DESIGN.md:89-96,257`).
   - **Critique delta:** so a solid with an open shell **never reaches the model**. A separate `model.heal` op cannot be applied to it. Healing open shells needs an import-time option (T5b), or importing as a sheet (`sheets: true`, `model.rs:160-162`) followed by healing the sheet.
10. **Topology edit primitives for repair already exist** and only booleans use them: `kill_inner_loop` (`edit.rs:145`, impl `ops.rs:1031`), `join_edges` (`edit.rs:192`), `join_faces` (`edit.rs:217`), `merge_vertices/edges/faces` (`edit.rs:154-171`), `replace_curve/surface` (`edit.rs:316-319`), `set_vertex_point` (`edit.rs:306`), `record_coincidence` (`edit.rs:330`), `make_face` (`edit.rs:65`), `reverse_face` (`edit.rs:294`) and `record_merged` (`edit.rs:345`). `regularize` is the only user of the joins (`regularize.rs:146,222`), and `same_surface` (`regularize.rs:20`) is private to that module.
11. **No detector for small features.** `sm-diag` works on meshes (`validate_mesh`, `crates/sm-diag/src/validate.rs:260`). There are no queries for small curves, small surfaces, slivers, short-edge chains, blends or holes by size.
    **Critique delta (correction):** the draft said "one stray 1e-4 sliver edge drives the whole auto size". **That is wrong.** `auto_size = clamp(0.01 d 2^((f-1)/2), l_min/4, d/2)` (`crates/sm-sizing/src/auto.rs:4-12`). `l_min/4` is the **lower** bound, so a tiny edge only lowers the floor, and the size stays `0.01 d 2^…`. The real harm of a short edge is local. The mesher must put at least one segment on it, which forces elements of edge length ≤ 1e-4 next to elements of the global size: bad quality, and in tet-dual polyhedra many tiny cells. T3b is void, and T4's acceptance is rewritten to measure the mesh.
12. **Surface-surface intersection exists** (`crates/sm-geom/src/intersect/ssi.rs`, `march.rs`, `bernstein.rs`), and the boolean kernel uses it. `sm-geom/src/fit.rs` has `interpolate_points` (`:49`). That is the base for any geometric repair (extend a surface, remove a blend), but nothing calls it for repair.

### Import: STEP only; facets reach the tracker but not from a file

13. **STEP** handles `MANIFOLD_SOLID_BREP`, `BREP_WITH_VOIDS`, `FACETED_BREP` (planes and straight edges synthesized, `crates/sm-cadio/src/decode.rs:696-700`) and assemblies, plus sheets behind an off-by-default `sheets` flag (`model.rs:160-162`). `TESSELLATED_SOLID`/`TRIANGULATED_FACE` are counted as unsupported in the summary (`decode.rs:101`). Imported faces carry no pcurves except at poles (`crates/sm-cadio/DESIGN.md:160`).
14. **IGES, SAT, STL and OBJ are absent.** DESIGN.md "Not done" lists IGES and STL/OBJ, and `docs/PARITY.md:47,50` leave them open. STL is neither read nor written:
    - `session.export` with `format: "stl"` is refused (`crates/sm-server/tests/session.rs:392`; `crates/sm-cli/tests/commands.rs:717`).
    - `crates/sm-meshio/src` has no STL module.
    - ARCHITECTURE lists "STL/OBJ read+write" under `sm-cadio` (`docs/ARCHITECTURE.md:105`).
15. **The tracker already takes facets.** `FacetInput { vertices, triangles, front, back, face }` (`crates/sm-rt-api/src/input.rs:285-297`), and its docs name "imported DAGMC `.h5m`/STL" as an intended source (`input.rs:5-6`). `check_facets` groups bad edges per volume. A one-triangle hole is **one** `facet.open_edge` at the mean of the group's vertices, with `gap` = the longest open edge. It never repairs (`crates/sm-rt/src/facets.rs:80-93`). `build_facets` (`crates/sm-rt/src/build.rs:232`) refuses any issue.
    **Critique delta:** `sm-meshio` has a DAGMC *writer* (`crates/sm-meshio/src/dagmc/`). The HDF5 reader `h5/read.rs` exists only under `#[cfg(any(test, feature = "testing"))]` (`crates/sm-meshio/src/h5/mod.rs:16-17`). It reads only the subset our writer emits: superblock v2, v2 object headers, compact links, contiguous datasets, no chunking or filters (`h5/mod.rs:1-12`). `.h5m` files written by MOAB/libhdf5 typically use a v0 superblock, v1 object headers, symbol-table groups (B-tree v1 + local heap), and often chunked or deflated datasets (verify on a real MOAB file). The reader does not cover them.
16. **Mesh import is a standing decision.** `docs/PARITY.md:91`: "Mesh import: deferred by decision (we import geometry, export meshes)". `PARITY.md:47` defers mesh-based geometry with it. There is no faceted `SurfaceDef` (`crates/sm-geom-api/src/store.rs:35-51`: plane, cylinder, cone, sphere, torus, NURBS, implicit; **Critique delta:** and `EllipticCylinder`). "Mesh as geometry" for the mesher would need a contract change. **Critique delta:** `FacetInput.face` is a `Vec<EntityId>` of B-rep faces. A raw STL has no face entity, so "STL straight to `FacetInput`" still has to mint face and volume ids somewhere. The faceted B-rep in T7 is that place.

## Design

### D1. Virtual topology as recorded, reversible model state

Best design: the overlay is part of the model state, built from recorded ops, and the mesher sees it as one `TopologyView`. Reasons:
- Replay, fork and git state come free (ARCHITECTURE §3): a composite is an op in the timeline, not a side effect of a session. `VirtualOp` is already replayable (`overlay.rs:115,339`).
- Ours records the host revision and `rebase`s (`overlay.rs:379-395`), and refuses with a located, typed error when it cannot.
- Meshers, sweeps, semantics and tracking never learn about it. Ids are per-rank `Partition*` kinds (`crates/sm-core/src/id.rs:50-57`), so `dim_of` needs no table.

**Critique delta:** the draft compared this with "Cubit's virtual topology must be rebuilt by hand after many geometry edits". That is from memory and unverified, so do not build the argument on it. What we can show is measured: replay, fork and undo give equal hashes (T1.3), and a host edit is either rebased or refused with a location (T1.4).

Shape of the change:
- **Where it lives.** One overlay in `ModelLayer` (ACR-V1).
  - **Critique delta:** store the **built** `Overlay` (it is `Clone` and owns its host snapshot, `overlay.rs:308-316`) behind an `Arc`, and derive `OverlayData` for digest and serde. Storing only `OverlayData` would replay every op on every read.
  - The overlay is shared with the decomposition. `split_realized`/`unsplit_realized` (`decomposition.rs:801,888`) take it from the layer, so the persisted `CutCells`/`face_cells` (finding 3) always resolve.
  - It contributes **nothing** to the digest when there is no overlay. **Critique delta:** "empty" means "never created" (`None`). After `remove`/`clear` the overlay keeps its id counters (`overlay.rs:820-838`). The counters must stay in the digest, because two states that will mint different ids next are different states. So remove restores the *view* exactly but not the state hash, and timeline undo restores the hash exactly.
  - Every pinned hash (C3, `tools/platform/expected.txt`, 62 data lines) is unchanged for existing scripts. A non-empty overlay digests `OverlayData` (host revision, ops in order, counters), quantized by the model tolerance like geometry.
- **Who sees it.**
  - **Critique delta:** this is not "one call site family". There are 45 `.topo()` reads in 18 files: sm-mesher (5 files), sm-boolean, sm-model, sm-fem (`lower.rs:166,227,354`), sm-smooth (`op.rs:178,253`), sm-poly (`ops.rs:89`), sm-tess (`query.rs:112`), sm-view (`queries.rs:186`), sm-rt (`overlaps_query.rs:111`) and sm-cadio. Every reader of a mesh classified on `Partition*` ids (smooth, fem sidesets, poly, view, tess) must see the same view, or it fails on unknown ids.
  - **Better design:** `ModelLayer::topo()` returns the overlay view when one exists (readers unchanged), and a new `ModelLayer::host()` returns the bare host for the few callers that edit or export it. Those are `sm-boolean` guards, `sm-model` transforms and copy, and `sm-cadio::write` (STEP export writes real geometry). That is about 5 edits instead of about 40, and it fails safe: a forgotten reader sees the virtual view, which is what the mesh was built on.
  - Anything that edits the host refuses on a cell the overlay hides, with a typed error like `DecompositionActive` (`crates/sm-model-api/src/error.rs:88-90`), or calls `rebase` and reports. Default: refuse, and offer `model.remove_virtual`.
- **Ops** (`sm-model`, new modules): `model.composite_faces`, `model.composite_edges`, `model.partition_edge`, `model.partition_face`, `model.collapse_edge`, `model.remove_virtual`, and the query `model.virtual_info`. `model.split` and `model.composite` of pieces (decomposition) get ops in the same crate through `Model::split_realized` (`decomposition.rs:801`).

### D2. Composite surfaces that mesh

Two stages, so the first is cheap and the second is general:

- **(a) Composite over one carrier.** If all members share one surface handle, the composite's parametrization *is* that surface. The composite exposes the member surface and per-coedge pcurves, and the existing schemes (map, pave, submap, tri) work unchanged. This needs a contract extension (ACR-V2).
  - **Critique delta:** "project edge samples" is exactly the path `loop_uvs` refuses on periodic surfaces (`scheme.rs:538-540`). On a periodic carrier, ACR-V2 must **synthesize** pcurves that stay on one consistent seam branch: unwrap `u` by continuity along each loop, as cadio already does at poles (`crates/sm-cadio/DESIGN.md:155-160`).
  - A composite that closes around the period (a full band) must **keep one interior generator edge as its seam**, used twice. Otherwise there is nothing to unroll.
  - The same pcurve synthesis, applied to host faces without pcurves, also fixes the W3c deferral for imported periodic faces (`w3c-report.md:131`). Do it once in `sm-scheme-api`/`sm-topo-api` rather than twice. Planar composites need none of this.
  - Value is narrower than the draft said: most "same handle, two faces" cases are better served by the exact host join (T0), finding 7.
- **(b) General composite (several surfaces).** Parametrize the union by a discrete harmonic map:
  - Triangulate the members with `sm-tess` at fine resolution and map the boundary to a convex curve.
  - Solve the Laplace system with Floater mean-value weights. The map is bijective for a convex boundary (Tutte 1963; Floater, "Mean value coordinates", CAGD 2003; Rado-Kneser-Choquet; verify). Gmsh's compound surfaces use the same idea (Marchandise, Remacle, Geuzaine et al., "High-quality surface remeshing using harmonic maps", IJNME 2011; verify).
  - The scheme meshes in the `(u,v)` disc, and every node is **projected back onto the exact member surface** by `ParametricSurface::project` of the member that contains it.

  Projecting onto the exact analytic members keeps analytic accuracy: a tracked cylinder stays a cylinder. That matters for the `.smrt` and for polyhedral cells whose faces must lie on the true surface.
  - **Critique delta:** drop the claim "Cubit's virtual surfaces are tied to facet approximations". From memory, CGM composite surfaces evaluate on their member surfaces, so the claim is unverified. Our measurable bar is acceptance T11.1 (every node on its exact member to 1e-12).
  - **Critique delta (priority):** stage (b) is where virtual topology pays for the user's goals. Slivers, chamfer strips and fillet strips force tiny triangles and tets, and tet-dual polyhedra inherit them. Stage (a) does not remove any of those. Keep T11 sized L, but make it the first candidate for the later W4 sub-wave if the T3 corpus shows slivers and blends dominate (Q6).

### D3. Collapse, partition, angle

- **Collapse edge (virtual).** `collapse_edge(edge, into_vertex)` records a virtual vertex merge in the overlay: the short edge disappears from the view and its two end vertices become one. It is reversible and moves no host entity. Collapse *angle* is a selection helper that emits collapse ops.
  - **Critique delta:** a virtual collapse leaves the adjacent edges' curves ending up to `|edge|` away from `into_vertex`. That is far above `linear` = 1e-6. Meshers sample curves by parameter, and `validate` would flag the vertex as off its curves. ACR-V3 must:
    - record the collapse gap per affected cell (as `record_merged` does on the host, `edit.rs:345`)
    - have the view report those cells' tolerance as `max(own, gap)`, capped by `linear_max`; a collapse above `linear_max` is refused
    - have edge samplers blend the end offset linearly over the edge.

    This is the "tolerant topology" model of ARCHITECTURE §9 Q2 applied to the overlay. Without it the collapse produces invalid views. A collapsed edge whose adjacent face becomes zero-width (a sliver strip) must also composite or remove that face, so `collapse_small` emits composites as well as collapses.
- **Partition.** Use the existing `partition_edge`, `partition_face` and `partition_volume_by_plane`. `partition_face` refuses faces with inner loops (`overlay.rs:1051-1053`). Pcurve support for chords on periodic faces comes with the ACR-V2 pcurve synthesis (risk R3).
- **Why virtual and not destructive:** a destructive collapse changes geometry and breaks lineage. A virtual one is an op in the log that a fork can drop. The user's neutronics geometry must stay exact where tracked; only the *mesh view* is simplified.

### D4. Healing as detect, propose, apply (each a typed op)

Ours follows the `merge_check` / `merge` model (`merge.rs:505`, `ops.rs:322,369`):
- `model.heal_check` (query): located, classified findings. Each has a `code`, the entities, a measured size or gap, and the op that would fix it. Codes:
  - `small_edge`
  - `small_face` (area and width)
  - `sliver_face` (aspect ratio)
  - `short_loop`
  - `gap_near` (reuse merge status near)
  - `open_shell`
  - `inconsistent_orientation`
  - `small_hole` (an inner loop below a size threshold)
  - `blend_candidate` (a cylinder, torus or cone strip of near-constant width between two faces, rolling-ball radius below a threshold)
- `model.heal` (op): applies the chosen fixes in a fixed order, **tolerance first**. It raises per-entity tolerance (`raise_tolerance`, capped by `linear_max`) and welds or merges within it. Only on request (`mode: "geometric"`) does it move geometry.
  - **Critique delta:** "never moves geometry" overstates it. Merging two faces `g` apart keeps the lowest-id face's carrier, so the other volume's boundary moves by `g`, within the raised tolerance. Every such move is recorded as `record_merged` with its gap. T5's acceptance states the volume change exactly.
- **Defeaturing ops** (destructive, each a typed op, same detect/apply split):
  - `model.remove_hole`: delete the hole's faces and `kill_inner_loop` on the faces it pierces. Exact for holes in planar faces; no SSI needed.
  - `model.remove_small_face`: absorb into a neighbour by extending surfaces and re-intersecting. Needs SSI; planar neighbours first.
  - `model.remove_blend`: remove the rolling-ball strip, then re-intersect the two support surfaces. Large.
  - Literature (verify): Sheffer, "Model simplification for meshing using face clustering" (CAD 2001); Thakur, Banerjee, Gupta, "A survey of CAD model simplification techniques for physics-based analysis applications" (CAD 2009); Butlin and Stops, "CAD data repair" (IMR 1996); Barequet and Sharir, "Filling gaps in the boundary of a polyhedron" (CAGD 1995).
- **Non-destructive first.** Most small-feature problems for meshing are solved by D3 and D2(b) without touching the exact B-rep. Defeaturing ops are for features that must really go, e.g. bolts in a transport model.
- **Where.** `crates/sm-boolean/` owns `merge`, `regularize`, `same_surface` and the fragment kernel that uses SSI. Heal ops are new modules there.
  - **Critique delta:** this was the draft's Q3. It is an ownership call for the orchestrator, not a user decision, so it is stated here. The same goes for "tolerance first" (draft Q4) and "virtual collapse" (draft Q5). All three follow ARCHITECTURE §9 Q2 and the exact-geometry needs of the tracker.
- **Critique delta (import):** healing cannot only be a separate op, because the reader refuses open shells before anything reaches the model (finding 9). Add `import_step { repair: "tolerant" }` (T5b, sm-cadio). It runs the same tolerant fixes inside the import and reports each one as a finding. The import stays detect-and-refuse by default, so pinned import hashes do not move.

### D5. Facets: from file to tracker first, to B-rep second

The ray-tracing goal needs the imports DAGMC takes: STL and `.h5m`.
- **STL reader** (binary and ASCII, hardened like the Part 21 parser: size caps, no panics on hostile bytes) in `sm-cadio`. It is content-addressed like STEP and welds by the sorted grid in `weld.rs`. Multi-`solid` ASCII gives several solids. **Critique delta:** binary STL stores `f32` coordinates. Exact checks compare against the polyhedron of those `f32` values read as `f64`, never against a double-precision analytic polyhedron.
- **STL to tracker.**
  - **Critique delta:** `FacetInput` needs `EntityId`s for faces and volumes (finding 16). So the path is STL → faceted B-rep in the model (minimal: coplanar triangles merged into planar polygon faces, exact planes) → the existing B-rep/facet tracker build.
  - No ACR-F1 is needed: `front`/`back` already carry `Region::Volume(id)` (`input.rs:291-294`).
  - Very large STLs (10^6 triangles become up to 10^6 faces) are the cost of this path. A faceted `SurfaceDef`, one face per feature patch, is the scalable answer, and that is Q1.
  - **Critique delta:** soften "DAGMC reports leaks only at track time". DAGMC ships `check_watertight` and `make_watertight` as separate tools. Our advantage is narrower and real: located, grouped findings (`facets.rs:84-91`) in the same import op, and a recorded, replayable repair (`heal`, `close_gaps`: stitch within tolerance, fill small holes by minimum-area triangulation; Barequet-Sharir, verify).
- **`.h5m` DAGMC reader** into the tracker. **Critique delta:** the "apples-to-apples benchmark" argument does not hold. The W3c benchmark already runs DAGMC on the `.h5m` *we write* from the same triangles (`w3c-report.md:120-126`). A reader adds one thing: running *existing* DAGMC models through our tracker. It also needs a real HDF5 reader (finding 15), so it is split into T8a and T8b and moved out of the first sub-wave.
- **STL to analytic B-rep** (for meshing, `heal`, booleans): segment the triangle mesh into patches by feature angle; fit planes, cylinders, spheres and cones by least squares or RANSAC; fit the rest as NURBS (`fit.rs`). References (verify): Schnabel, Wahl, Klein 2007; Benko and Varady; Bechet et al., CAD 2002. This is the step toward "mesh-based geometry", and it is a user decision (Q1).
- **IGES**: parse the 80-column fixed format (types 100, 102, 110, 120, 122, 124, 126, 128, 130, 142, 144, 314, 406, 502, 504, 508, 510, 514). Surfaces 128 and trimmed 144 map to NURBS. The hard part is sewing: IGES files usually have no topology, so the shell is rebuilt within tolerance by the weld and `merge` machinery. Large, and low value for the user's STEP-driven workflow.

### D6. Priority logic, one sentence each
- **Critique delta:** the cheapest real win for imported geometry is the exact host join (T0). Then the pcurve synthesis shared by imported periodic faces and composites (ACR-V2), then composites. Stage (a) composites alone make few new things meshable.
- The tracker's biggest practical lever is "does dirty geometry get in": healing at STEP import (T5b) for the user's STEP models, STL import (T7), and a `heal_check` that locates leaks.
- Destructive defeaturing is a long tail: do it after the detector shows what the user's real models need.

## Tasks

Notation: Owner = crates the worker edits. W4-0 items are orchestrator edits to frozen crates. Gates for every task:
- `cargo build`, `cargo test -p <crate>`, `cargo clippy -p <crate> -- -D warnings`
- a determinism test: two runs, `RAYON_NUM_THREADS=1` vs default, hashes equal
- rustdoc on every public item
- `sm_testkit::strategies::config(n)` for property tests
- no `HashMap` order, no `unwrap` on input paths

The W3c hash rule still holds (C3): a moved existing milestone hash stops the task.

### W4-0 (orchestrator ACRs, on main before workers branch)
- **ACR-V1 `ModelLayer` overlay.**
  - `Option<Arc<Overlay>>` in `ModelLayer` (`sm-layer-api/src/model.rs:44`), shared with the decomposition (finding 3).
  - `topo()` returns the overlay view when present, and new `host()` returns the bare host (D1).
  - `set_overlay`; digest contribution only when `Some`, including the counters.
  - Hash impact: **none for existing scripts.** Test: every pinned hash in `tools/platform/expected.txt` is equal, and the `sm-boolean`/`sm-model`/`sm-cadio` call sites switch to `host()` in the same ACR.
- **ACR-V2 composite surface and periodic pcurves.**
  - `TopologyView::composite_surface(face)`: the shared surface when all members share one handle.
  - Pcurve synthesis by continuity unwrap on periodic carriers, for composite coedges and for host coedges without pcurves.
  - A seam kept on composites that close around the period (an `Overlay` change).
  - `GenCtx::loop_uvs`/`face_normal` use all of the above (`scheme.rs:494`, `geom.rs:152`).
  - The planner maps `PartitionFace` to `Target::Surface` (`planner.rs:291`).
  - **Critique delta:** this ACR is now medium, not small. Split it: V2a (planar and non-periodic composites plus the planner fix) is small and goes first; V2b (periodic pcurve synthesis plus seam) can land with T2b.
- **ACR-V3 collapse.** `VirtualOp::CompositeEdges` and `VirtualOp::CollapseEdge` in the overlay, with a recorded gap and per-cell view tolerance (D3). `TopologyEdit::collapse_edge` (destructive primitive, default `Unsupported`) only if T6 needs it; otherwise drop it.
- ~~**ACR-F1 `FacetInput` provenance**~~ **Critique delta:** dropped (D5: regions are already per triangle; ids come from the faceted B-rep).

### T0 REGULARIZE-OP (S) - exact join of co-surface faces and edges *(new; Critique delta)*
- **Owner:** `crates/sm-boolean/` (an op over the existing `regularize`, `regularize.rs:231`).
- **Scope:** `model.regularize {volumes}` joins co-surface faces and degree-2 edges of one volume within the model tolerance, moving nothing beyond it (`regularize.rs:1-8`). It reports the joined pairs and the refused ones.
- **Acceptance:**
  1. `webcut_plane` of a cylinder through its axis, then `unite {regularize:false}`, then `model.regularize` gives the face/edge/vertex counts of `create_cylinder` (3 faces), and `volume_moments` equals `pi r^2 h` to 1e-12.
  2. Applied to the M5 lattice STEP import it changes nothing (no co-surface pairs), and the hash is equal.
  3. Determinism gate.
- **Hash impact:** none on existing.
- **Dependencies:** none. Size S.
- **Risk:** joining the two lateral halves yields a band with no seam. If `join_faces` produces it, T0 must keep or re-insert a seam edge, or `loop_uvs` refuses the face (finding 5). Acceptance 1 includes meshing the result with `mesh.auto` to catch this.

### T1 VTOPO-OPS (M) - virtual-topology ops and layer plumbing
- **Owner:** `crates/sm-model/` (new modules `vtopo.rs`, `vtopo_query.rs`).
- **Scope:**
  - `model.composite_faces`, `model.composite_edges`, `model.partition_edge`, `model.partition_face`, `model.remove_virtual` and `model.virtual_info` over the layer's overlay.
  - Ops for `split_realized`/`unsplit_realized`.
  - Host-edit refusal error `VirtualActive(id)`; `rebase` on host moves where legal.
- **Acceptance (analytic):**
  1. **Critique delta (fixture replaced):** `webcut_plane` of `create_cylinder` through the axis, then `unite {regularize:false}`, gives one volume whose lateral face is in two halves. Composite those two halves. `face_moments.area = 2 pi r h` to 1e-12, and `model.list` shows one `PartitionFace` in place of the two. If the webcut halves do not keep one surface handle, the worker records that, and stage (a)'s test uses `regularize`'s same-surface re-pointing first.
  2. `partition_edge` at parameter `t` of a circle edge gives two edges whose lengths sum to `2 pi r` to 1e-12, and a new vertex at `C(t)` to 1e-12.
  3. Record, replay and fork (W2 scripts style) give equal state hashes, and undo/redo restore the overlay exactly. **Critique delta:** `remove_virtual` restores the *view* exactly (the canonical topology visit of the view equals that of the host, and `model.list` is equal). The state hash differs from the pre-composite hash only by the id counters, and the next minted id is fresh.
  4. A boolean on a body with an active composite returns `VirtualActive`, never a corrupted state.
  5. Property test: random composite/partition/remove sequences keep `validate` clean and never reuse ids.
  6. **Critique delta (new):** `split_realized` through the op on the layer's overlay, then save, load and replay: every `CutCells` id resolves in the replayed view.
- **Hash impact:** none on existing; new virtual-op scripts get new pins.
- **Dependencies:** ACR-V1, ACR-V3 (composite edges). Size M.

### T2 VTOPO-MESH (M) - meshing on the overlay view, composite (a), planar
- **Owner:** `crates/sm-mesher/`, `crates/sm-scheme-map/`, `crates/sm-scheme-pave/` (only if needed for composite loops). **Critique delta:** with ACR-V1's `topo()` returning the view, sm-smooth, sm-fem, sm-poly, sm-tess and sm-view need no edits. T2 adds one smoke test per such crate in `crates/sm-mesher/tests` (smooth and export a mesh on a composite) rather than editing them.
- **Scope:** `mesh.*` on the view; classification on `Partition*` (`sm-mesh-api/src/classify.rs`); composite surfaces of stage (a) on **non-periodic** carriers; interval matching over virtual edges.
- **Acceptance:**
  1. The composite of the two halves of a brick face split by `webcut_plane` then `unite {regularize:false}` (one plane handle) meshes with the node count of one mapped face with the same boundary intervals.
  2. *(moved to T2b)*
  3. `mesh.run` on a model with an overlay is bit-identical at 1 and N threads.
  4. The mesh of an overlay-free model has the same hash as before (C3 census).
  5. **Critique delta (new):** a volume selected together with one of its composite faces plans the face as a `Target::Surface` (planner fix of ACR-V2a).
- **Hash impact:** none on existing; new rows pinned.
- **Dependencies:** ACR-V1, ACR-V2a, T1. Size M.

### T2b VTOPO-MESH-PERIODIC (M) - composites on periodic carriers *(split from T2; Critique delta)*
- **Owner:** `crates/sm-mesher/`, `crates/sm-scheme-pave/`.
- **Acceptance:**
  1. The T1.1 cylinder composite meshes with pave, seam kept: area = `2 pi r h` within the mesh's chord error, and every node within 1e-12 of the cylinder.
  2. An imported STEP cylinder (no pcurves) meshes through the same synthesis (closes the W3c `--model` periodic-face deferral for the meshing side).
- **Dependencies:** ACR-V2b, T2.

### T3 VTOPO-DETECT (M) - `model.heal_check`
- **Owner:** `crates/sm-boolean/` (new module `heal_check.rs`). ~~Optional T3b in `sm-sizing`~~ **Critique delta:** dropped. `l_min` is a lower bound of the clamp, so a short edge never drags `auto_size` down (finding 11).
- **Scope:** the D4 query with codes `small_edge`, `small_face`, `sliver_face`, `small_hole`, `gap_near`, `open_shell` and `blend_candidate`. Each finding is located, with thresholds relative to `ModelTolerance` and the bounding-box diagonal.
- **Acceptance (Critique delta: fixtures named, since no chamfer or fillet op exists; expected sets exact):**
  1. **Notch.** `create_brick` (unit) minus a cube of side `s = 1e-4` at one corner gives exactly 9 `small_edge` (the notch edges, length `s` to 1e-12) and 3 `small_face` (area `s^2` to 1e-12), and nothing else.
  2. **Step.** Unit brick A united with brick B = `[s, 1+s] x [0,1] x [1,2]` (`s = 1e-4`) gives exactly 4 `small_edge` and 2 `sliver_face`, each sliver with aspect ratio `1/s` to 1e-9 relative.
  3. A plate with a through hole of radius `r_h` yields one `small_hole` at threshold `> 2 r_h` and none below.
  4. Two bricks separated by gap `g` give `gap_near` iff `tol_linear < g <= linear_max`, matching `merge_check` status near.
  5. Findings are sorted by (code, entity id) and identical for 1 and N threads.
  6. **Fillet.** A brick minus the corner block `[0,r]^2 x [0,L]` plus a quarter cylinder of radius `r` (built by `subtract` and `unite`, regularized) gives one `blend_candidate` with radius `r` to 1e-9.
- **Hash impact:** none (read-only query).
- **Dependencies:** none (first sub-wave). Size M.

### T4 VTOPO-COLLAPSE (M) - virtual collapse
- **Owner:** `crates/sm-model/`, using the overlay collapse from ACR-V3.
- **Scope:** `model.collapse_edge`, `model.collapse_angle` (select and emit), and `model.collapse_small` (from `heal_check` findings; emits collapses and composites, D3).
- **Acceptance (Critique delta: rewritten; the old `auto_size` check passed with or without a collapse):**
  1. On the T3 step fixture, `mesh.auto` without cleanup has minimum element edge length ≤ `s`. After `collapse_small`, the minimum element edge length is ≥ `auto_size / 4` and the worst scaled Jacobian improves. Report both numbers.
  2. `remove_virtual` restores the view exactly (as T1.3).
  3. The host complex hash is identical before and after.
  4. `validate` of the view is clean, with the collapse gap recorded on the affected cells.
- **Hash impact:** none on existing.
- **Dependencies:** ACR-V3, T1, T3. **Critique delta:** size S/M → M (gap bookkeeping, composites for degenerate strips).

### T5 HEAL-TOL (M) - tolerant heal and gap closing
- **Owner:** `crates/sm-boolean/` (new `heal.rs`).
- **Scope:** `model.heal {tol, fixes}`:
  - weld vertices and merge edges/faces at tolerance (reuse `merge`)
  - raise per-entity tolerances capped by `linear_max`
  - fix inconsistent orientation per shell (`reverse_face`, `edit.rs:294`)
  - close an open shell whose open loop is a closed planar polygon within tol (`make_face`, `edit.rs:65`)
- **Acceptance:**
  1. **Critique delta:** two bricks with gap `g = 5e-5` (`< linear_max`) heal to one shared face. The kept face is the lowest id (`merge.rs:3-4`), so the total volume equals `V1 + V2 ± A g` with the sign fixed by which brick keeps its face, to 1e-12. The moved brick's tolerance is raised to ≥ `g`.
  2. After `heal`, `model.merge_check` reports no near pairs.
  3. An orientation-flipped shell regains positive `volume_moments`.
  4. A planar hole closed by `make_face` gives a watertight shell (`validate` clean, Euler characteristic 2).
  5. `heal` twice equals `heal` once (idempotent hash).
  6. Every change appears as lineage (`Merged{gap}`).
- **Hash impact:** none on existing.
- **Dependencies:** T3.

### T5b STEP-REPAIR (S/M) - healing inside the STEP import *(new; Critique delta)*
- **Owner:** `crates/sm-cadio/`.
- **Scope:** `import_step { repair: "tolerant" }` runs T5's fixes on a solid the reader would refuse with `OpenShell` (`DESIGN.md:96,257`) and reports each fix as a finding. The default stays detect-and-refuse.
- **Acceptance:**
  1. A STEP cube with one face removed (fixture) imports with `repair` and has volume 1 to 1e-12.
  2. Without `repair`, the same file is refused as today.
  3. Existing import hashes are unchanged.
- **Dependencies:** T5 (needs its fixes as a library function `sm-cadio` can call; if that crosses the crate boundary, an ACR names the shared function).

### T6 DEFEAT-HOLE-SLIVER (M) - hole and sliver removal
- **Owner:** `crates/sm-boolean/`.
- **Scope:**
  - `model.remove_hole`: a hole in a planar face; delete the hole faces, then `kill_inner_loop`.
  - `model.remove_sliver`: absorb a sliver into the neighbour that shares the longest edge, by `join_faces` when co-surface, else by SSI re-intersection of the neighbours.
- **Acceptance:**
  1. Removing a through hole of radius `r` in a plate of thickness `h` raises `volume_moments` by `pi r^2 h` to 1e-12.
  2. Removing a blind hole of depth `d` raises it by `pi r^2 d` to 1e-12, and the floor face disappears.
  3. Removing a sliver between two coplanar faces gives one face (`join_faces`) whose area equals the sum.
  4. A hole that is not removable (shared with a second volume) is refused with a located reason.
  5. Lineage records `Deleted` on every removed cell.
- **Hash impact:** none on existing.
- **Dependencies:** T3.

### T7 STL-IN (M) - STL reader, writer and faceted body
- **Owner:** `crates/sm-cadio/` (new `stl.rs`, op `model.import_stl`, query `cad.stl_info`).
- **Scope:**
  - Binary and ASCII STL: detect by size (`84 + 50 n == len`), not by the `solid` header. Multi-solid, size caps, welding at `weld_tol`, units.
  - A faceted body in the B-rep: coplanar triangles merged into planar polygon faces (exact planes), reusing the `FACETED_BREP` path (`decode.rs:696`). This gives the face and volume ids the tracker needs (D5).
  - A `check_facets`-style report.
  - An STL writer: binary and ASCII, fixed order, one `solid` per volume.
  - Content-addressed `{sha256}`/`{inline}` source as in `model.import_step`.
  - **Critique delta:** wiring `session.export format: "stl"` belongs to the `sm-server`/`sm-cli` owners. It is a separate S task (T7b) in the same sub-wave, not "a one-line follow-up" done by this worker.
- **Acceptance:**
  1. An icosphere STL (80 triangles) imports with `V - E + F = 2` and zero `facet.open_edge`. **Critique delta:** its volume equals the polyhedron volume computed from the file's `f32` vertices (in `f64`) to 1e-12 relative.
  2. An STL cube (12 triangles) imports as **6 planar faces**, 12 edges and 8 vertices.
  3. An STL with one missing triangle gives exactly one `facet.open_edge` at the centroid of the missing triangle, with gap equal to its longest edge (`facets.rs:84-87`).
  4. Hostile files (truncated header, NaN, count larger than bytes, 2^32 triangles) return a typed error with no panic, within the memory budget.
  5. **Critique delta:** ASCII and binary STL of the same shape give equal state hashes, where the ASCII prints each `f32` in shortest round-trip form.
  6. Export, import, export gives equal bytes.
- **Hash impact:** none on existing; new scripts get new pins.
- **Dependencies:** none. Size M.

### T8a H5M-READ-OWN (M) and T8b H5M-READ-MOAB (L) *(split; Critique delta)*
- **Owner:** `crates/sm-meshio/` (`h5/read.rs` promoted out of the `testing` gate and hardened; new `dagmc/read.rs`).
- **T8a scope:** read our own `.h5m` subset into a faceted model or `FacetInput`, with `GEOM_DIMENSION`/`CATEGORY` tags, surface sense tags and group `mat:` names. Refuse tet meshes and unknown tags with typed errors that name the tag.
- **T8a acceptance:**
  1. Read back our writer's output: triangle sets equal.
  2. The tracker on that read-back agrees with the in-process tracker on 10000 rays.
  3. Hostile files (truncated, cyclic groups) give a typed error.
- **T8b scope:** v0/v1 superblocks, v1 object headers, symbol-table groups, chunked and deflated datasets, as MOAB writes them (verify on a real file), or a `libhdf5` dependency (Q7).
- **T8b acceptance:** the DAGMC reference `.h5m` that the benchmark images use reads, and the tracker on it agrees with `dagmc-moab` on 10000 rays.
- **Hash impact:** none.
- **Dependencies:** none. **Priority:** later sub-wave, unless the user has DAGMC models to bring (Q8).

### T9 STL-BREP (L) - facets to analytic B-rep
- **Owner:** `crates/sm-cadio/` (new `facet_fit.rs`); `crates/sm-geom/` only for fitting helpers if missing.
- **Scope:** feature-angle segmentation; plane, cylinder, sphere and cone fits with a residual bound, else a NURBS fit; edges from patch boundaries; the fit residual reported per face. Gated by Q1.
- **Acceptance:**
  1. An STL cylinder (chord error `e`) imports as 3 faces (2 planes, 1 cylinder), radius within `e`, axis within 1e-6 rad.
  2. A tessellated sphere gives one sphere face with radius within `e`.
  3. Residuals are reported and bounded.
  4. The result passes `validate` and meshes with `mesh.auto` (needs T2b's pcurve synthesis for the periodic faces).
- **Dependencies:** T7, ACR-V2b.

### T10 IGES-IN (L) - IGES reader with sewing
- **Owner:** `crates/sm-cadio/` (`iges/`). **Scope:** as in D5. W5+ unless the user brings IGES files.
- **Acceptance:**
  1. A unit cube in IGES (six trimmed planar 144/128) sews to one closed shell with `V - E + F = 2`.
  2. A cylinder sews with its seam edge used twice.
  3. Units and transforms (124) are applied.
  4. Hostile files give typed errors.
- **Dependencies:** T5.

### T11 COMPOSITE-GENERAL (L) - composite surfaces over several surfaces (stage b)
- **Owner:** `crates/sm-mesher/` (new `composite_param.rs`), exposed to schemes through ACR-V4 (a `ParametricSurface` for a composite face, handed in `SurfaceMeshInput`).
- **Scope:** D2(b).
- **Acceptance:**
  1. The composite of a half-cylinder and a half-sphere cap meshes with pave, every node on its exact member surface to 1e-12, total area within the chord error.
  2. The parametrization is a bijection: all disc triangles have positive area and Euler characteristic 1.
  3. Bit-identical at 1 and N threads.
- **Dependencies:** T2, ACR-V4. **Critique delta:** later W4 sub-wave if T3 on the user's corpus shows slivers and blends dominate (D2(b)); otherwise W5.

### T12 DEFEAT-BLEND (L) - blend and small-face removal by re-intersection
- **Owner:** `crates/sm-boolean/`. Uses SSI.
- **Acceptance:**
  1. On the T3.6 fillet fixture, `remove_blend` restores the sharp brick: volume back to `L·W·H` (it rises by `r^2 (1 - pi/4) L`) to 1e-10.
  2. A fillet that ends at a vertex blend is refused with a located reason.
  3. Lineage is complete.
- **Dependencies:** T3, T6. W5+.

## Priority (for the user's goals)

**Now (first W4 sub-wave)**
- **W4-0:** ACR-V1, ACR-V2a, ACR-V3 (no hash impact). ACR-V2b as soon as one worker can validate it (T2b).
- **T0 REGULARIZE-OP** *(Critique delta: new, S)*: exact, cheap, and fixes split faces in imported and unregularized bodies without virtual topology.
- **T3 VTOPO-DETECT:** read-only, no contract change, and it tells the user what their real models need, so T4/T6/T11/T12 are built for observed features.
- **T7 STL-IN** (+T7b wiring): the first non-STEP front door to the tracker.
- **T5 + T5b** *(Critique delta: moved up)*: today a STEP solid with one open edge is refused outright (finding 9). For the user's STEP-driven work this blocks the front door more than STL does.
- **T1 + T2 (planar composites):** the plumbing every later virtual op needs.

**Later W4 sub-wave:** T2b (periodic composites and imported periodic faces), T4 (virtual collapse), T6 (holes, slivers), T8a, T11 if the corpus asks for it, T9 only if Q1 is yes.

**W5+ (or on trigger):** T8b (Q7/Q8), T12 (blends), T10 (IGES).

## Risks

- **R1 Overlay staleness.** Any host edit can invalidate an overlay. `DecompositionActive` covers volumes only (`error.rs:88-90`). Mitigation: refuse host edits on hidden cells (T1.4), and `rebase` (`overlay.rs:385-395`, atomic) only where the touched cells are unaffected. The default is a typed refusal.
- **R2 Hash pins.** The digest must not change when there is no overlay. `tools/platform/expected.txt` (62 data lines) is the test, and a change stops the task (C3). **Critique delta:** ACR-V1 also switches about 5 call sites from `topo()` to `host()`. A wrong switch changes behaviour, not hashes, so the ACR runs the full workspace tests, not only the pins.
- **R3 Pcurves.** Imported faces carry none except at poles, and periodic composites and chords need them. Mitigation: synthesize them once (ACR-V2b). Until then, T1 refuses `partition_face` on a periodic face without pcurves, with a located error.
- **R4 Composites break scheme applicability.** Sweep and submap rely on loop structure and vertex types. A composite changes the vertex type count. Mitigation: T2 runs the existing applicability tests on composites.
- **R5 Healing changes geometry.** Mitigation: tolerance first, every change in lineage with its gap, `mode: "geometric"` as an opt-in.
- **R6 Hostile files.** The STL and `.h5m` readers are new parsers. CLAUDE.md's no-`unwrap` rule and the Part 21 parser's budgets apply (T7.4, T8a.3).
- **R7 Size.** T11, T12, T10 and T8b are each L. T11/T12 depend on SSI maturity.
- **R8 Literature unchecked.** References are from memory (marked verify).
- **R9 Collapse validity** *(Critique delta: new)*. Virtual collapse leaves vertices off their curves by up to the edge length. Without ACR-V3's gap bookkeeping the view is invalid (D3).
- **R10 Seamless bands** *(Critique delta: new)*. Joining (T0) or compositing the halves of a periodic face can remove the only seam. Both must keep one (T0 risk, ACR-V2b).

## Questions for the user

**Critique delta:** draft Q3 (healing home), Q4 (healing default) and Q5 (collapse virtual or destructive) are orchestrator design calls with one-sided answers. They are now stated in D3/D4. The remaining questions are real decisions:

1. **Mesh import and mesh-based geometry.** PARITY.md:91 defers mesh import by decision. Should STL and `.h5m` import (T7, T8) be allowed under that decision, while mesh-as-geometry for meshing (T9, a faceted `SurfaceDef`) stays deferred? **Recommendation: yes for T7/T8.** A triangle surface is geometry input here, not a mesh to edit, and T7's faceted B-rep keeps it inside the existing B-rep. Keep mesh-based geometry deferred until the first STL too large for one face per planar patch (10^5+ triangles).
2. **IGES.** Drop, defer to W5+, or do now? **Recommendation: W5+.** Your models are STEP; IGES files usually have no topology and need sewing.
3. *(was Q6)* **Real models.** Do you have dirty STEP or STL models to test on (the local STEP files in `test-geometry-sources`)? Two or three of them decide the T3 thresholds, whether T11 (composites over different surfaces) moves into W4, and the order of T5/T6. **Recommendation: pick them now and commit their sha256 as the T3 corpus next to the analytic cases.**
4. *(new, Q7)* **HDF5 for foreign `.h5m`.** Should we extend the pure-Rust HDF5 reader to what MOAB writes (L, no C dependency) or depend on `libhdf5` (smaller, but adds a C library to the build and CI)? **Recommendation: pure Rust**, consistent with `h5/mod.rs:3` ("can be lifted into its own crate"), and only once Q8 says it is needed.
5. *(new, Q8)* **DAGMC models.** Do you have existing DAGMC `.h5m` models you want to run through `sm-rt`? If not, T8b waits (the benchmark already compares on the same triangles).
6. *(new)* **Periodic composites now or later.** Is meshing a cylinder or sphere band as one composite (T2b) needed in W4? T0 already joins split periodic faces exactly. **Recommendation: later W4 sub-wave, together with the imported periodic-face fix it shares.**

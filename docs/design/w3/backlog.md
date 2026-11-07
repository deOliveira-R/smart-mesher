# W3 backlog (collected at the end of W2)

Sources: W2 integration (decision 10), the post-integration contract batch, and the W2 polish wave (decision 11). Each item names its owner. "Contract" means a frozen crate, so the orchestrator schedules it.

## Contract changes
- **CR-2 completion** (`sm-model-api`): `validate` still uses `FaceMoments::plane`. Switch it to `TopologyView::face_plane`, then drop sm-model's `Tolerance::with_linear(1e-6)` mirror-deck workaround.
- **Plan once** (`sm-scheme-api`): `Driver::mesh_from_plan`, plus `DriverOptions::max_cells` or a cell estimate on `Plan`. Today `mesh.run` plans twice and keeps its own budget.
- **Planner firmness cascade** (`sm-scheme-api`): today an edge request always beats a face or volume request. Cubit's rule is that firmness decides first, then the latest command wins (decision 9).
- **Scheme-derived counts** (`sm-scheme-api`): `IntervalSink::goal` is averaged with the edge's sizing goal and the Default counts. So bias/stretch schemes that derive n from sizes cannot use it: n comes out as 2 instead of 6. They need a preference that replaces the sizing goal without becoming a soft count.
- **Bulk mesh commit** (`sm-layer-api`, `sm-mesh-api`): bulk adopt/retire and `MeshBuild::add_template_cells`. A 10⁶-cell `mesh.run`+`mesh.delete` takes 10.6 s, against 4.7 s for the driver alone.
- **`ModelLayer` from `(TopoHost, Model)`** (`sm-layer-api`): lets sm-fem test against `sm_testkit::topo::mirror_chamber()` instead of an in-test op.
- **Overlay `MeshRead` for export** (`sm-mesh-api`): sm-fem's `clone_host` is O(mesh) per export and has not been measured at 10⁶ cells.
- **Element type in `.smsh`** (`sm-meshio` format): it is checked at export but not written. Fold this into the polyhedral-format work.

## Features
- **`mesh.plan` / explain query:** report which scheme ran or will run on each entity (`mesh.info` has counts only).
- **`mesh.run` on faces alone** (Cubit `mesh surface`). Needed for QUAD4 and the P0 surface-only map.
- **Tri/tet meshing,** for the P0 TET4 and QUAD4 element types.
- **Script record:** write a session's op history out as a `.smsh` script (PARITY A, P1).
- **`smesh serve --ws`:** forward to sm-server's `ws` feature.
- **Sequential meshing against meshed neighbours:** replaces the `mesh.run` neighbour guard (decision 9).

## Small owner items
- sm-server `write_atomic` uses a fixed `.part` temp name. Use a unique one (pid + counter) so concurrent exports are safe.
- sm-mcp: an engine-backed variant of `respelled_ids_feed_back_as_arguments` (needs a real-kernel dev-dependency).
- sm-mesh `contract_alloc`: the warm-up before the measured pass weakens the zero-allocation contract. Drop it (the per-thread counter alone fixed the flake), or assert that the cold pass allocates nothing too.
- A `cargo fmt` pass over sm-mcp and sm-scheme-api (pre-existing drift; CI does not check fmt).

## Validation, quality diagnostics, exporter conformance (user direction, 2026-10-25)
Principle: our own checks decide whether a mesh is valid and how good it is. External tools (OpenFOAM `checkMesh`, Fluent readers, ParaView/meshio) only judge an exporter. If our validator passes and `checkMesh` fails, the bug is in the OpenFOAM exporter. Exporters own each target's quirks (e.g. OpenFOAM's 7-point `wedge` vs 6-point `prism`, Fluent c0/c1 orientation). Internal cells stay exact. Exports are never tuned to one tool.

- **Validity exists; expose it.** `sm_mesh_api::validate` + `validate_geometry` already cover checkMesh's topology part and more (about 35 stable codes: closedness, incidence, orientation, positive volume, normal direction, planarity, classification). Add a `mesh.validate` op (CLI + MCP) that returns the violations with code, entity id, classification (which surface/volume), and location. Every mesh-producing op must pass it in tests, so "our mesh is invalid" is always our bug.
- **Finite-volume face metrics** (`sm-quality`): non-orthogonality, FV skewness, face warp/flatness, face concavity, neighbour volume ratio, cell determinant, interpolation weight. Write each formula in `DESIGN.md` next to the checkMesh / Fluent definition, so our numbers can be cross-checked against `checkMesh` output, not just its pass/fail.
- **Quality diagnostics op** (`mesh.quality`, CLI + MCP): summary, histogram, worst-N cells with location, aggregation per geometry entity (through classification: "bad cells cluster on surface 12 near curve 7"), threshold profiles (generic, openfoam, fluent, transport), and suggested actions (intervals on curve X, scheme change, smoothing). It can store the offending cells as a group, so follow-up ops (smooth, refine) can target them.
- **Exporter conformance harness** (CI, Docker OpenFOAM v2506): for each fixture, our validator passes → export → `checkMesh -allGeometry -allTopology` (polyMesh) / `fluent3DMeshToFoam` + `checkMesh` (Fluent) / meshio round-trip (VTU, CGNS). Any external failure is an exporter bug. Also compare max non-orthogonality and skewness against our metrics within tolerance. References: `polyDualMesh` (tet → poly), Gmsh (tet quality).

## Exporter format rulings (user, 2026-10-25)
- **OpenFOAM polyMesh export: ASCII only.** No OpenFOAM binary writer; users who want binary run `foamFormatConvert` (OpenFOAM's own tool).
- **Fluent `.msh` export: ASCII first** (spec: `docs/research/fluent-msh.md`). Binary waits until the open binary questions (§14 items 5–8) are settled against Fluent itself.
- **Fluent gap:** the user will try to get Fluent (trial or student licence). Checklist once available: (1) load `docs/research/fluent-msh/*.msh` and the exporter's fixtures, confirm no orientation or volume complaints; (2) save a Fluent-written ASCII and binary `.msh` of a small mixed mesh as golden files (settles binary newline, int width, 39 domain-id, solid zone type); (3) check that a polyhedral (type 7) mesh and a non-conformal interface load.

## Ray tracing and delta tracking (W3b, from docs/research/dagmc-internals.md and delta-tracking.md)

- **One segment iterator, two modes.** Tracking yields segments `{t0, t1, cell_or_node, Σ_M}`. Surface tracking is the special case where a segment is one cell and Σ_M = Σ_t, so nothing is rejected. Delta tracking uses a node's local majorant. Switching mode along a flight is unbiased.
- **Majorant hierarchy.** It must be a disjoint partition traversed by DDA: lattice/universe nodes, plus a kd-tree or octree over meshed regions. A BVH has overlapping boxes, so it serves point location but not majorant segments. Nodes store bounds and a deduplicated material-set id; majorant tables are keyed by (material set, energy band). Additive to `sm-rt-api` (new provided trait or new trait).
- **Hinted point location.** At a tentative collision, location starts from the node that owns the majorant: O(1) lattice index, a binary search on r² for pin rings, or a walk or small BVH inside a meshed leaf. Never from the root. OpenMC's delta-tracking PRs (#3971, #4044) restart at the root, and that is why their fusion case got slower.
- **Region jump.** In a large homogeneous region DAGMC makes one O(log T) ray fire while a mesh walk pays per cell. When the caller does not need per-cell segments (Monte Carlo without mesh tallies), add an exact per-region boundary structure for one-jump crossings. MOC/Sn and mesh tallies keep the cell walk.
- **Safety distance.** The distance from a point to a convex cell's planes is an exact lower bound. Skip the intersection test when the sampled flight is shorter. This is DAGMC's signed-distance-field prototype (Shriwise 2017), but exact and without a grid.
- **Hybrid policy.** Use Serpent's rejection test (`dt` 0.9) at run time, plus a per-node, per-band cost model decided at build time; both depend only on (node, material, energy). A majorant violation is an error, never clamped. Use track-length estimators on surface segments and collision estimators on delta segments; mixing both in one tally needs tests.
- **Benchmark honestly.** Compare segments per track × cost per segment against one ray fire per track. Compare against DAGMC `ray_fire` (MOAB and double-down/Embree) and XDG `walk_elements`; XDG already walks tet/hex meshes, so cell walking alone is not novel. Our claims rest on exact analytic faces on polyhedral cells, no restart from the tree root, native instancing, hinted location, determinism and flat memory.
- **Serpent gaps.** The research could not reach serpent.vtt.fi (HTTP 403). STL point-in-solid, the adaptive search mesh and polyhedral-cell location are unverified. The user can paste the wiki pages "Delta- and surface-tracking", `polymesh.html` and `check_stl.html`.

## From W3a integration (2026-10-31; details in `w3a-report.md`)
- **OpenFOAM INT-8 on one-layer 2D cases** (sm-meshio, sm-fem): every cell of a one-layer hex or wedge mesh is split. Add an export option to disable the split and a 2D fixture.
- **Export notes in `smesh export --json`** (sm-cli/sm-server): then assert the INT-8 split count against `smeshCellMap` in the milestone.
- **Warped dual faces** (sm-meshio): split into planar pieces in the OpenFOAM exporter; measure INT-9 counts.
- **Transport diagnostic** (sm-quality/sm-diag): flag cells with two or more boundary faces.
- **Smooth suggestion** (sm-diag): `mesh.quality` should suggest `smooth` for spread scaled-Jacobian warnings on real meshes.
- **sm-scheme-tet:** `recover_facet` rounding (n=64 cliff); `RecoveryFailed.tri` placeholder; pass-level respace unit test; O(n·hints) restart check.
- **sm-server:** `Server::open` binds another builder's pending store.
- **Contracts:** `SchemeNote` / `Patch::notes` (sm-scheme-api); `MeshEntry.derived_from`.
- sm-cadio wall-clock tests → work counters; sm-poly typed error for unrepaired cells.

## From the post-W3a contract batch and W3a follow-ups (2026-11-04)
Merged: contract batch 55a20a9; follow-ups foam2d, tet, server, cadio, poly (a55b1ed..ee78766).

Rulings proposed for W3b-0:
- Firmness cascade (decision 9): resolve at command time in `mesh.size`/`mesh.intervals` so stored intent is order-free; flip `mesh.run` to Firmness only after a milestone A/B with unchanged hashes.
- Re-bless the `mesh.quality` output schema golden once to publish `scheme_notes` / `n_scheme_notes`.
- Move scheme notes from `Plan` to an unhashed `MeshEntry::notes`; until then clear `plan.notes` in `mesh.to_poly`.
- sm-op-api ACR: `ApplyCtx::retire_range` for bulk delete (10⁶ cells: 7-11 s today); test hash equality with per-id retire.
- sm-protocol ACR: `session.export` gets `split_small_determinant` (default true) and a way to name empty/wedge patches; sm-fem emits them. Add 2D conformance fixtures (incl. the unsplit tri-prism slab).

Cleanups:
- sm-server `host.rs`: move `notes_of` above `write_dir`'s doc comment.
- sm-scheme-api: `SchemeNote` `#[non_exhaustive]` + constructor; `MeshIntent::set_intervals` running counter (O(n²) today).
- sm-mesher DESIGN.md "Scheme notes" text is stale; sm-diag dev-deps comment.
- `ExportNote::parse` loses the code when the count is missing.
- sm-meshio: misleading comment in `tests/foam_2d.rs`; FOAM_FORMAT.md §8 "six above" needs the list; §11 open follow-ups.
- sm-scheme-tet: n=64 cliff test runs only in release (CI is debug); hint-index work-counter test; record path B smoothed tet SJ 0.1848.
- sm-cadio: refusal tests no longer bound parse work; per-run Newton cost not counted; tighten `global_projections <= 16` to MAX_GLOBAL; `sm_cadio::work` module/function name clash; global atomics on the public API.
- sm-poly: map `UnrepairedCell` to `OpError::Failed`, not `Invalid`; `tests/robust.rs` asserts nothing about `region`; agglomeration can make invalid cells after the check.
- Commit a268047 has a Sonnet co-author trailer (process note only).

## From W3b-0 (2026-11-05)
- P1 limits (sm-mesher DESIGN.md): rows written at command time on edges that a later webcut replaces fall back to the (firmness, seq) tie-break, and a serde-loaded intent (seq 0) may plan differently; a default-firmness `mesh.size` never overrides an older soft request on another entity. Add tests before W3b-SUBMAP or any path that loads IntentLayer through serde.
- Move `retire_mesh_bulk` (sm-mesher) into `sm_layer_api::retire_mesh` so every caller gets the range path (orchestrator, frozen crate).
- `mesh.delete`: the remaining ~4.5 s at 10⁶ cells is one SHA-256 per id; a parallel, order-independent sum needs rayon in sm-op-api.
- sm-rt `RegionInfo.tags`: not carried in cache images, and hashed without a presence marker; fix when a producer first sets tags.
- DAGMC reference h5m: volume 177 is in both `mat:Graveyard` and `mat:Steel_comp`; EXPORT must not take it as a one-material-per-volume example.

## From W3b-INT acceptance (2026-11-07)
- w3b-report.md: number the deviations D1-D8 consistently; record the CI result (rc=0, 59-70 min); name the Chrome and playwright-core versions used by the viewer smoke.
- tools/w3b_driver.c: the chord tolerance has a (1+chord) factor that R7 lacks (drop it or note it); `ridx()` would read `sig[-1]` if a segment came back with region SMRT_COMPLEMENT under ComplementPolicy::Outside (unreached today; guard it).
- sm-server `smrt_regions`: ByCellSets resolves sets by name, so a non-block cell set with a block's name could be picked; add a comment or test.
- PARITY Transforms row: M1 checks only volume counts and total volume, not "the merged interfaces"; reword or point to M4/M6.
- Open for a ruling: `.smrt` has one region per block (volume identity is not in the file); models with volumes in no block fall back to volume-id names with a note (or should that be a typed error?); FOAM-WARP `split_warped_faces` needs a `session.export` option like P5.

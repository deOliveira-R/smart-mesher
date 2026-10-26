# W0a orchestrator decisions (2026-10-23)

Answers to the open questions in `mesh-api-recommendation.md` §6 and `op-api-recommendation.md` §6. The final crates implement the recommendations **plus** these decisions.

## sm-mesh-api (base: `w0/mesh-api/facefirst` + grafts in recommendation §3, defects §4)

1. **Partitions/ghosts:** partition-local views are *not* `MeshRead`; they get a separate trait in the distribution crate later. v1 reserves `FaceKind::PartitionCut` and `validate` accepts it.
2. **Non-conformal records:** deferred to the `interfaces` domain. v1 reserves `FaceKind::NonConformal` only.
3. **High-order node order:** `smartmesh.nodes.v1` = Gmsh ordering for template shapes up to degree 2. Everything else returns `Unsupported` in v1, and so does mirroring high-order cells.
4. **Parents/active:** add `cell_parent: Option<CellIx>` and `cell_active: bool` columns now; ids depend on this (addendum §7.2). Topology-changing edits come with the refinement crate.
5. **Explicit ids (`finish_with_ids`):** not now. Ids come from the shared allocator, and mesh import is deferred.
6. **`columns()`:** required. Performance is a requirement, and lazy implementors are not.
7. **serde:** done. `sm-core` has optional `serde`/`schemars` features. `EntityId` serializes as `Kind#serial` (text), and binary backends use `to_raw()`.
8. **Cargo.lock:** the merge updates it.
9. **Canonical face order:** keep `(left, right, loop)`. Exporters permute.

## sm-op-api (base: `w0/op-api/registry` + grafts in recommendation §3, defects §4)

1. **HEAD:** `HEAD` is per-checkout state and lives in gitignored `.cache/`, like git's HEAD. Branch refs (`timeline/refs/heads/*`) are committed, one file per branch. ARCHITECTURE §4 gets amended.
2. **Dependencies:** allow `serde_json`, `toml` (drop the hand-written TOML parser) and `sha2` (don't keep hand-written crypto).
3. **sm-core serde:** done (see mesh 7). Ops use `EntityId` directly.
4. **Node identity:** the node hash excludes the result. The recorded result check uses the quantized state hash. A mismatch is a strict `Diverged` error by default, because determinism is a binding rule.
5. **Cross-layer reactions (`on_retired`):** run in the topological order of the concept DAG (`docs/graph/concepts.yaml`), not in name order.
6. **Undo:** moves the branch ref (git-reset style), with the reflog in `.cache/`. Abandoned nodes stay in `timeline/ops/` (content-addressed), so any undone experiment can still be checked out or forked.
7. **File I/O:** `fs::save/load`, atomic writes and `project.toml` move to `sm-engine`. `sm-op-api` keeps pure `to_files`/`from_files`.
8. **Large layers:** add an optional persisted-snapshot hook to `LayerType` now (default: none), so the storage API can plug in without a breaking change.

# W0c orchestrator decisions (2026-10-23)

## sm-core changes (done)
- New kinds: `Prototype=36`, `Lattice=37`, `Symmetry=38`, and per-rank decomposition kinds `PartitionCell=48` (piece), `PartitionFace=49`, `PartitionEdge=50`, `PartitionVertex=51`. No `dim_of` needed.
- `Transform`: `translation`, `reflection(n, d)`, `rotation(p, axis, angle)`, `compose`, `inverse`. `IDENTITY`, `det` and `is_improper` already existed.

## sm-geom-api + sm-topo-api (base: `w0/geomtopo/cellcomplex` + grafts §3 + must-fix §4 of `geomtopo-recommendation.md`)
- Q1 merge tolerance: `merge_vertices` takes an op-level `max_gap`; edge and face merges inherit it. The discarded gap is recorded in `LineageEvent::Merged { gap }`.
- Q2 hash rounding: quantization step = `ModelTolerance::linear × 2^-10`. The same rule is used for the mesh `geometry_hash`. Document it as the cross-platform hash rule.
- Q3 Cargo.lock: the orchestrator owns it and regenerates it at merge.
- Q4 NURBS: keep the small reference de Boor evaluator in sm-geom-api, so the contract is complete.
- Q5 overlay ids: use the per-rank kinds above.
- Q6 `CoedgeKey{face, edge, side}` + lineage are enough for P0–P1. No new kinds.
- Q7 geometry queries the semantics layer needs (`face_moments` by quadrature within tolerance, `face_edges`, dense `face_ordinal`, `coincident_faces(f)` with imprint/merge status) live in the sm-topo-api read trait. The final read trait **must cover every query in `semantics-recommendation.md` §5's `TopologyView`**.
- Q8 instancing: **not in sm-topo-api**. Prototypes, instances, lattices and their locations live in sm-model-api (Model layer, ir.md §5). Each topology complex is self-contained, and a STEP assembly maps to model-layer instancing on import.
- Q9 transform ops: in sm-core (done).
- Q10 P0 surfaces: a single-nappe parametric cone plus a double-nappe implicit cone; ring torus only (R > r). Spindle/horn tori come later.
- Lineage: one `Lineage` type, owned by sm-topo-api (sm-naming's P0 scope is folded into topology). Topology cut operations return `CutOutcome { lineage, caps }`.

## sm-model-api (base: `w0/semantics/tables` + grafts from grouptheory per `semantics-recommendation.md`)
1. Kinds: done (see sm-core).
2. Transform ops: done.
3. Topology queries: slices are fine; `face_ordinal`, `face_moments`, `face_edges` and `coincident_faces` are added to the sm-topo-api read trait (Q7). sm-model-api **depends on sm-topo-api** and uses its read trait instead of a private `TopologyView`.
4. Lineage: use sm-topo-api's `Lineage`.
5. Interface validation reads topology's `coincident_faces(f)` status. Never infer coincidence from moments.
6. Translation symmetry is declared, not detected (P0–P1). Confirmed.
7. Mixed-material cells are out for P0–P1. Volume coloring is an exact partition. Confirmed.
8. Determinism test for API crates: run twice and compare hashes, plus a cross-thread hash comparison (`std::thread`). No rayon in API crates.
9. Cargo.lock: the orchestrator owns it.
10. An uncolored boundary face is a **Warning** in validation, and an **Error** at export time if the target format requires boundary colors. Cut caps take the `cap_color` passed to `apply_cut`.

# W0d orchestrator decisions (2026-10-23)

Merged: sm-geom-api + sm-topo-api (faa768b), sm-model-api on top (7433b24). The deviations are listed in each crate's DESIGN.md.

## Sign-off
- **Transformed interfaces (residual of W0c model decision 5):** an interface with an explicit transform has its two face sets in different places, so no `coincident_faces` record can exist for it. For P0–P1 the counterpart test stays as overlapping bounding spheres from moments, and it reports a Warning, not a proof. The real check, which maps one side by the transform and then matches imprint/merge status within tolerance, belongs to the `interfaces` domain (W3). An interface without a transform still requires a recorded coincident pair.

## Follow-ups (not blocking, tracked for W1/W3)
- geom hash: scale the `quantize_scale_free` zero cutoff by the largest magnitude in each definition, so cancellation noise after large translations hashes as zero.
- model composites: match Part/Whole faces by base face across members, so interface faces are internal; make `pieces_of_face` agree with `piece_faces`; include `PieceShape::Sweep` CutIds in the `check_integrity` max-id scan.

# W0e orchestrator decisions (2026-10-23)

Merged: sm-scheme-api (5dbf6de; base = the `constraint` candidate, recommendation in `scheme-recommendation.md`) and sm-testkit (8dc2ef6). The losing candidates are kept as `archive/w0/scheme/*` tags.

## API change requests (all three scheme designers raised the first three independently)
1. **Classification onto decomposition entities:** accepted. `Classification::dim` accepts `PartitionVertex` (0), `PartitionEdge` (1) and `PartitionFace` (2), with `Param::fits` allowing the parameters that make sense (Curve/Surface when the virtual cell has a carrier, otherwise None).
2. **Cut faces get persistent ids:** accepted. Keep the W0c Q5 rule that the topology overlay mints `Partition*` ids. Add a virtual volume split to the sm-topo-api `Overlay`, which creates the cut face, its edges and its vertices as `Partition*` cells. sm-model-api's `Decomposition` records the `CutId → overlay ids` mapping (persisted, never reused, deterministic), and a resolver returns the cells of any `PieceFace`. That lets split pieces be meshed and classified.
3. **schemars features** on sm-topo-api and sm-model-api for the types used inside `MeshIntent`/`Plan`: accepted.
4. **Duplicate transform algebra** in sm-scheme-api: remove it and use sm-core's `Transform::compose`/`inverse`. Index-map helpers stay.
5. **Surface-only meshes:** accepted as a requirement (shell elements, sheet bodies, surface previews, deck-face checks). Recommended design: sm-mesh-api meshes carry a topological dimension, and a dim-2 mesh keeps the face-first invariants one level down (cells are polygons, facets are edges, facet→(left, right)). The fixer and the reviewer may choose otherwise only with a written reason.
6. **Many-to-one sweep with several source faces into one target:** blocking. It is a core Cubit capability and needs a real test.

# W0f/W0g: contract freeze policy (2026-10-23)

W0f applied the W0e batch plus 58 accepted gap fixes over four critique rounds (21, 12, 10 and 15 accepted per round, merged at 0c24cfa). It did not converge: an open-ended "anything awkward to express" criterion over about 100 PARITY items finds new items every round. W0g therefore audits only for changes that cannot be made later, and the contracts freeze in two tiers:

- **Frozen** (a change breaks stored projects or every worker branch): entity ids and kinds, hashes and their quantization, persisted/on-disk/wire formats (serde shapes, tagging, field names, defaults), enum shapes (every growable public enum is `#[non_exhaustive]`), op/query naming rules, and the trait signatures that implementation crates implement.
- **Open** (through `api_change_request` during waves, batched by the orchestrator): additive methods, new variants behind `#[non_exhaustive]`, new provided trait methods, docs.

Known correctness restriction: sm-mesh-api `unfold` accepts only commuting involutions (mirrors, half turns) and returns `Unsupported` for other deck groups instead of a wrong result. Whether rotational sectors need a group-description input is decided by the W0g audit.

## W0g result: contracts frozen (2026-10-24)
The freeze audit accepted 14, 5, 4 and 2 items in its four rounds (merged at the commit after f01249d). Apart from two hash-framing fixes (prefix-free pcurve hashing, per-quantity quanta in the mesh quantized hash), all were `#[non_exhaustive]` markers with constructors. The semantic-breaking lens found nothing after round 1. Rotational/dihedral unfold is additive (a new group-based entry point next to `unfold`) and is scheduled with the tracking/export work.

Residual: about 140 public types stay exhaustive by design (sides, senses, dims, keys, planes, axes, snapshot/node DTOs, derived views). Growing one of them is an orchestrator-coordinated change: an `api_change_request` plus a fix-up of all users in the same merge.

**W0 is complete. The contract crates are frozen under the two-tier policy above.**

# W1 orchestrator decisions (2026-10-24)

W1 raised 79 ACR strings from eight implementation crates. After dedup there are 44 distinct items. Each claim was checked against the contract source at 1b5cca0. The three frozen-tier questions were settled by experiments in a scratch worktree (`.claude/worktrees/w1-acr-triage`), and nothing from those experiments is committed. Rulings follow the W0f/W0g two-tier policy.

## Contract bugs (fix now; behavior fix, no signature change)
1. **sm-topo-api `end_op` ignores `Undo::Flags`:** accepted as a bug. The `touched`/`check` seeds in `complex/ops.rs` cover only `Alloc|Row|State|Carrier`, so a flag-only op such as `set_flags(pole_edge, NONE)` commits states that `validate` rejects (`ShellOpen`, `EdgeEnds`). Fix: seed `Undo::Flags { id, .. }` into both sets. sm-topo copied the same omission (`lifecycle.rs:55`), so it gets the same fix in the same merge.
2. **sm-op-api `Session` unborn-branch guard:** accepted as a bug. When `head == Head::Ref(name)` and `name` has no ref (after an undo to the root), `create_branch_at(name, h)` and `rename_branch(_, name)` create the ref while `position()` stays `None`, so the in-memory session disagrees with what a reopen produces. Fix: both calls return `TimelineError::HeadOnBranch(name)` in that case. Only an error path changes.

## Open-tier additive changes (accept)
3. **`sm_topo_api::check_cells` + `check_cells_with(open_shell)` public:** accepted. This removes the roughly 150-line copy in sm-topo. sm-topo's long-row `Inserted` optimization stays local.
4. **`normalize_row` / `canonical_rotate` public:** accepted. Canonical rows determine the hashes, and every implementor needs them byte-identical.
5. **`canonical_words` + doc that `canonical_visit`/`canonical_hash` may be overridden if the word stream is identical:** accepted. This is docs plus one helper.
6. **`run_topology_conformance` in sm-testkit:** accepted, with a factory parameter instead of `Default` (the scenarios need `with_tolerance`). Both sm-topo-api and sm-topo run it, so the reference and the production store execute identical scenarios. This removes 8 copied test files (about 4,000 lines).
7. **`LineageLog` dense `(kind, serial)` index:** accepted as perf-only with no signature change. Land it only with a benchmark showing the gain, and keep iteration order ascending by id.
8. **`CellShape`/`FaceKind` `code()`/`from_code()` + `Basis::from_code()`:** accepted. The codes equal the declaration discriminants that `hash_connectivity` already hashes (`as u64`) and the `.smsh` tables (0..=7 and 0..=3). They are documented as frozen, and new variants append.
9. **`validate_classification` / `validate_records` public:** accepted. `validate(m)` output must stay byte-identical (golden test). This removes the hand-mirrored `Validation::Fast` rules in sm-mesh, whose copy has already drifted (the face-curve node-range hole).
10. **`ExtrusionMap::n_base()` / `n_extruded()` / `n_layers()`:** accepted. These are trivial accessors.
11. **`MeshBuild::add_template_cells` + `reserve` (provided):** accepted. Defaults loop `add_cell_from_vertices` / do nothing. An override must produce the same `connectivity_hash` as the default.
12. **`MeshRead::cell_normal` (dim-2, provided):** accepted, narrowly. It returns the Newell area normal of the corner cycle (sm-quality's `SurfaceNormals::Own`) and `None` for dim-3. The carrier-oriented normal stays with `FaceNormals`.
13. **Optional `bytemuck` feature on the dense index types and `SignedFace`:** accepted. They are all `repr(transparent)` over `u32`, the feature is off by default, and the orphan rule plus `forbid(unsafe_code)` leave no downstream alternative.
14. **`PCurve::d2`:** accepted. It returns zero for `Line2`/`Polyline2` and de Boor for `Nurbs2`.
15. **`eval_many` provided on `ParametricCurve`/`ParametricSurface`:** accepted. Contract: `out[i] == eval(in[i])` bit-for-bit, equal lengths (panic otherwise), so a rayon override stays deterministic.
16. **`ParametricSurface::ray_hits_tol` (new provided method, `tol: &ModelTolerance`):** accepted. It adds one `Crossing::Tangent` hit at a near-miss closest approach within `tol.linear`. `ray_hits` and its results are unchanged.
17. **`sm_geom_api::roots` public (`Root`, `solve_quadratic`, `real_roots`):** accepted in sm-geom-api, not sm-math, because an API crate must not depend on an implementation crate. sm-geom drops its `quadratic`/`poly_roots` copies once a differential test shows they agree. `brent`, `horner` and `solve_dense` stay local.
18. **`State::can_snapshot` via provided `LayerType::supports_snapshot() -> bool { false }`:** accepted. `CodecError` is a `String` newtype, so adding an error kind would change its shape.
19. **`Session::trim_reflog(keep)`:** accepted.
20. **`Session::open_with_policy(.., policy, ctl)`:** accepted. `open_with` stays strict. Under `Warn`/`Rebaseline` a mismatching seed is skipped with a warning, not an error.
21. **`Engine::apply_record_canonical` public:** accepted. It exists today as `pub(crate)`.
22. **`batch::is_reference(&str) -> bool` public:** accepted. sm-lang's `is_batch_ref` duplicates the contract grammar.
23. **`sm_core::ENTITY_ID_PATTERN` const:** accepted, as a plain `pub const` (not feature-gated) that the `JsonSchema` impl uses. sm-lang's `ID_PATTERN` duplicates it.

## Frozen-tier / behavior-changing (decided by experiment)
24. **`EntityId` over `NonZeroU64`:** accepted. Kind byte 0 is never valid, so `new`/`from_raw` already guarantee a nonzero value. The experiment gave identical `to_raw`/`from_raw` bits for every kind at serials {0, 1, 42, MAX}, identical `Kind#serial` text with a round-trip, `from_raw(0) == None`, and `size_of::<Option<EntityId>>() == 8`. Derived `Ord`/`Hash` are unchanged. The workspace builds clean with all targets and all features, and every workspace test passes except one wall-clock test that fails without the change too (see 36). The persisted, wire and hash forms are byte-identical, so the frozen tier is untouched.
25. **Tighten `check_knots` interior multiplicity to `<= degree`:** deferred. 4 sm-geom tests fail under the tighter rule: the `fix0_curves.rs` C^-1 interior-knot curve and surface tests, `plane_hits_of_repeated_knot_curves_are_on_the_plane` and `repeated_knots_and_high_degrees_match_the_reference`. They are W1 regressions that deliberately support C^-1 knots, so sm-geom already handles them. Revisit with STEP import.
26. **Tolerance-aware tangency inside the reference `ray_hits`:** rejected in that form, because it would change reference results. Covered by 16.
27. **`Point2`/`Vector2` in sm-core:** rejected. 2-D code uses `[f64; 2]` or `sm_geom_api::Uv`. A third 2-D type adds conversions without adding safety.
28. **`Mat3` alias in sm-core:** rejected. An alias of `[[f64; 3]; 3]` names nothing new, and `Transform::m` already spells it.

## Rejected / deferred (contract)
29. **`CurveDef::Parabola` / `Hyperbola`:** deferred. `CurveDef` is `#[non_exhaustive]`, so the change is legal, but a rational quadratic NURBS already represents both conics exactly. New variants would add projection, hashing tags (`for_each_value` 5/6) and a `carrier_words` path to every implementor. Revisit when STEP import needs the native entities.
30. **`cell_corners_into(&mut [VertexIx; 8])`:** rejected. Dim-2 polygons can have more than 8 corners, it is still a virtual call, and the requester reports the cost as small. A corner column in `TopologyColumns` is deferred until profiling shows a need.
31. **Intersection trait / shared result types in sm-geom-api:** rejected. The request itself proposes nothing, and consumers depend on sm-geom.
32. **`finish_with_ids`:** stays deferred (W0a mesh decision 5), with `.smsh` id-preserving import. Scheduled with mesh import.
33. **`OpInfo` positional marker, output role names and cardinality:** deferred to a W2 design ACR, together with the `x-entity-kind` convention (35). `OpInfo` is `#[non_exhaustive]`, so the change is additive, but role declarations touch every op and need one design.
34. **O(live) `State::entities_of`:** deferred. It is internal perf with no signature change, and the retired set is normally small relative to `next`. Needs a measurement first.

## Not contract changes (routed to W2)
35. **Routed:** the `topo.related` query (sm-topo / sm-engine registers it), `Host` over `Session`, `timeline.head()`, `session.*` script commands, the `x-entity-kind` schema convention, Deck → named face-set lowering, and the ARCHITECTURE §4 amendment.

## Note
36. **Pre-existing timing failure:** `sm-topo/tests/ref_reference.rs::building_many_bricks_is_near_linear` (release build, 10,000 bricks through the paranoid `Dual` twin) takes about 604 s against a 2 s budget. It fails on main without any contract change. Fix it in sm-topo: build through `TopoStore` without the twin, or turn paranoid mode off for the timing assertion.
Merged: W1 crates (sm-meshio, sm-topo, sm-quality, sm-engine, sm-math, sm-geom, sm-mesh, sm-lang) at 1b5cca0; contract batch groups topo 2738c4f, mesh c52cb2f, geom afe2fa6, op ebf5a9c.

# W2 orchestrator decisions (2026-10-24)

Plan: `docs/design/w2/plan.md` (task specs T1–T13, stage-0 contract changes ACR-1..5, milestone test §4).

1. **Blocks are the volume color.** `sm_model_api::Model` keeps one exact volume partition (W0c model 7), so `fem.block_add` writes that partition: a volume's block *is* its material color, and each volume (and so each element) is in exactly one block. This matches Exodus (one block per element) and the transport consumer, which resolves colors to materials. Cubit-style overlapping blocks can come later as a new `Group`-backed `fem` op without breaking stored projects. Sidesets are face colors.
2. **Meshed volumes are not transformed in W2.** `model.move/rotate/scale/reflect` on a meshed volume fails with "mesh.delete first". Meshes on deleted volumes are marked `stale`, not deleted. Moving the mesh with its geometry is W3.
3. **Export is not an op.** It is the protocol method `session.export` and the script command `session.export(...)`, a pure function of the state (`sm_fem::export`).
4. **Stage 0 before workers:** ACR-1 (scheme stubs behind the `stubs` feature, clippy-clean, since CI runs `--all-features`), ACR-2 (`OpInfo` output roles, cardinality, positional marker, `x-entity-kind`), ACR-3 (new contract crate `sm-layer-api`; frozen only after compiling `create_brick` and `mesh.run` sketches pass in its tests), ACR-4 (new crate `sm-protocol`, v0, unstable until W3), then ACR-5 (skeleton crates with fixed entry points, plus the defaulted `Host::session`).
5. **Stage 1 runs on Anthropic Cloud** (all tasks at once, no local 8-worker cap). Cloud sessions have no general internet, so the tasks that depend on external specs get them as repo docs (decision 6), and the local Opus reviewers check against the real sources.
6. **External specs are in the repo, not in task text.** `docs/research/mcp-2025-06-18.md` (MCP server excerpt) and `docs/research/cubit-meshing-semantics.md` (Cubit curve schemes, intervals, matching, map and vertex types, paraphrased). The second also fills Cubit's unpublished parts (auto size formula, size→count rounding, dualbias with odd n, the curvature density) with binding `[Ours]` decisions. Where the plan fixes something, the plan wins.
7. **Stage 0b (review follow-ups, before freezing):**
   - `ApplyCtx::record_role` reports surviving ids in output roles. POSITIONAL and OUTPUTS are validated at registration.
   - The catalog is owned and deserializable (`OpInfoOwned`/`QueryInfoOwned`, including `tool_name`). The `sm-protocol` result types are `#[non_exhaustive]` and have constructors.
   - In `sm-layer-api`:
     - `insert_mesh` and `retire_mesh` are the only valid mesh lifecycle path. The minted set must equal the mesh's ids exactly.
     - `model_txn` retires removed record ids.
     - `topo_txn` can report survivors in roles.
     - Retired-id lookups are sorted and binary-searched.
   - Binding worker notes are in plan §3 "Stage-0b notes".
8. **Export with no mesh named:**
   - Exactly one mesh is exported.
   - No mesh is a `no-mesh` error.
   - Several meshes is an `ambiguous-mesh` error.
   
   The `Request::Export` doc is updated to match.
Merged: stage 0 at f001d6d (ACR-1 029c6d5, ACR-2 f4ee040, ACR-3 59e0fdc, ACR-4 75dd25d); stage 0b at e29d8ba (polish 2701085, skeletons 6ae9cde).
9. **Stage-1 review rulings:**
   - **`mesh.run` neighbour guard** (refusing a volume that shares a face with a meshed volume outside the targets): kept for W2, because it is a conformity safety net. Meshing against an existing neighbour mesh, Cubit's sequential meshing, is W3.
   - **`mesh.default_size`:** accepted into the catalog.
   - **Size→count rounding:** the frozen `sm-scheme-api` `size_count` rounds half away from zero (L=10, size=4 → 3). `docs/research/cubit-meshing-semantics.md` is amended to match. Cubit publishes no rule.
   - **10⁶-cell `mesh.run`+`mesh.delete`:** 10.6 s against 4.7 s for `Driver::mesh`, from id adoption, retirement and the digest. Plan risk 3 is partly realized. A bulk adopt/retire path and `MeshBuild::add_template_cells` commit are W3 items.
   - **Dependencies outside the plan's per-task Deps lines:** serde, schemars, thiserror and rayon (required by the common rules), plus test-only crates. Accepted for all W2 crates. The Deps lines are upper bounds on internal crates only.
   - **Planner firmness cascade** (edge requests always beat volume requests regardless of firmness, against Cubit's rule) and `Driver::mesh_from_plan` (no double planning): queued as a `sm-scheme-api` ACR for W3.
   - **Queued contract requests for the post-integration batch:**
     - `Model::uncolor_volumes`
     - optional `OpError::Unsupported`
     - `face_moments` on pole surfaces (CR-1) and the `FaceMoments::plane` centroid fix (CR-2)
     - an explicit no-outputs marker (CR-4)
     - normalization overflow in `Transform::rotation`/`reflection`
     - `IntervalSink::goal`
     - re-exports of `Rev`/`Hash256`/`EntityId` from `sm-protocol`
     - the failure form of `Request::ScriptRun`
   - **`ParametricCurve::breakpoints`** is done right away. Without it the curve arc-length map misses 1e-9 on NURBS with very short, high-contrast knot spans, the only open finding of T4.
10. **Integration rulings (W2 milestone green at 38d48e3; merged with the curve and sizing follow-ups at 2efc34b):**
   - **NURBS equal-spacing tolerance (T4):** 1e-9 relative plus one parameter ulp (`eps·max(|lo|,|hi|,1)`) times the edge's maximum speed. Under the frozen fraction-output contract a node near a fast short span cannot be placed closer than that; on a narrow edge a fraction ulp is far below one parameter ulp, so the parameter ulp is the achievable reading. Plan T4 is amended.
   - **`fem` layer hash in milestone row 1:** empty layers are left out of `layer_hashes()`, so the milestone script sets `fem.block_element_type HEX8`, which makes the layer non-empty. This adds an op and weakens nothing.
   - **MCP text vs `structuredContent`:** with sm-mcp's `respell` feature on, the text may be the script spelling (`sm_lang::respell_json`) of `structuredContent`. Tests accept either form.
   - **`map (surface)` P0:** for W2 it is satisfied by surfaces meshed inside a volume run. `mesh.run` on faces alone (Cubit `mesh surface`) is W3.
   - **Block element type in output:** for linear elements the type follows from the cell shape and is checked at export. Writing the element type into `.smsh` is queued for the polyhedral-format work (W3).
   - **sm-cli → sm-engine:** sm-cli uses sm-server's re-exports only. Dropping the direct dependency is in the follow-up wave.
   - **Follow-up wave after integration:** end-to-end tests through `standard()` for `mesh.intervals`, auto size and `mesh.default_size`, explicit `equal`/`map` selection, and `mesh.info` scheme reporting; the curve sliver at edge ends within ulps of a knot; `Eq` rows in the sizing brute-force test.
   - **Follow-up merged (2026-10-25):** P0 end-to-end tests through `standard()` (`crates/sm-cli/tests/p0_rows.rs`), sm-cli without sm-engine, the curve end-sliver fix, and `Eq` rows in the sizing brute-force test. Open: no query reports which scheme ran for each entity (`mesh.info` has counts only); a `mesh.plan`/explain query is queued for W3. `sm-mesh` `contract_alloc` showed one flaky stray allocation once under full CI.
11. **Contract batch and polish wave (2026-10-25):**
   - **Contract batch** (merged at 2c7a065, all changes additive):
     - sm-core accepts the script spellings of entity kinds (`Face#1`, `GeomSurface#7`) as aliases. `Surface`/`Curve` stay the geometry carriers. Output stays canonical.
     - `Transform` rotation/reflection use a scaled norm outside [2^-500, 2^500], and are bit-identical inside that range.
     - `OpError::Unsupported`.
     - The `TypedOp::NO_OUTPUTS` marker, which replaces the hand list in sm-server's registry test.
     - sm-protocol re-exports `EntityId`, `Hash256`, `Head` and `Rev`.
     - `Model::uncolor_volumes`, so `fem.block_remove` deletes non-empty blocks (with a `block-uncolor` warning).
     - `IntervalSink::goal`.
     - The `sm_scheme_api::Param` re-export.
     - CR-1 (pole-face moments), CR-2 (`face_plane`), and the CR-3 doc.
     - Orchestrator ruling: the sm-testkit conformance check that expected pole faces to be refused now asserts the sphere's exact area and volume.
   - **Stub `map` name collision:** the frozen stubs stay unchanged. sm-scheme-map's tests build their own registry.
   - **Dependencies:** decision 9 applies. The per-task Deps lines bound internal crates only. Also accepted are dev-dependencies on contract crates (e.g. sm-mesh-api for `MeshRead`/`validate` in tests) and rayon where its use is deterministic and tested. The plan's Deps lines are not amended one by one.
   - **`script.run` failure form:** the form documented on `Request::ScriptRun` (`script-error`, `data` = `ScriptResult` + `rendered` + `error_code`, partial state kept) is the protocol contract. sm-mcp and sm-cli follow it.
   - **MCP errors:** `invalid-params` stays JSON-RPC -32602, per the MCP spec for invalid arguments. Application errors stay `isError: true`.
   - **MCP `respell`** stays off by default, so text content equals `structuredContent` JSON. Both spellings are accepted as arguments.
   - **sm-cli project discovery:** the current default is kept.
   - **Face colors:** every `Domain::Face` color lowers as a sideset in W2. A non-sideset face color needs a domain or tag in the model contract (W3 if a plugin needs it).
   - **Polish wave** (13 cloud workers, each Opus-reviewed, merged after 2a1f879): the review notes collected through W2 are resolved or marked obsolete. Each crate's W2_REPORT.md has a "Polish wave" table.
   - **Remaining contract and feature work:** `docs/design/w3/backlog.md`.

12. **W3a rulings (2026-10-25, from the W3 planning workflow: 14 task drafts, ray-tracing design panel, cross-task critic):**
   - **Wave split.** W3a: booleans/imprint/merge/webcut, STEP import, submap, pave, tri, sweep, tetmesh, smoothing, Exodus, OpenFOAM + `.smsh` 1.1, tet→poly, validation/quality diagnostics, ray tracing, milestone. W3b: auto scheme selection, PyO3, web viewer, DAGMC h5m + Gmsh msh4 export, remaining primitives, surface-only `mesh.run`, `mesh.plan` query, script record, STEP writer if it slips, and the deferred backlog contract items (bulk mesh commit, plan once, firmness cascade, scheme-derived counts, overlay `MeshRead`). W3a performance targets that depend on bulk commit are recorded, not gated.
   - **Layers.** sm-quality moves to L2 (its deps are sm-core + sm-mesh-api only). New crates: sm-boolean L2, sm-cadio L2, sm-exodus L2, sm-scheme-{submap,pave,tri,sweep,tet} L3, sm-smooth L3, sm-poly L3, sm-diag L3, sm-rt L3, sm-rt-capi L4.
   - **Dev-dependencies on peers.** Allowed when acyclic and the peer is already on main. A worker never depends (dev or normal) on another W3a crate that is still being built; fixtures in sm-testkit decouple them.
   - **Booleans** live in sm-boolean (`BooleanPlugin`), not sm-model. Webcut ops output `positive` and `negative` roles (pieces on the +normal / −normal side) in addition to `bodies`/`volumes`/`cut_faces`. `merge` takes an explicit skip list (interface faces from the `Model` boundary tables), filled by the op. Mandatory: webcut plane, imprint, merge, unite/subtract/intersect with regularize on planar and quadric faces, including a unite whose input shares a merged face with a volume outside the op. Stretch: webcut body/sheet, imprint curves, NURBS faces.
   - **Ray tracing gets a frozen contract `sm-rt-api` (layer C)**, overriding the critic's "no contract until a second implementor". Reason: the transport code is an external consumer, and the contract lets the C ABI/DAGMC-compat crate and the tracker be built in parallel. Base: the panel's mesh-native design (both judges: 8.2/8.0 vs 6.8–7.5 vs 5.9–6.0) with grafts: `LEVEL_INLINE = 8` (spills, no hard cap), caller-owned `Scratch`, `distance_to_boundary` peek, boundaries derived from `sm_model_api` (`FaceAction`, `ResponseKind` incl. `Albedo`), `RecoveryPolicy`, composite `CacheKey`, `TrackerInfo` (prototypes, copies, memory_bytes, faceted_faces), `Precision`, DAGMC-compat provided functions, stable negative `RtError::code()`, thread-local last error in the C ABI. W3a sm-rt implements VOLUME_MESH + EXACT_PLANAR (cell walking), facet input and BVH locate; ANALYTIC/HIERARCHY backends are W3b. The tracker cache is a `.smrt` sidecar in the contract's `CacheImage` format; embedding as a `.smsh` chunk is W3b. No `tracking` field in the protocol in W3a.
   - **STEP import is content-addressed:** `model.import_step {source: {sha256} | {inline}}`, never a path. Bytes come from an injected `StepInputs` service (sm-server provides it; CLI/MCP/lang stage a path into the project's inputs store and rewrite it to `{sha256}`). No `ApplyCtx::blob` contract change. STEP writer is stretch.
   - **Exodus** is its own crate sm-exodus.
   - **Element type names:** internal and `.smsh` 1.1 use `POLYHEDRON` / `POLYGON`. Exporters map them (Exodus `NFACED`/`NSIDED`, VTK `VTK_POLYHEDRON`, OpenFOAM faces).
   - **Poly:** `mesh.to_poly {mesh, keep_source?: bool = true}` converts a whole all-tet mesh entry into a new mesh entry. Partial dualization of a mixed mesh needs conformize and is deferred. Output must pass `validate` + `validate_geometry` with default tolerances: non-planar dual faces are split into planar pieces (volume conservation exact). The milestone exercises poly in a timeline fork where the part is tet-meshed.
   - **Scheme names and provisional scores:** `trimesh` (Delaunay default), `tridelaunay`, `triadvance`; no `tri` alias (that is the stub's name). Surface scores: map 100, submap 80, pave 40, trimesh 30. Auto lists stay unchanged in W3a, so W2 hashes are unchanged; auto selection is W3b. Real scheme crates reuse stub names; test registries must not combine stubs with the real crate.
   - **Tri → tet surface quality:** tri documents a 20° minimum angle where the fixed boundary allows it; tetmesh must accept any valid closed surface and its quality thresholds are calibrated against 20° input. No boundary Steiner points in W3a (shared surface meshes stay conformal).
   - **Milestone:** tetmesh volumes get `mesh.scheme trimesh` on their faces explicitly (the V2|V3 interface makes V2 a prism sweep: true 6-node prisms).
   - **Diagnostics:** one worker owns the sm-quality FV-metric delta and the new sm-diag crate (registers `mesh.validate`, `mesh.quality`, `mesh.flag_cells`), in that order (one-crate exception). The suggestion table reads the op registry rather than hard-coding which ops exist.
   - **Smoothing** pins deck-pair nodes in W3a (tied smoothing is W4).
   - **Stage 0 contract work:** `TopologyEdit::join_edges` / `join_faces` (default `Unsupported`; implemented in CellComplex and TopoStore), topology fixtures (torus, cone, brick_with_void, nested_spheres), `MeshLayer::replace_mesh` + `add_mesh_set`, `GenCtx::loop_uvs`, `GenerateFailure::with_nodes` (serde default), `ExportFormat::{Exodus, Openfoam}` + `ExportResult.files`, CR-2 completion, `ModelLayer` from `(TopoHost, Model)`, sm-testkit fixtures (planar_face, annulus_face, sweep fixtures, surf3, tet_block family, lattice, nonconvex_poly, pin_assembly, boolean cases, w3a part + STEP), the sm-rt-api crate, skeletons for the new crates, and the `unsafe` exception for sm-rt-capi.

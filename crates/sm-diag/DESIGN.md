# sm-diag: design

## Positioning

**Similar to** OpenFOAM `checkMesh`, Fluent's mesh check (orthogonal quality) and the quality reports of Cubit/Verdict, **differs in** returning structured JSON for MCP and the CLI where those tools print text: every finding is located (entity, classification owner, nearby model entities, a point), tied to the CAD model through classification ("bad cells cluster on `TopoEdge#7`, 4.1 times the mesh average"), and paired with a rule-based suggestion that names a registered op; validity is decided by our own validator (`sm_mesh_api::validate` plus corner-Jacobian validity), never by an external tool, and offending cells can be stored as a `MeshSet` in the timeline; results are bit-identical for 1 or N threads and replayable, **giving** an agent or a script one deterministic call that says whether a mesh is sound, where and why it is not, and what to do next.

## Layer and contract

L3. Dependencies: `sm-core`, `sm-mesh-api` (serde, schemars), `sm-quality` (L2, the numerics), `sm-op-api`, `sm-layer-api`, `rayon`, `serde`, `serde_json`, `schemars`, `thiserror`. No W3a crate. `DiagPlugin` (id `diag`) registers the queries `mesh.validate`, `mesh.quality` and the op `mesh.flag_cells`; it registers no layer and needs `sm_layer_api::CoreLayers`. sm-mcp and sm-cli expose registered queries and ops by themselves. Installing the plugin after `MeshPlugin` and filling `OpNames` is the sm-server integration step.

Files: `validate.rs` (`mesh.validate`), `quality.rs` (`mesh.quality`, clusters, worst entries), `suggest.rs` (the fixed rule table), `flag.rs` (`mesh.flag_cells`), `analysis.rs` (metric columns and grading shared by quality and flag), `locate.rs` (entity, owner, near, location), `common.rs`, `error.rs`.

## Wire rules

* No NaN or inf in JSON. Undefined values are `Option` fields, which serialize as `null`; the engine's canonical codec then drops `null` entries, so over `Engine::run_query` an undefined field is absent. Both mean "undefined". The Verdict sentinel `1e30` stays a number. A non-finite limit (OpenFOAM's flatness never fails) is reported as no limit.
* Entities are `EntityId` strings (`"MeshCell#12"`) plus the dense `index` (`EntityRef`). The id is `null` only for an out-of-range index.
* Every reply has `schema_version: 1`. Maps are `BTreeMap`; lists have a stated total order.
* Every op input and query input uses `deny_unknown_fields`; `mesh` carries `x-entity-kind = ["Mesh"]`. `mesh` defaults to the only mesh of the state (a pure function of the state, safe for replay); with none or several and no id the error lists them.

## `mesh.validate`

Pure core `validate_mesh(m, &ValidateOptions, live)`; the query adds the entry's `stale` flag (`mesh.stale`, a warning).

Sources, in order: (1) `sm_mesh_api::validate`; (2) if no structural code (`ids.order`, `index.range`, `loop.*`, `face.*`, `cell.*`, `shape.*`) was found and `geometry` is on: `validate_geometry` and `sm_quality::validity` mapped to `quality.inverted` / `quality.degenerate`, otherwise `geometry_checked: false` with `geometry_skipped_reason`; (3) `mesh.stale`; (4) `diag.class_retired`: one finding per classification entity that `live` rejects (the query passes `State::contains`), located at the lowest-index affected entity (cell, else face, edge, vertex), owner = the retired id.

Violation: `{code, severity, entity, detail, owner, near, location}`. `owner` is the entity's own classification (a non-model owner is reported as is). `near` is the sorted, distinct classification entities of the entity's vertices whose dimension is below the owner's, at most 8 (none when the owner has no dimension). `location` is the vertex point, edge midpoint, face centroid or cell centroid; a cell centroid that is non-finite or leaves the bounding box of the cell's vertices (an inverted cell) is replaced by the vertex mean; `null` for an out-of-range index. There is no suggestion field; suggestions live in `mesh.quality`.

Report: `valid` (no error-severity violation), `total`, `by_code` (sums to `total`), `by_owner` (top 10 by count then id), `violations` sorted by severity, code, entity kind, index, detail and cut to `limit` (0 to 10000, default 100), `truncated`. `build_report` is public so the order can be tested under reversal of the input list.

## `mesh.quality`

Pure core `quality_report(m, &QualityOptions, ops)`; `ops` answers "is this op registered".

**Metric set.** The profile's rules, or `metrics`. A requested metric is graded only if a rule exists (the profile's, or one created by a `thresholds` override, which takes sense and boundary limit from the first rule of that metric in any profile). Overrides must name a metric of the set. A metric no cell supports (finite-volume metrics on a 2-D mesh, Verdict `Skew` on tets) goes to `skipped: [{metric, reason}]`.

**Columns.** Verdict metrics through `sm_quality::compute`, finite-volume ones through `fv_geometry` + `fv_compute` (once). Face-indexed metrics are graded per face (boundary faces with the boundary limit); `mesh.flag_cells` maps face grades to cells with `face_to_cell(Max)`.

**Per metric.** `count` is the number of entities of the metric's kind: `histogram.counts + below + above + undefined + n_sentinel == count`. `undefined` counts non-finite values. `n_sentinel` counts `|v| >= 1e30`; summary and histogram exclude them, `worst` and the grade counts do not. `n_warn` counts entities graded warn, `n_fail` those graded fail. The histogram spans the finite non-sentinel range. For an ungraded metric the sense is informational (the rule's, else a small fixed table, else higher-is-worse), the threshold is empty and `grade` is `null`.

**Worst.** At most `worst_n` (0 to 100, default 10) finite entries, worst first (value by `total_cmp` in the sense direction, then index ascending); per-chunk top lists are merged in chunk order. Entry: `{entity, value, grade, location, owner, near, cells}` (`cells` only for faces).

**Clusters.** An entity is *bad* if it grades warn or fail. Its touch set is the model entities of dimension at most 2 (`Topo*` and `Partition*`) among the classifications of its vertices (a face's own vertices, a cell's corners or the vertices of its faces). For each model entity, `n_bad` counts bad entities touching it and `n_total` counts entities with a defined value touching it, in one sequential pass over the column. `fraction = n_bad / n_total`, `enrichment = fraction / overall_bad_fraction` with `overall_bad_fraction` over all entities with a defined value. Sorted by fraction descending (exact cross-multiplication), `n_bad` descending, id ascending; the top 10. A metric without bad entities has no clusters. A touch set uses the entity's own vertices, so a surface cluster is diluted by the interior faces that touch the surface: an interior-heavy mesh needs a strongly bad surface to reach fraction 0.5.

**Verdict.** `fail` if any cell is invalid or any metric has a failing entity, else `warn` if any entity warns, else `pass`. `validity` counts come from `sm_quality::validity`.

**Suggestions** (`suggest.rs`, one fixed table, deterministic, sorted by severity then id; severity is `error` if the metric fails somewhere, else `warning`). Suggestions name ops by string only.

| Trigger | Suggestion | Op | `params` |
|---|---|---|---|
| cluster on a `TopoEdge`, fraction >= 0.5 and enrichment >= 2 | `adjust_intervals` | `mesh.intervals` | `{current_count}` = interior vertices classified on the edge + 1 |
| cluster on a `TopoFace`, same thresholds | `change_scheme` | `mesh.scheme` | `{n_bad, n_total}` |
| the same, metric `face_flatness`, `face_warp` or `face_concavity` on the transport profile | `refine_surface` (instead) | `mesh.size` | `{n_bad, n_total}` |
| bad entities of a metric and no cluster with enrichment >= 1.5 | `smooth` (target: the mesh) | `mesh.smooth` | `{n_bad}` |
| inverted or degenerate cells | `fix_geometry_or_scheme`, one per owning volume | none | `{n_inverted, n_degenerate}` |

`args` is `null` throughout (the op's own schema decides the rest). **Availability reads the op registry.** `QueryCtx` has no registry handle, so the host injects `OpNames(BTreeSet<String>)` (sm-server fills it from `Registry::op_names()`); `available = ops(op)`. If the service is absent every suggestion with an op is `available: false` and its reason says "op registry not provided"; if the registry lacks the op the reason says it is not registered. Nothing hard-codes which ops exist: `mesh.smooth` becomes available the day `sm-smooth` is installed. Suggestions without an op (`fix_geometry_or_scheme`) are `available: false`.

## `mesh.flag_cells`

Input `{mesh?, name, profile?, metrics?, thresholds?, level?, include_invalid?, grow?}`; `thresholds` override limits as in `mesh.quality`; `grow` (0 to 5, default 0) adds that many rings of face neighbours to the flagged cells (each ring: every cell across a face of the previous state, so the result is independent of any ordering); `name` is 1 to 64 characters of `[A-Za-z0-9_.-]`. A cell is flagged if any selected metric (with a rule) grades at or above `level` or, with `include_invalid`, the cell is inverted or degenerate. Output role `set` (`Cardinality::Optional`, kind `MeshSet`): a `SetMembers::Cells` set stored with `sm_layer_api::add_mesh_set`, the only place an id is minted. The `meshes` digest changes because sets are inside `topology_hash`. An empty selection succeeds with a `diag.nothing_to_flag` warning, no set and an unchanged state hash. A name already used in the mesh is `OpError::Invalid` (checked before any mutation). `mesh.delete` retires the set id with the mesh. Sets are append-only until `mesh.unflag_cells` (W3b) and flow into every export as ordinary sets.

## Determinism

Everything that touches many entities is chunked over `sm_quality::CHUNK` with one output slot per entity; counts are integers; summaries and histograms are sm-quality's fixed-chunk reductions; the cluster pass and the report assembly are sequential; every ordering has a total order. Tests hash the JSON bytes of both queries and the flagged cells for 1 and N rayon threads, repeated and from concurrent callers (`sm_testkit::determinism::check`), and replay `mesh.flag_cells` from the root with a disabled derived cache.

## Performance

Benchmark (`benches/quality.rs`, record only, not asserted): `quality_report` with the generic profile (two Verdict metrics, six finite-volume metrics, validity, no clusters because nothing is bad) on a 100^3 hex block (1 000 000 cells, 3 030 000 faces), 4 cores: **0.87 s** (0.82 to 0.94 s over 10 samples, release profile). `validate_mesh` on the same block: 3.8 s, dominated by `sm_mesh_api::validate`. Each call recomputes every column (about 8 MB per metric per million cells); there is no cache in W3a (`DerivedCache` is W3b).

## Known gaps

Shell-orientation validation for dimension-2 meshes (needs `FaceNormals` over the topology), equiangle skewness, cell-level aspect ratio in the checkMesh definition, per-material profiles, `mesh.unflag_cells` and a derived cache are deferred (W3b). `quality_report` assumes a structurally valid mesh (every finished mesh is); run `mesh.validate` first on a suspect one. The Transport and Generic thresholds are provisional and the Transport numbers await the user's ratification. Topology-liveness of classification entities (beyond state liveness) is not checked.

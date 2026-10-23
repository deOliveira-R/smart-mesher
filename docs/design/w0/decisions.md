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

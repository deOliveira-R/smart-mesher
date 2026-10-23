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

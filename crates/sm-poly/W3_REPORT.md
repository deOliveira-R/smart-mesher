# W3-POLY report: `sm-poly` (tet -> polyhedral, barycentric dual)

Status: **done**. Every mandatory acceptance bullet (1-11) and both measured stretch items (S1, S2)
are implemented and covered by named tests. The Docker step of the plan (`polyDualMesh` volume
comparison in `tools/conformance.sh`) belongs to the integration agent; this VM has no Docker, and
the method is written down in `DESIGN.md`.

## Summary

* `dualize(src, &PolyOptions, &dyn EdgeResolver, builder, &mut SerialAlloc) -> (Finished, DualReport)`
  builds one dual cell per (vertex, region), one face per primal edge of a region and one polygon
  per boundary fan, classified from the model, planar in `FaceMode::Planar` (hub triangles for
  non-planar edge faces, one quad per triangle for a non-planar fan), conservative to rounding.
  Passes `validate` and `validate_geometry(default)` on every fixture and on perturbed, rotated,
  scaled and multi-volume blocks.
* Op `mesh.to_poly {mesh, keep_source?}` (output role `mesh`), query `mesh.poly_info {mesh}`,
  `PolyPlugin` (id `poly`, no layers). The op records the succession `source -> [source, new]`.
* Library extras: `EdgeResolver`, `VertexClassResolver`, `TopologyResolver`, `poly_stats`,
  `face_nonplanarity`, optional `Agglomerate` (S2, library only).
* Structure: `prep.rs` (parallel analysis), `polydata.rs` (numbering, `PolyData`, builder commit),
  `agglom.rs` (S2), `dual.rs` (entry point), `ops.rs`, `resolver.rs`, `stats.rs`, `options.rs`,
  `error.rs`, `geom.rs`. See `DESIGN.md`.

## Acceptance bullet -> test

| # | Test (file) |
|---|---|
| 1 `single_tet` | `acc1_single_tet` (acceptance.rs) |
| 2 block 3x3x3 `Keep` | `acc2_block333_keep_counts_and_volumes` |
| 3 block 3x3x3 `Planar` | `acc3_block333_planar_matches_keep` |
| 4 two volumes | `acc4_two_volumes`, `acc4_interface_area_and_no_boundary_flag_on_interface` |
| 5 warped, L-shape | `acc5_warped_and_lshape` |
| 6 classification, edge table | `acc6_face_areas_are_recovered`, `acc6_edge_table_is_classified`, `acc6_source_edge_table_is_used_first_and_edges_follow_the_source`, `acc6_topology_resolver_on_lattice_fixture`, `acc6_single_interval_curves_need_topology`; resolvers: `topology_resolver_finds_the_shared_curve`, `vertex_class_resolver_needs_a_curve_end` (resolvers.rs) |
| 7 builder differential, determinism | `acc7_builders_agree`, `acc7_determinism` (1 vs 4 threads, repeats, concurrent calls); op cold replay: `acc8_cold_replay_has_no_divergence` |
| 8 op end to end | `acc8_default_keeps_the_source`, `acc8_keep_source_false_retires_the_source`, `acc8_refusals_leave_the_state_untouched`, `acc8_poly_info_equals_poly_stats`, `registration_shape_matches_the_contract` (ops.rs) |
| 9 proptest | `prop_block_dual_is_valid_and_conserves_volume` (props.rs) |
| 10 errors | `acc10_errors_are_typed`, `alloc_is_unchanged_on_error`; extra: `bowtie_edge_is_an_open_ring_error`, `interface_not_on_a_model_face_is_refused`, `a_vertex_with_one_feature_edge_is_unsupported` (robust.rs) |
| 11 `DESIGN.md` | `DESIGN.md` (rule + diagrams, count formulas, interior-centroid omission, `keep_source`/`ambiguous-mesh`, polyDualMesh method) |
| robustness (extra) | `perturbed_lattices_stay_valid_and_conserve_volume`, `multi_volume_blocks_with_several_splits`, `rotated_scaled_and_shifted_blocks_behave_like_the_aligned_one` |
| S1 | `s1_dualize_384k_tets` (perf.rs, ignored), bench `dualize` |
| S2 | `s2_block444_merges_small_cells`, `s2_default_concavity_guard_rejects_the_hub_bulges`, `s2_rounds_zero_is_the_identity`, `s2_never_merges_across_volumes_and_keeps_the_interface`, `s2_is_deterministic_and_builders_agree`, `s2_keep_mode_and_warped_inputs_stay_valid` (agglomerate.rs) |

Definition of done, all green: `cargo build --workspace`; `cargo test -p sm-poly --all-features`
and with `RAYON_NUM_THREADS=1`; `cargo clippy -p sm-poly --all-targets --all-features -- -D warnings`;
`RUSTDOCFLAGS="-D warnings" cargo doc -p sm-poly --no-deps`; `cargo bench -p sm-poly -- --test`.

## Measured values

Counts re-derived by hand first; all agree with the spec (no spec error).

| fixture, mode | cells | edge faces | boundary polygons | faces | points | notes |
|---|---|---|---|---|---|---|
| `single_tet` | 4 | 6 | 12 | 18 | 15 | each volume 0.0416666..., max non-planarity 1.7e-16 |
| `tet_block([3,3,3])` `Keep` | 64 | 279 | 96 | 375 | 464 (162 vol + 234 face + 60 edge + 8 vertex) | 8 cells of volume 1, 24 of 0.5; area 54, 9 per model face; max non-planarity 0.0827 |
| same, `Planar` | 64 | 279 | 96 | 627 | 464 | 90 edge faces hub-split into 342 triangles (`375 - 90 + 342`); max non-planarity 7e-16 |
| `tet_block_two_volumes([4,2,2],.,2)` `Planar` | 54 | 196 | 99 | 487 | 344 | 72 hub splits, 264 triangles; 9 faces between volumes, interface area 4 |
| `tet_block_warped(amp 0.1)` `Planar` / `Keep` | 64 | 279 | 116 | 910 / 395 | 519 / 468 | curved `+z` face: every edge on it is a split edge, so one polygon per triangle; Keep max non-planarity 0.112 |
| `tet_lshape(2,1)` `Planar` / `Keep` | 63 | 262 | 108 | 626 / 370 | 462 | no `GeomSideTet` finding in either mode |

* Volume conservation: asserted to 1e-12 relative on all of the above (`Planar` and `Keep`); the
  observed differences are rounding only (e.g. 26.999999999999996 for the 3x3x3 block). On
  rotated/scaled/shifted blocks 1e-9 is asserted (scale 1e3 with shift 1e4 costs digits).
* Perturbed Kuhn blocks (`lattice::tet_kuhn(...).perturbed(seed, amp)`, amp 0.05/0.15/0.25 over 3
  seeds, `TopologyResolver`): valid, planar, volume-conserving in both modes.
* S1 (informational): `tet_block([40,40,40])` (384 000 tets, release, 4 cores, shared VM):
  `Planar` 0.46 s, `Keep` 0.38 s; 1 thread 0.99 s / 0.88 s; 68 921 cells, 529 246 / 472 606
  faces; peak RSS 377 MB with the 263 MB source resident (about +115 MB). Earlier run of the same
  test: 0.38 s / 0.97 s. The builder commit is serial and not separated (bulk commit is W3b).
* S2 on `tet_block([4,4,4])` (125 cells, volumes 0.0816 x6, 0.1667 x18, 0.2552 x2, 0.3333 x18,
  0.5 x54, 1.0 x27, median 0.5, threshold 0.375 at ratio 0.75): with `max_concavity` 0.1 (spec
  default) **0 merges**, with 0.5 0 merges, with 0.75 or more **20 merges** (125 -> 105 cells).
  The reason is measured, not assumed: the unmerged `Planar` cells already have concavity 0.14
  (median) / 0.235 (max) because hub triangles bulge (`Keep`: 0.043 / 0.084). I did not relax the
  default. Acceptance "cell count drops by exactly `report.merges`" holds for every setting
  (`cells == unmerged cells - merges`), and the volume sum, `validate`, default
  `validate_geometry`, interface faces (9 on the two-volume fixture) and `max_rounds = 0` identity
  are tested with `max_concavity = 1.0`.

## Deviations (with reasons)

1. **Branch.** Developed and pushed on the assigned `claude/w3-sm-poly-g2s3q3`; the task text
   names `claude/w3-sm-poly`, which is pushed with the same final commit.
2. **Extra items (all `#[non_exhaustive]`/additive):** `PolyError::BadClassification{face}` (a
   region-boundary face not classified on a model face); `DualReport` gained `hub_triangles`,
   `split_fans`, `merges`; `PolyOptions` gained `agglomerate` (S2); public `face_nonplanarity`.
3. **Hub numbering.** The spec calls the interior-edge hub `m_e`; for an interior edge it is a
   new vertex numbered after the primal vertices (a boundary edge's hub is its existing `m_e`).
4. **Error mapping in the op.** `NotTet`, `Dim2`, `UnsupportedFaceKind`, `OpenRing`,
   `AmbiguousEdge`, `BadClassification` -> `OpError::Invalid`; `UnsupportedFeature` ->
   `OpError::Unsupported` (the documented meaning: valid request, capability absent); builder
   failures -> `OpError::Failed`.
5. **Decode-time refusals.** An unknown field, `targets`, or a missing `mesh` is rejected by the
   registry's strict decode as `OpError::Registry(Decode{..})` before `apply` runs, not as
   `OpError::Invalid`; the no-state-change half (identical state hash) holds and is tested.
6. **`tet_block_two_volumes` with `split = 1` (or `n0 - 1`)** makes single-interval x-curves in
   one volume; `VertexClassResolver` cannot resolve those (as the plan states), and no stage-0
   fixture has a `TopologyView` for it. The multi-volume robustness test therefore uses splits
   with at least two intervals per side. `TopologyResolver` itself is unit-tested on single
   intervals (`topology_resolver_finds_the_shared_curve`).
7. **Plan copy.** The op copies the `Arc<Plan>`; the test can only assert `plan.is_some()`
   equality because a `Plan` cannot be built without the scheme crates (no dev-dep allowed).
8. **Negative-case constructions that are not stage-0 fixtures:** the bowtie mesh
   (`bowtie()` in robust.rs, two tets sharing one edge, no fixture is non-manifold). The other
   negatives are derived from fixtures through the edit API (`NonConformal` face, interface
   re-classified on a volume, fake feature edge in a source edge table). The "one non-tet cell"
   refusal uses `meshes::mixed_transition`.
9. **Dependencies.** Task list: sm-core, sm-mesh-api, sm-mesh, sm-op-api, sm-layer-api,
   sm-topo-api, rayon, thiserror, serde, schemars; ruling G2 allow-list: smallvec (hot-loop
   temporaries without heap allocation). Dev: sm-testkit, proptest, criterion, serde_json.
   `Cargo.lock` changed (new crate edges only).
10. **`.smsh` goldens (E6).** Nothing in this crate writes or reads `.smsh`; no golden or hash
    known to me moves.
11. **Forced edge class.** `finish` creates the edge table only if some edge class is set; with
    `edges = true` and no feature segment (a mesh without features) one segment between a tet
    centroid and its neighbour is classified on its volume so the table exists.
12. **Orientation assert.** The ring orientation is derived topologically and
    `debug_assert`ed geometrically (`assert_ring_orientation` in prep.rs); a globally inverted
    (negative-volume) source trips it in debug builds.

## Known gaps

* A persistent mark that the poly mesh is derived from its source (ruling M3): the op records the
  succession in the `Delta`, but `MeshEntry` has no field for it (see the ACR below).
* Seam edges (a `TopoEdge` with the same `TopoFace` on both sides) are detected only if the
  source edge table classifies them; a vertex with exactly one feature edge returns
  `UnsupportedFeature`. Non-manifold region boundaries return `OpenRing`.
* Boundary dual points sit on the faceted surface (chord error on curved bodies, W3b).
* Planar splitting raises faces per cell (627 faces vs 375 on the 3x3x3 block, ~3.4 hub triangles
  per split face) and lets two cells share several faces.
* Dual cells in `Planar` mode are mildly concave (concavity up to 0.235) because hub triangles
  bulge; they are positively oriented and pass `validate_geometry`. The agglomeration guard's
  default (0.1) is below that, see S2.
* Serial builder commit dominates large inputs; bulk commit is W3b.
* Not done: S3 (op-exposed `faces: "keep"`, `merge_angle_deg`), partial dualization of a mixed
  mesh, circumcentric dual, vertex moving planarization (all deferred by the plan).
* The `polyDualMesh` comparison needs Docker/OpenFOAM and the OpenFOAM exporter: integration.

## api_change_requests

1. **crate `sm-layer-api`** (`MeshEntry`): add `pub derived_from: Option<EntityId>` (default
   `None`, included in the `MeshLayer` digest under a bumped `DIGEST_VERSION`, cleared when the
   referenced mesh is retired), plus `MeshEntry::with_derived_from(id)`. *Why:* ruling M3 asks
   for a machine-readable, persistent mark that the poly mesh is derived from its source so the
   `mesh.run` neighbour guard and exports can treat it. Today only `Delta::successors` carries
   it, and that is not stored in the state. *Proposed signature:*
   `pub fn with_derived_from(mut self, source: EntityId) -> Self`. sm-poly would set it in
   `ToPoly::apply` (one line) once it exists.

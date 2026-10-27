# W3-POLY report: `sm-poly` (tet -> polyhedral, barycentric dual)

Status: **done.** The former blocked case (dual cells at a reflex interface edge failing
`validate_geometry`, `GeomSideTet`) is fixed by splitting such cells into wedges (see Polish).
Every mandatory acceptance bullet (1-11) and both measured stretch items (S1, S2) are implemented
and covered by named tests. The Docker step of the plan (`polyDualMesh` volume comparison in
`tools/conformance.sh`) runs in the integration harness: Docker
(`opencfd/openfoam-default:2506`) is available to the orchestrator, and the step needs the
OpenFOAM exporter, which this crate does not depend on. The method is in `DESIGN.md`.

## Summary

* `dualize(src, &PolyOptions, &dyn EdgeResolver, builder, &mut SerialAlloc) -> (Finished, DualReport)`
  builds one dual cell per (vertex, region), one face per primal edge of a region and one polygon
  per boundary fan, classified from the model, planar in `FaceMode::Planar` (hub triangles for
  non-planar edge faces, one quad per triangle for a non-planar fan), conservative to rounding.
  Passes `validate` and `validate_geometry(default)` on every fixture and on perturbed, rotated,
  scaled and multi-volume blocks, including reflex interface edges, inclusions and concave
  corners (cells that are not star-shaped about their centroid are split into wedges).
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
| 6 classification, edge table (L-shape: dual edges on the reflex `TopoEdge` checked by entity) | `acc6_face_areas_are_recovered`, `acc6_edge_table_is_classified`, `acc6_source_edge_table_is_used_first_and_edges_follow_the_source`, `acc6_topology_resolver_on_lattice_fixture`, `acc6_single_interval_curves_need_topology`; resolvers: `topology_resolver_finds_the_shared_curve`, `vertex_class_resolver_needs_a_curve_end` (resolvers.rs) |
| 7 builder differential, determinism | `acc7_builders_agree`, `acc7_determinism` (1 vs 4 threads, repeats, concurrent calls); op cold replay: `acc8_cold_replay_has_no_divergence` |
| 8 op end to end | `acc8_default_keeps_the_source`, `acc8_keep_source_false_retires_the_source`, `acc8_refusals_leave_the_state_untouched`, `acc8_poly_info_equals_poly_stats`, `registration_shape_matches_the_contract` (ops.rs) |
| 9 proptest | `prop_block_dual_is_valid_and_conserves_volume` (props.rs) |
| 10 errors | `acc10_errors_are_typed`, `alloc_is_unchanged_on_error`; extra: `bowtie_edge_is_an_open_ring_error`, `interface_not_on_a_model_face_is_refused`, `a_vertex_with_one_feature_edge_is_unsupported` (robust.rs) |
| 11 `DESIGN.md` | `DESIGN.md` (rule + diagrams, count formulas, interior-centroid omission, `keep_source`/`ambiguous-mesh`, polyDualMesh method) |
| reflex edges (polish) | `reflex_interface_edges_are_split_into_valid_wedges`, `perturbed_reflex_notches_are_valid`, `split_cells_are_deterministic`, `checkerboard_edge_is_an_open_ring_error` (robust.rs) |
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
| `tet_lshape(2,1)` `Planar` / `Keep` | 63 | 262 | 108 | 626 / 370 | 462 | no `GeomSideTet` finding in either mode, no split (favourable notch orientation; unfavourable ones are split, see Polish) |

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
   equality because a `Plan` cannot be built without the scheme crates (no dev-dep allowed); the test says so in a comment.
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

* Reflex interface edges: resolved in Polish (wedge split). Residual risk only for a sliver tet
  at a split vertex (wedges are accepted with a 1e-9 relative margin) or more than 8 repair
  rounds; neither observed.

* A persistent mark that the poly mesh is derived from its source (ruling M3): the op records the
  succession in the `Delta`, but `MeshEntry` has no field for it (see the ACR below).
* Seam edges (a `TopoEdge` with the same `TopoFace` on both sides) are detected only if the
  source edge table classifies them; a vertex with exactly one feature edge returns
  `UnsupportedFeature`. A region that is not edge-manifold returns `OpenRing`: its tets around
  an edge form more than one chain, as in a checkerboard where two volumes alternate around an
  edge (`checkerboard_edge_is_an_open_ring_error`) or two tets of one volume touching only along
  an edge (`bowtie_edge_is_an_open_ring_error`).
* Boundary dual points sit on the faceted surface (chord error on curved bodies, W3b).
* Planar splitting raises faces per cell (627 faces vs 375 on the 3x3x3 block, ~3.4 hub triangles
  per split face) and lets two cells share several faces.
* Dual cells in `Planar` mode are mildly concave (concavity up to 0.235) because hub triangles
  bulge; they are star-shaped about their centroid (checked, split otherwise) and pass
  `validate_geometry`. The agglomeration guard's default (0.1) is below that, see S2.
* Serial builder commit dominates large inputs; bulk commit is W3b.
* Not done: S3 (op-exposed `faces: "keep"`, `merge_angle_deg`), partial dualization of a mixed
  mesh, circumcentric dual, vertex moving planarization (all deferred by the plan).
* The `polyDualMesh` comparison needs the OpenFOAM exporter and runs in the integration harness
  (Docker is available to the orchestrator); not run from this crate.

## api_change_requests

1. **crate `sm-layer-api`** (`MeshEntry`): add `pub derived_from: Option<EntityId>` (default
   `None`, included in the `MeshLayer` digest under a bumped `DIGEST_VERSION`, cleared when the
   referenced mesh is retired), plus `MeshEntry::with_derived_from(id)`. *Why:* ruling M3 asks
   for a machine-readable, persistent mark that the poly mesh is derived from its source so the
   `mesh.run` neighbour guard and exports can treat it. Today only `Delta::successors` carries
   it, and that is not stored in the state. *Proposed signature:*
   `pub fn with_derived_from(mut self, source: EntityId) -> Self`. sm-poly would set it in
   `ToPoly::apply` (one line) once it exists.

## What changed in review (round 0)

* MUST: the unreported `GeomSideTet` at reflex interface edges is now reported (Known gaps, status
  line, summary, `DESIGN.md` positioning item 4 corrected and a "Risks" entry) and pinned by a
  regression test; no algorithm change. Option (c) (op-level check) declined, see Known gaps.
* Acceptance 6: the L-shape test now checks that feature-curve dual edges sit on the expected
  `TopoEdge` (endpoint class) and that some are on the reflex `TopoEdge`.
* `Prep::build` reports the actual offending cell in `NotTet` when a cell does not have 4 corners.
* `tests/ops.rs`: `is_invalid` matches `OpError::Invalid` (through `ApplyError::Op`) instead of a
  Debug substring; the decode-time refusals additionally assert they are not `Invalid`.
* Acceptance 8: decode-time refusals (deviation 5) and plan-copy check (deviation 7) documented as
  accepted deviations; the test says why only `plan.is_some()` is compared.
* Observation for the `Keep` inclusion added next to the reflex-edge gap.

## Polish

### Reflex-interface-edge gap fixed (orchestrator priority)

* **Root cause.** `validate_geometry` needs every cell star-shaped about its volume centroid. A
  dual cell at a reflex edge of its region spans more than 180 degrees, so its kernel is the
  wedge between the two face planes. In the Kuhn lattice the corner cubes at a vertex are unequal
  (the cube whose diagonal starts at the vertex gives all 6 tets, the others fewer), and when that
  heavy cube sits next to the missing quadrant the centroid leaves the kernel: cell 12 of the
  `{x<2,y>2}` notch has its centroid at y = 2.12 > 2, on the wrong side of the interface kite in
  y = 2. The cell itself was right (closed, planar, conservative); its shape is the problem, so no
  tolerance or winding change can fix it. Perturbed meshes fail at any notch orientation.
* **Fix (`concave.rs`, `dual.rs`, `polydata.rs`).** After the dual is built every cell is checked
  with the validator's rule (with a 1e-12 V rounding margin). A failing cell `(v, R)` is rebuilt from the corners of its
  tets (`{lambda_v >= lambda_w}` of each tet: convex hexahedra with planar quads); its edge faces
  are refined to one kite `(m_e, fc, c_t, fc')` per ring tet so the corners tile it; corners are
  grouped greedily into wedges that stay edge-manifold and star-shaped about their own centroid
  (margin 1e-9 V); each wedge is an output cell and the quads `(v, m, fc_f, m)` between wedges are
  new interior faces. Boundary and interface polygons at a split vertex become one quad per
  triangle in every region, so interface faces stay shared. Edge faces of neighbouring cells pass
  through the centroid of every *bent* triangle (an interior triangle at a split vertex) so all
  cells stay closed; neighbours are re-checked and split in a later round if needed (at most 8,
  observed 1-2). Faces only subdivide space: the volume is unchanged. Inputs with no failing cell
  produce the same output as before (all stage-0 count tests unchanged).
* **Report.** `DualReport` gained `split_cells` and `wedges` (additive, `#[non_exhaustive]`).
  `cells = (vertex, region) pairs + wedges - split_cells - merges`. **Integration note:** the
  milestone row 11 relation "polycase cells = sum over volumes of the vertices touching it" holds
  only when `split_cells == 0`; with a reflex interface (V1|V2 at x = y = 10) assert
  `cells == pairs + wedges - split_cells` instead.
* **Results** (`tet_block([4,4,4])`, `validate` and `validate_geometry(default)` clean, volume to
  1e-12, both builders agree; `reflex_interface_edges_are_split_into_valid_wedges`):

  | case | `Planar` split -> wedges | `Keep` split -> wedges |
  |---|---|---|
  | notches `{x>2,y>2}`, `{x<2,y<2}`, `{x<2,z<2}`, `{x>2,z>2}` | 0 | 0 |
  | notches `{x<2,y>2}`, `{x>2,y<2}`, `{y<2,z>2}` (formerly cells 12/96, 8/92, 60/64) | 2 -> 4 | 5 -> 7 |
  | 3-concave corner `{x<2,y<2,z>1}` | 2 -> 4 | 5 -> 7 |
  | single-tet inclusion | 4 -> 4 (refined only) | 4 -> 4 |
  | cube inclusion `[1,3]^3` (formerly 12 `GeomSideTet` in `Keep`) | 0 | 12 -> 12 |
  | octant inclusions at `(2,2,2)` (3 reflex edges meet) | 0 or 2 -> 4 | 0 or 5 -> 7 |

  Perturbed notches (free coordinates moved by up to 0.3, 4 seeds, 4 notches,
  `perturbed_reflex_notches_are_valid`): 28 cells split, all valid. Determinism of the split
  (1 vs 4 threads, repeats): `split_cells_are_deterministic`.
* **Cost.** One extra parallel check over all cells per dualization; S1 (384k tets, release):
  the check took 0.06-0.14 s against 1.2-1.8 s for the `PolyData` build
  and 4-6 s end to end, measured on a machine with load average ~850 on 10 cores (absolute
  times are not comparable with the S1 figures above; the check is a few percent). No cell is
  split on the lattice, so the output is unchanged.
* **Not run:** `polyDualMesh`/`checkMesh` comparisons. Not useful from this crate: the plan keeps
  checkMesh out of unit tests (our `validate` judges meshes), and the OpenFOAM writer is an
  integration dependency; the integration harness (Docker available to the orchestrator) owns it.

### Review should-fix items

1. Widen the reflex-edge known-gap text: **declined as obsolete**: the gap is fixed, and the
   single-tet inclusion and 3-concave corner are now asserted valid instead of described.
2. Edge-manifold (checkerboard) refusal class: named in `DESIGN.md` (op refusals), the
   `PolyError::OpenRing` docs and Known gaps above; pinned by
   `checkerboard_edge_is_an_open_ring_error`.
3. Docker wording: the status line and Known gaps now say Docker is available to the
   orchestrator and the comparison runs in the integration harness.

The pinned known-gap tests `known_gap_reflex_interface_edge_gives_side_tet_violations` and
`known_gap_keep_mode_enclosed_inclusion` are replaced by the validity tests above.

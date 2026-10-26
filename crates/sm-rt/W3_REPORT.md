# W3-RT report: `sm-rt`

Branch: `claude/w3-sm-rt-08sugi` (the task text says `claude/w3-sm-rt`; the session harness fixed the
branch name, so everything was pushed there). Last commit subject: `w3 sm-rt: done`.

## Summary

`sm-rt` implements the frozen `sm-rt-api` contract for `VOLUME_MESH | EXACT_PLANAR`:

* `MeshTrackerBuilder` (`TrackerBuilder`: `builder_version`, `cache_key`, `build`, `load`) accepting
  `BuildInput::Mesh` and `BuildInput::Facets`; `Brep` and `World` are `RtError::Unsupported` naming the
  variant. `build_concrete` returns the concrete `MeshTracker` (needed for `nudge_count()`).
* `MeshTracker`: flat SoA/CSR structure, Cyrus-Beck walk for convex cells, exact fan-triangle
  crossing test (`orient3d` + SoS) for the rest, face-BVH walk for large facet cells, cell-BVH
  `locate`, exact `closest`, decks applied in `step`, nudge recovery, rayon `trace_bundle`.
* Facet input: grouped, located watertightness report (`check_facets`), one polyhedral cell per region.
* `.smrt` cache: `export_cache`, `load`, `load_image`, `write_smrt`, `write_smrt_file`, `read_smrt`.
* `RtPlugin` (`rt.info`, `rt.check`, `rt.locate`, ruling R2).
* `examples/trace_rays.rs` (smfacets/smrays bench side) and `examples/bench_rt.rs` (acceptance 14 numbers).
* `DESIGN.md` (positioning, layout, rules, tolerance policy, numbers).

Release-mode run of the whole suite (`cargo test --release -p sm-rt`) and the debug suite with `RAYON_NUM_THREADS=1` also passed on the final tree. Gates run on the final tree: `cargo build --workspace`; `cargo test -p sm-rt --all-features`;
`RAYON_NUM_THREADS=1 cargo test -p sm-rt --all-features`; `cargo clippy -p sm-rt --all-targets
--all-features -- -D warnings`; `RUSTDOCFLAGS="-D warnings" cargo doc -p sm-rt --no-deps`; the
`#[ignore]`d 1e7-history audit in release.

## Acceptance -> test

Debug builds run a tenth of the 1e6 lines (1e5) so `cargo test` stays fast; `cargo test --release`
runs the full 1e6 and all the numbers below are from the release run.

| # | test (file) | measured |
|---|---|---|
| 1 | `acc01_hex_block_iur_chords` (chords.rs) | 1e6 lines, 0 lost, max chord-sum error 6.3e-13, max per-cell \|z\| 3.83 over 1000 cells (all < 4; with 1000 cells a 4-sigma cell is expected in about 6 % of seeds, so this bound is tight by construction) |
| 2 | `acc02_{truncated_octahedron,hexagonal_prism,mixed_transition,dual_patch}_chords` | 1e6 lines each, 0 lost, errors <= 1.4e-13, max \|z\| 0.47 / 0.88 / 1.52 / 1.52; hull chords from independent per-cell plane clipping, runs merged for the non-convex union |
| 3 | `acc03_nonconvex_two_chords_through_the_notch`, `acc03_nonconvex_iur_expected_chord` | two L chords of sqrt(2)/2 within 1e-12; 1e6 lines, L-cell and brick means within 4 sigma (max \|z\| 0.08), 0 lost |
| 4 | `acc04_shared_face_consistency_forward_and_reverse`, `acc04_edge_exits_pick_the_lowest_face_and_never_get_lost`, `acc04_vertex_exits_pick_the_lowest_face_and_never_get_lost` | forward and reversed traces cross identical face lists (4 meshes, 25 000 lines each in release, 47 470 crossings on hex_block(4) alone in debug); exits through exact edges/vertices pick the lowest of the 2/3 tied faces; lines within 1e-9 of every edge and vertex: 0 lost, chord sums exact to 1e-12 |
| 5 | `acc05_cauchy_icosphere_shell`, `acc05_cauchy_hex_block_and_truncated_octahedron` | 1e6 lines: hex_block(10) 1.05 sigma (637 128 hits), truncated octahedron 2.50 sigma (354 541 hits), icosphere(4) shell 0.29 sigma (332 980 hits), all below 4 |
| 6 | `acc06_*` (decks.rs) | mirror reflects (-dx, dy, dz) bit-exactly; translation wrap within 1e-12 and `entered_via == mate`; quarter-turn wrap within 1e-12 / 1e-15; White, Vacuum, Albedo, Reflective return `Boundary{Respond}` with the state on the face; closed reflective box 1e4 histories x 1e3 steps (release), 0 lost, 0 nudges, all inside |
| 7 | `acc07_*` (facets.rs) | tetrahedron, icosphere(3), nested pair build; removing one triangle gives exactly one `facet.open_edge` with `gap` = longest edge within 1e-12 and `location` on the triangle; flipped triangle gives `facet.inconsistent_orientation`; hostile soups never panic; `locate` agrees with an independent winding number on 1e5 points (release; 2e4 debug), nested shells right |
| 8 | `acc08_*` (locate.rs) | 1e5 random points in hex_block(10) and mixed_transition equal brute force (with no hint, a wrong hint, the right hint); points at 1e-3/1e-9/0 from faces, edges, vertices are exact or flagged `on_face`; directionless locate on a face identical across calls, hints and 1/N threads |
| 9 | `acc09_lost_particle_audit_debug_size` (2e5 histories x 60 steps), `acc09_lost_particle_audit_full` (`#[ignore]`, 1e7 histories x 100 steps), `acc09_reflex_edge_stress` | full run (release): 1e7 histories, 1e9 steps, 0 lost, 0 sense mismatch (no `Lost` at all) under `RecoveryPolicy::Fail`; with `Nudge{3}` (2e5 histories x 100 steps) 0 lost and `nudge_count() == 0`; models: hex_block(6) with White, nonconvex_poly, mixed_transition, nested icospheres (Reflective), hex_block with mirror decks. The first full run found the bug listed below; the numbers here are from the rerun on the final logic. Debug run: 2e5 histories x 60 steps = 1.2e7 steps, 0 lost |
| 10 | `acc10_*` (cache.rs) | round trip via `write_smrt`/`read_smrt`/`load`: equal `structure_hash`, equal `TrackerInfo`, bit-identical 1e5-ray bundle; one flipped byte in any section is `CacheCorrupt`; more than 500 hash-consistent single-byte tampers never panic and are `CacheCorrupt` or a no-op; changed coordinate/boundary/option are `CacheStale` with the right reason; key unchanged for 1 and 3 threads |
| 11 | `acc11_build_cache_and_bundle_are_deterministic` | `sm_testkit::determinism::check` over cache bytes and bundle hashes (mesh and facets), 1 vs 4 threads, repeats and concurrent runs |
| 12 | `acc12_*` (errors.rs) | `Unsupported` (Brep, World), `Build`, `BadDirection`, `NonFinite`, `UnknownId`, `CacheStale`, `CacheCorrupt`, plus `InvalidInput` and `BadAlbedo` through the provided methods |
| 13 | `acc13_warm_scratch_means_zero_allocations` (alloc.rs) | 0 allocations in `step`, `locate`, `closest`, `distance_to_boundary` after warm-up on a hex mesh (3668 events), the non-convex mesh (379) and nested facet shells (1292) |
| 14 | `examples/bench_rt.rs` | DESIGN.md table (record only) |
| R2 | `r2_rt_queries_*` (plugin.rs) | `rt.info`, `rt.check`, `rt.locate` in a session with `CoreLayers`; derived-cache hit gives the same answer |

Other tests: `scope6_bundle_override_is_bit_identical_to_the_default_sequential_walk`,
`scope6_bin_maps_cells_regions_and_mesh_sets`, `scope8_region_boundaries_surfaces_fire_peek_and_dagmc_shims`,
`scope3_lost_particles_are_diagnosed_and_nudge_recovery_is_counted`,
`scope5_complement_policy_region_makes_the_notch_a_complement_cell`.

## Deviations from the task text (frozen contract and rulings win)

1. **Branch name** (above).
2. **`StepEnd`**: the frozen enum has one `Boundary{surface, face, action}` event instead of `Deck` and
   `DomainBoundary`; `Reflect` and `Wrap` are applied in `step`, `Respond` is left to `resolve_boundary`.
3. **No `LostCause::SenseMismatch`, no `LostInfo.retries`**: a sense mismatch is reported as
   `LostCause::NoExitFace`, a failed nudge as `RecoveryFailed`; the nudge count is an `AtomicU64` on the
   concrete `MeshTracker` (`nudge_count()`), never read by a query (api_change_request 1).
4. **`FacetInput::check_watertight` is provided by the contract** (one issue per bad edge); it is tested
   and `check_facets` is a grouped diagnostic on top of the same scan, because acceptance 7 wants one
   group for one missing triangle.
5. **`load` takes the input** (frozen signature) to compute the key; `load_image` loads without the mesh.
6. **Caps**: `VOLUME_MESH | EXACT_PLANAR` for mesh and facet input; `PERIODIC` stays unset although matched
   `Wrap` decks are applied in `step` (the task fixes the caps; see DESIGN.md).
7. **`mesh()` is `None`**: a `Box<dyn Tracker>` is `'static` and the input mesh is borrowed (api_change_request 2).
8. **Partition-classified cells**: `RegionRule::ByVolumeClassification` keys on the classified entity as
   given; it cannot map a `PartitionCell` to its parent volume without a topology view
   (`MeshInput` carries none), so each partition cell is its own region.
9. **Non-planar faces** are tracked through their fan and reported as non-fatal `mesh.nonplanar_face`
   issues instead of failing the build; refined meshes (inactive cells) and non-conformal faces are refused.
10. **Rotation deck** is built inline with `Deck::rotation` and `BoundarySpec::add_model_face`, not taken
    from a model fixture (none has a quarter-turn pair on the mesh boundary).
11. **Insertion-order bullet of acceptance 10**: the key is a function of the mesh's own
    `topology_hash`/`geometry_hash`, so it is exactly as canonical as the mesh (the mesh canonicalises face
    order and loop starts; the cell index is part of its identity). `acc10_key_follows_the_mesh_identity_hashes`
    asserts a rebuild gives the same key and a cell swap changes it. **This rewording of the plan's
    "key does not change with cell insertion order" needs explicit orchestrator acceptance (review round 0).**
12. **`trace_bundle` is overridden** (rayon over fixed blocks, appended in order); a test proves it
    bit-identical to the default sequential walk. Measured (`bench_rt 20`, icosphere(4) shell, 1e6 rays,
    10-core Apple machine): 42.5k rays/s with 1 thread vs 97.2k rays/s with 4 threads (2.3x), so the
    override earns its place. `bin_map` is overridden only to add `MeshSets`; `Cells`/`Regions` equal the default.
13. **Tolerance rules** that the task did not fix (probe offsets, the 4-tol entry window, "outside on a
    leaving direction") are in DESIGN.md.
14. **`crc32fast` not used** (the contract's `hash64` covers integrity); extra dependencies: `smallvec`
    (the contract's `TrackState` type), `schemars`, `serde`, `serde_json` (plugin queries; allow-list).
15. **`Cargo.lock`** gained the entries of sm-rt's new dependencies (cargo wrote it; no other manifest was touched). `#![forbid(unsafe_code)]` is set in the library.
16. **Test allocator**: `tests/alloc.rs` contains the counting `GlobalAlloc` (the only `unsafe`, each method
    with a `// SAFETY:` comment, same pattern as `sm-rt-api/tests/alloc.rs`); the library has no `unsafe`.
17. **Stretch** items S2-S5 and `rt.fire` were not done. S2 (`Brep` with `SurfaceRefine::None`) would
    contradict the mandatory "`Brep` is `Unsupported`" test; it is a few lines in `builder.rs` if wanted.

## Review round 0 (what changed)

* **Must fix, complement exit.** A one-cell face that no `BoundarySpec` rule claims now crosses into the
  implicit complement when the tracker has one (facet input always; mesh input with
  `ComplementPolicy::Region`): `StepEnd::RegionBoundary{to_cell: COMPLEMENT, to_region: complement}`.
  The face then has no action (`f_action == NONE`); with no complement it keeps `spec.default`. Explicit
  rules (Reflect, Wrap, Respond) still give `Boundary`. Where the complement leaves the vertex box the
  step ends `RegionBoundary{to_cell: NONE}` (`LeftModel`, i.e. vacuum); `spec.default` is not applied
  there (an orchestrator decision if Reflect or Wrap should apply at the box). Tests: `scope5` updated,
  new `scope4_a_ray_leaves_a_body_into_the_notch_complement` and
  `scope4_disjoint_bodies_are_connected_through_the_complement` (two tets, 0.7 + 2.2 + 0.8 chords,
  explicit Reflect still claims its face); `acc07_nested_shells...` now ends in the complement.
  DESIGN.md section Complement rewritten. `examples/trace_rays.rs` already continues on
  `RegionBoundary` to a cell, so its crossing counts now include the complement hops (as DAGMC's do).
* **Should fix 1.** `assign_actions` interns actions in a `BTreeMap` keyed by exact bits: O(n log n).
* **Should fix 2.** Acceptance 10 wording: kept as in item 11 above, flagged for explicit acceptance.
* **Should fix 3.** `trace_bundle` speedup measured and recorded (item 12). Kept.
* **Should fix 4.** `acc13` now warms up on two seeds and measures a third, unseen set of rays; still zero
  allocations. Exactly degenerate crossings still allocate inside `sm_math::orient3d_sos` (known gap).
* **Should fix 5.** Debug runs keep the reduced line counts (gate does not run release). The full counts
  were run once with `cargo test --release -p sm-rt --all-features` on this commit: all pass.
* **Should fix 6.** New plugin test `r2_rt_check_reports_an_invalid_boundary_as_a_located_issue`: a
  `wall` response record bound to albedo 2.0 gives `ok == false` with issue `boundary.bad_albedo`.
  To make that possible `rt.check` now reports an invalid boundary record as a finding instead of
  failing the query.

## Bugs found by the audits (fixed, now covered by tests)

* The first 1e7-history audit found 2 lost particles in 1e9 steps, both at the re-entrant edge of
  `nonconvex_poly`: entering crossings inside the start tolerance were treated as a sense error and a
  point in the notch air within `tol` of the wall was located in the body. Fixed by the entry window and
  the leaving rank rule (DESIGN.md), covered by `acc09_reflex_edge_stress` (starts within 3e-7 of the
  edge, 1e6 starts in release) and by the rerun of the full audit.
* Excluding the entry face by identity was wrong when `locate` put a point on a face the direction
  leaves; now only grazing or entering crossings of the entry face are excluded.

## Known gaps

* ANALYTIC, HIERARCHY, non-matching PERIODIC, `.smsh` chunk, mmap, SIMD, `F32Traversal` (W3b or stretch).
* `rt.fire`; overlap detection; curved `closest`.
* The large-mesh step rate is cache-miss bound (DESIGN.md); no layout work beyond CSR/SoA was done.
* Exactly degenerate crossings (a line through a fan edge) allocate inside `sm_math::orient3d_sos`; the
  allocation test uses generic rays.
* Orchestrator step: the Docker conformance and DAGMC comparison are not part of this crate; the
  `trace_rays` example is ready for the `sm-rt-capi` harness (formats as in the task text; the
  `bench/README.md` it names is not on main, so the formats were taken from the task text).

## api_change_requests

1. `sm-rt-api::LostCause` / `LostInfo`: add `LostCause::SenseMismatch` and `LostInfo.retries: u32`
   (both `#[non_exhaustive]`, additive). Why: acceptance 9 names them and C-ABI diagnostics want the
   retry count. Proposed: `SenseMismatch` ("first crossing is entering and no exit within 4 tol");
   `pub retries: u32` set by `Nudge` recovery.
2. `sm-rt-api::MeshInput`: carry `Option<Arc<dyn MeshRead>>` (or a `mesh_arc` constructor) so a built
   tracker can return `Tracker::mesh() -> Some`. Why: the `VOLUME_MESH` capability doc promises it; the
   tracker cannot borrow the input and a loaded cache has no mesh.
3. (optional) `sm-rt-api::TrackerCaps`: define `PERIODIC` as "applies matched `Wrap` decks" so the bit
   can be set for conforming periodic pairs.

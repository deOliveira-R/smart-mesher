# W3 report: sm-smooth

Status: **done**. Mandatory acceptance 1-9 and both stretch items (Condition objective, 2-D
Winslow) are implemented and tested. One number misses its target and is recorded, not gated:
the Laplacian throughput (1.26 M node-updates/s against the 5 M target, see "Measured values").

Branch: the session assigned `claude/w3-sm-smooth-ndof2z`, so the work is pushed there instead
of `claude/w3-sm-smooth` (the session rules forbid pushing another branch name).

## Summary

- Library: `smooth`, `apply_outcome`, `SmoothInput`, `SmoothRequest`, `NodeSelect`, `Method`
  (`Laplacian`, `SmartLaplacian`, `CentroidArea`, `Optimize{MeanRatio|Condition, p}`,
  `Winslow`), `Schedule`, `SmoothOutcome`, `SmoothReport`, `QualityStats`, `SmoothError`.
- Plugin: `SmoothPlugin` (id `"smooth"`) registers op `mesh.smooth` (NO_OUTPUTS, writes only the
  `meshes` layer through `replace_mesh`) and query `mesh.smooth_preview`; no layer, no export.
- Classification constraints, pins, guard, colouring, repair loop and the update rules are
  specified in `DESIGN.md`.
- No `unsafe`, no `unwrap` / `expect` in `src/`, no `HashMap`, no randomness. `README.md` has the
  deferred list; `DESIGN.md` the formulas, measured values and bench table.

## Acceptance -> test

| # | Bullet | Test (file) |
|---|---|---|
| 1 | Recovery 3-D, both schedules, vertices bit-identical | `a1_laplacian_restores_the_perturbed_hex_lattice` (`recovery.rs`) |
| 2 | Recovery 2-D, Laplacian and CentroidArea, both schedules | `a2_laplacian_and_centroid_area_restore_the_quad_lattice` (`recovery.rs`) |
| 3 | Optimize reaches Shape 0.99; `sum beta^p` non-increasing; tet lattice | `a3_optimize_reaches_shape_099_on_the_jittered_hex_lattice`, `a3_colored_objective_never_increases`, `a3_tet_lattice_keeps_the_unperturbed_mean_shape` (`optimize.rs`) |
| 4 | Never inverts matrix (640 runs) | `a4_every_method_keeps_a_jittered_quad_lattice_valid` (`never_inverts.rs`) |
| 4 | L-shape: unguarded Laplacian inverts, ours does not | `a4_the_guard_is_exercised_on_an_l_shaped_region` |
| 4 | Inverted input: SJ non-decreasing, `n_inverted` non-increasing, `smooth.inverted_input` | `a4_an_inverted_input_never_gets_worse`; diagnostic in `a7_diagnostics_name_the_problem` (`op.rs`) |
| 5 | Stays on the cylinder (face, circles, seam, vertices, Param re-evaluation, t in range) | `a5_nodes_stay_on_the_cylinder_for_every_method` (`geometry.rs`) |
| 5 | Equal spacing within 1e-6 | `a5_laplacian_and_centroid_restore_equal_spacing` |
| 5 | Trim-reject path (`rejected_geometry += 1`, node unmoved) | `a5_a_projection_outside_the_trimmed_face_is_rejected`; curve range: `a_move_beyond_the_end_of_an_edge_is_rejected` (`unsupported.rs`) |
| 6 | Mirror deck slides in the plane | `a6_mirror_deck_nodes_slide_in_the_plane` (`decks.rs`) |
| 6 | Translation deck pair pinned with its edges, `pinned_deck` names the faces | `a6_translation_deck_pair_is_pinned_with_its_edges`; op diagnostic in `a7_diagnostics_name_the_problem` |
| 6 | Explicit interface face pinned | `a6_an_explicit_interface_face_is_pinned` |
| 7 | 10^3 brick: lattice restored, volume 1000, `Delta.created` empty, `touched_layers == ["meshes"]`, `smooth.report`, undo / redo / cold replay, `validate` / `validate_geometry` before and after, preview equals op report with unchanged hash | `a7_smoothing_a_jittered_brick_restores_the_lattice` (`op.rs`) |
| 7 | `entities:[face]` moves only that face's nodes | `a7_entities_restrict_the_moved_nodes` |
| 7 | Unknown mesh, node-less `TopoFace`, bad params, high-order mesh fail with the hash unchanged | `a7_failures_leave_the_state_hash_unchanged` |
| 7 | A face shared with an unselected volume stays conformal (`fix_boundary: false`, listed volume): cap, cap edges and the other volume are bit-identical; the listed volume's outer faces move | `a7_a_face_shared_with_an_unselected_volume_stays_conformal` (a mutation of the closure rule to "any listed volume" makes it fail) |
| 7 | Dim-2 listed face moves its edges with `fix_boundary: false` (deviation 2 and 3) | `a_listed_face_of_a_surface_mesh_moves_its_edges_with_fix_boundary_false` (`unsupported.rs`) |
| 7 | `NothingToSmooth` fails with the hash unchanged | `a7_nothing_to_smooth_fails_without_a_change` |
| 7 | Params round-trip through the schema | `a7_params_round_trip_through_the_schema` |
| 8 | `check_f64_bits(3 runs, 4 threads)` on `points`, every method x schedule x fixture; default pool; exact `geometry_hash` and state hash for 1, 2, 4 threads | `a8_points_are_bit_identical_for_every_method_and_schedule`, `a8_geometry_hash_and_state_hash_do_not_depend_on_threads` (`determinism.rs`); the whole suite also passes with `RAYON_NUM_THREADS=1` |
| 9 | Bench | `benches/smooth.rs` (numbers below, `DESIGN.md`) |
| S1 | `Objective::Condition`, max Condition <= 1.01; never-inverts and determinism coverage | `a3_condition_objective_reaches_max_condition_101`; `condition` rows of the matrix in `never_inverts.rs`, `determinism.rs`, `geometry.rs` |
| S2 | Winslow recovery 1e-9 h on the lattice and its shear; matrix and determinism | `winslow_restores_the_lattice_and_is_affine_invariant`, `winslow_is_two_dimensional_only` (`winslow.rs`); `winslow` rows of the matrix, determinism and geometry tests |
| - | High-order record is `Unsupported`; polyhedral nodes frozen | `a_high_order_record_is_unsupported`, `a_polyhedral_cell_freezes_its_nodes` (`unsupported.rs`) |

## Measured values

All release builds, this VM (4 cores). Tolerances were not relaxed.

- A1 (hex 4x4x4, jitter 0.25 h, seed 7, `tolerance` 1e-12): Colored 45 sweeps, deviation 2.8e-13;
  Jacobi 92 sweeps, 2.0e-13 (limit 150 sweeps, 1e-9 h = 2.5e-10).
- A2 (quad 6x6, jitter 0.25 h, seed 11): Laplacian Colored 92 sweeps 2.9e-12 h, Jacobi 198
  sweeps 1.0e-12 h (limit 300, 1e-9 h); CentroidArea Colored 84 sweeps 2.7e-12 h, Jacobi 180
  sweeps 8.2e-13 h (limit 400, 1e-6 h).
- A3 (hex 6x6x6, jitter 0.2 h, seed 3): initial min Shape 0.6448; Optimize p=4 Colored ends at
  0.99999827 (mean 0.99999964) after 51 sweeps, no rejections. Condition objective: max
  Condition 1.719 -> 1.0000012 after 47 sweeps. `sum beta^p` non-increasing over 40 single
  sweeps. Kuhn tets 4x4x4, jitter 0.2 h, seed 5: unperturbed mean Shape 0.75595, perturbed
  0.72089, SmartLaplacian 0.76420 (32 sweeps), Optimize 0.88847 (18 sweeps).
- A4: at jitter 0.45 h only seed 4 of seeds 0..29 keeps the 12x12 lattice valid (per-axis
  displacement is `amp * U[-1,1)`); the test scans for the first valid seed (4, initial min
  ScaledJacobian 0.0608). 640 runs all valid, min SJ >= min(initial, 1e-3). L region: an 8x8-cell
  L, reflex corner moved by (-0.5, -0.5), interior jitter 0.3 h seed 19: one unguarded Jacobi
  sweep inverts 1 cell; the guarded matrix (16 method variants x 2 schedules x 4 iteration
  counts) refused 5228 moves and stayed valid.
- A5 (tube 12x6, r=1, h=2, jitter 0.2 h): all geometry checks within 1e-12; 400 sweeps restore
  angular spacing to 4.3e-14 (Laplacian) / 3.3e-13 (CentroidArea) rad and axial spacing to
  4e-16 / 1e-14.
- A7: 10^3 brick, volume-node jitter 0.1 h, 100 Colored sweeps (`tolerance` 0): deviation 9.0e-7
  (< 1e-6). With jitter 0.2 h the same run gives 1.8e-6: Colored Gauss-Seidel converges like
  `cos^2(pi/10)^k`, so the test uses 0.1 h.
- Winslow (quad 6x6 of size 6, interior jitter 0.25 h, shear x' = x + 0.15 y): Colored 87 sweeps
  2.3e-12 h (sheared 2.4e-12 h); Jacobi 169 / 170 sweeps 3.0e-12 h / 2.6e-12 h.
- A9 bench (`DESIGN.md` has the table): 64^3 Laplacian Colored 0.198 s per sweep = 1.26 M
  node-updates/s, Jacobi 0.242 s = 1.03 M/s, fixed cost 0.34 s; 16^3 Optimize p=4 51.5 ms per
  sweep (0.066 M/s). **The 5 M/s target is not met.** Reason: the guard evaluates
  `ScaledJacobian` and `Volume` on every incident cell through `sm_quality::eval` (150-200 ns per
  call). A per-cell metric cache already halved the evaluations (0.74 M/s before). Fused
  guard kernels are a W3b item.

## Deviations (with reasons)

1. **Branch name**: see above.
2. **Dimension-2 `fix_boundary`**: in a surface mesh, `TopoFace` nodes are the interior nodes and
   `NodeSelect::All` selects them whatever `fix_boundary` says; `fix_boundary` then governs the
   `TopoEdge` nodes. The task text ("volume-classified nodes") would leave a surface mesh with
   nothing to move under the default request. A listed `TopoFace` in a dim-2 mesh plays the volume's
   role for its edges.
3. **Upward-closure rule counts meshed entities only** (entities in `classified_entities`).
   `lattice::quad` meshes one face of a brick, so counting the unmeshed side faces would pin its
   edges forever; a face shared with an unmeshed neighbour volume moves with its volume.
4. **Jacobi** applies the per-node guard against the old positions in the compute phase in
   addition to the repair loop (fewer repairs); both count in `rejected_guard`. After 20 repair
   passes the whole sweep is reverted.
5. **Extras beyond the spec**: pin reasons `vertex`, `unsupported_shape`, `isolated`,
   `winslow_unstructured`; `SmoothReport::pinned_entities` and `inverted_before`; diagnostic
   `smooth.pinned_interface`; a sweep that moves nothing counts as converged.
6. **Armijo**: at most 11 trial steps (initial step plus 10 halvings).
7. **Test parameters the task left open**: A4 seed scan, L region, A7 jitter 0.1 h (see above).
8. **Op-level high-order test** uses an extra coordinate node (the check `coords().len() !=
   vertex_count()`); a real `HighOrderCell` record is tested at library level on the quad
   fixture, because a 3-D record needs the edge table, which mints ids.
9. **Dependencies** (all internal crates from the task list, minus `sm-math`, which is not needed,
   plus the G2 allow-list): `sm-core` (features `serde`, `schemars`), `sm-geom-api`,
   `sm-topo-api`, `sm-model-api`, `sm-mesh-api` (feature `schemars`, so `CellIx` is schema-able
   in the report), `sm-quality`, `sm-op-api`, `sm-layer-api`, `rayon`, `serde`, `serde_json`
   (op diagnostic), `schemars`, `smallvec`, `thiserror`. Dev: `sm-testkit`, `sm-mesh`,
   `sm-mesher`, `sm-scheme-api[stubs,schemars]`, `criterion`, `serde_json[float_roundtrip]`
   (the report JSON must compare equal after a text round trip). No external crate outside the
   allow-list.

## Known gaps

- The 5 M node-updates/s target is not met (see above); no analytic gradients, so `Optimize` is
  slow (0.066 M updates/s) and, being steepest descent on a min-over-corners metric, can stall in
  flat valleys. All fixtures tried converge.
- `smooth.no_improvement` also fires on an already optimal mesh.
- `mesh.smooth_preview` runs the whole smoothing (it is the only way to report the result).
- Deferred to W3b (also in `README.md`): 3-D Winslow, equipotential, untangle, randomize;
  analytic gradients / Newton; polyhedral and high-order meshes; parallel colouring; targeting
  by stored mesh set; dihedral feature preservation; bulk-commit-dependent perf gates. Tied
  smoothing of translation / rotation decks is W4.
- `Partition*` -> `unprojectable` is implemented (it is in the constraint table) but untested: no
  fixture produces partition-classified nodes.
- Acceptance 8 reads "exact `geometry_hash` and state hash" as checked at op level for one
  method (`optimize`, `fix_boundary: false`) across 1, 2 and 4 threads; the bit-identity of
  `points` is checked for every method x schedule x fixture, and the geometry hash is a function
  of the points and classification parameters.
- `quality()` evaluates `Shape` and `Condition` with two `compute` calls (identical values to
  `compute_many`, which is what the task text names; `compute_many` errors when one metric is
  undefined for every shape, the two calls degrade each metric independently).
- Docker is not needed by this task.

## Notes for the orchestrator

- `sm_testkit::lattice::perturb` is quadratic in the node count: it calls `set_class` once per
  node and each call rebuilds the reverse class index (`perturbed` takes 1.1 s at 16^3, 14 s at
  24^3, 78 s at 32^3, so 64^3 would take about 40 minutes). A batched `set_classes` at the end
  would fix it. The 64^3 bench therefore jitters volume nodes with `set_point` only.
- Wiring `SmoothPlugin` into `sm_server::standard()` is the D3 integration item.

## api_change_requests

None.

## Test status

`cargo build --workspace`, `cargo test -p sm-smooth --all-features`,
`RAYON_NUM_THREADS=1 cargo test -p sm-smooth --all-features` (debug, about 3 minutes each),
`cargo clippy -p sm-smooth --all-targets --all-features -- -D warnings` and
`RUSTDOCFLAGS="-D warnings" cargo doc -p sm-smooth --no-deps` all pass.

# W2 report: sm-sizing (T3)

Branch: `claude/w2-sm-sizing-nbd2tk` (the branch the session designated; the task text names
`claude/w2-sm-sizing`, which I did not create — see Deviations).

## Summary

Filled in all four fixed entry points of the skeleton, signatures unchanged:

* `IiaSolver: IntervalSolver` — exact integer branch and bound with the lexicographic objective
  (max relative deviation, sum, smallest vector). Hypergraph components, two-phase global max,
  rayon, reference fallback beyond a documented size / node budget, exact re-verification.
* `AnglePicker: CornerPicker` — Cubit thresholds, exact repair DP, hard requests win.
* `size_field` — `Constant`, `"linear"`, `"geometric"`, `"bias"`.
* `auto_size` — formula from cubit-meshing-semantics §2; typed `SizingError` (thiserror).

Design, objective, limits and benchmark numbers: `DESIGN.md`.

## Acceptance bullet -> test

| Acceptance (plan T3) | Test |
|---|---|
| 4-sided face, goals (3, 5.2, 3.4, 5) -> 3\|3, 5\|5 | `iia::four_sided_face_with_unequal_goals_gives_opposite_equal_counts` |
| Brick, size 1, edge 10 -> all 10 | `iia::brick_with_size_one_and_edge_ten_gives_ten_everywhere` (through `Driver::mesh`) |
| `row_setup(3)` with mixed hard counts -> unique answer | `iia::row_of_three_bricks_with_mixed_hard_counts_has_the_unique_answer` |
| Conflicting hard counts -> `Infeasible` naming both rows | `iia::conflicting_hard_counts_name_both_rows`, `iia::conflicting_hard_counts_through_a_sum_row_name_the_row_and_the_fixed_counts` |
| Even-sum rows with odd goals -> minimal parity repair | `iia::even_sum_with_odd_goals_gets_the_minimal_parity_repair`, `iia::an_odd_hard_sum_is_infeasible` |
| Optimality vs brute force (seeded proptest, <= 6 vars, counts <= 8) | `optimality::matches_exhaustive_enumeration` (5 fixed ChaCha seeds, 400 cases each in debug and 3000 in release, multi-goal and propagation-fixing generator, full argmin vector compared, `Err` iff no assignment), `optimality::matches_exhaustive_enumeration_without_upper_bounds` (unbounded domains, windowing checked), regression tests A, B, C |
| Agreement with `ReferenceIntervalSolver` on the scheme-api stub scenarios | `iia::agrees_with_the_reference_solver_on_the_stub_scenarios` (bricks, sizes, hard counts, `row_setup(2,3,5)`, driver plans compared) |
| Picker: rectangle with mid-edge vertex -> End x4, Side | `picker::rectangle_with_a_mid_edge_vertex_is_four_ends_and_a_side` |
| L-shaped face under Submap -> sum 4, one Corner | `picker::l_shaped_face_under_submap_has_one_corner_and_sum_four` |
| Infeasible request -> error naming the request | `picker::infeasible_requests_name_the_request_and_the_constraints` |
| Size-field integral h = 0.01 + x on a unit edge = ln 101 | `fields::linear_size_integral_on_a_unit_edge_is_ln_101` (via `geom::edge_size_goal`) |
| `auto_size` monotone, exact on unit cube | `fields::auto_size_is_monotone_in_the_factor`, `fields::auto_size_is_exact_on_a_unit_cube` |
| Determinism: 1 vs N threads, permuted component order | `iia::identical_across_threads_runs_and_constraint_order` (`sm_testkit::determinism::check`, rayon pools of 1 and 4 threads, rotated/reversed constraint lists); `RAYON_NUM_THREADS=1 cargo test` also run |

Additional tests: hard-picker requests/soft requests/threshold table, sides equal to the
reference picker on the split brick (`picker::layouts_equal_the_reference_picker_where_the_choice_is_forced`),
reference-fallback regression (`iia::fallback_keeps_rows_whose_terms_cancel`), large unbounded
counts (`iia::large_counts_with_unbounded_domains_stay_fast_and_exact`), larger random systems
never worse than the reference (`optimality::feasible_and_never_worse_than_the_reference_on_larger_systems`),
field parameter validation, bias/geometric analytics, auto size selection rules and curved edges.
Benchmark: `benches/solver.rs`.

## Definition of done

`cargo build --workspace`, `cargo test -p sm-sizing --all-features` (also with
`RAYON_NUM_THREADS=1`, and `--release`), `cargo clippy -p sm-sizing --all-targets --all-features
-- -D warnings`, `RUSTDOCFLAGS="-D warnings" cargo doc -p sm-sizing --no-deps`: all clean.

## Deviations (with reasons)

1. **Branch name.** The session designated `claude/w2-sm-sizing-nbd2tk`; the task text says
   `claude/w2-sm-sizing`. I followed the session instruction (never push to another branch).
   The orchestrator merges by SHA.
2. **Two-phase solve.** The plan says components can be solved independently. They cannot for
   this objective: the maximum deviation is global, so a component below the global maximum
   must still minimize its own sum under the global cap. The brute-force proptest found this
   (a rowless class was optimized against its own maximum). Implemented as phase 1 (component
   optima, `D*` = largest maximum) and phase 2 (re-solve slack components under the cap `D*`).
3. **Soft vs default goals.** Plan is silent. A class with soft counts uses only those as goal
   terms, otherwise its goals and default counts, else goal 1. No weights (the reference's 10x
   is not used). Documented in the `iia` module docs.
4. **Objective terms are per edge** (every goal of every edge of a merged class), not one mean
   goal per class; hard-fixed classes contribute no term.
5. **Cold replay / `Diverged` check does not apply.** The crate registers no ops or layers, so
   there is nothing to replay; the determinism tests compare solver and picker output instead.
6. **Dependencies.** Used: sm-core, sm-topo-api, sm-scheme-api + `rayon`, `thiserror`,
   `serde_json` (size-field params). `sm-math` and `sm-geom-api` from the skeleton were unused in
   the library and are dropped (`sm-geom-api` stays as a dev-dependency for test fixtures);
   dev: sm-testkit, sm-scheme-api[stubs], sm-mesh-api, sm-model-api, proptest, criterion.
   **Deviation for approval:** the plan's Dev line lists only sm-testkit and sm-scheme-api[stubs];
   the extra dev-only crates (sm-model-api, sm-mesh-api, sm-geom-api, proptest, criterion) are
   contract crates and test tools used by tests and the benchmark. No library dependency
   changes. The orchestrator should approve or reject.
7. **`SizingError::NotImplemented` removed** (skeleton placeholder); new typed variants
   (`BadFactor`, `NoGeometry`, `UnknownEntity`, `Geometry`, `Degenerate`), enum is
   `#[non_exhaustive]`.
8. **`IiaSolver` gained two public fields** (`max_component_classes`, `max_nodes`) and
   `with_limits`; `Default` and `IiaSolver::default()` are unchanged.
9. **`bias` size field**: the plan says only "Bias/Geometric along an axis"; I define three
   functions (`linear`, `geometric`, `bias`) with the parameter sets in `field.rs` docs.

## Lockfile

`Cargo.lock` changes only the `sm-sizing` entry (dependencies added/removed above); there is no way to avoid it with cargo.

## Known gaps

* Beyond 512 classes in one component, or after 400 000 search nodes, a coupled component is
  solved heuristically (best of search incumbent and reference solver); optimal only below the
  limits. Pruning uses per-class objective bounds only (no row-aware bound), so dense coupled
  `>=` rows over many free classes can hit the node budget.
* `bias`/`geometric`/`linear` fields are not tested against Cubit numerically (no Cubit here).
* Phase 2 of a component whose search ran out of budget keeps the best candidate found, not a
  proven optimum.
* The picker has no weights for "turn" and "interval change" (Cubit's picker weights); only
  angle and request firmness.

## api_change_requests

* **sm-scheme-api** (non-blocking): make `solve::sides_of` public (or expose
  `FaceLayout::from_types(face, rule, corners)`). `AnglePicker` re-implements the 25-line
  side construction; a test pins equality with `ReferenceCornerPicker` on the split brick, but a
  shared function would remove the duplication risk.

## Fix round 0 (review findings)

Must fix:

1. Candidate scan in `Search::dfs` now splits at `floor(center)` (left = floor, right =
   floor + 1), so the max deviation is monotone on each side even for asymmetric multi-goal
   curves. Fixes repros A and B.
2. The deviation window in `solve_local` now skips only slots without goal terms; propagation-fixed
   slots are windowed too (an empty window empties the level). Fixes repro C.
3. Optimality tests: multi-goal and propagation-fixing generator, 5 seeds, release case counts of
   3000 per seed, an unbounded-domain variant, regression tests A, B, C. Verified that the new
   tests fail on the previous solver (A, B, C and the unbounded variant).
4. `auto_size`: the clamp bounds can cross (closed circle, helix); `hi = max(d/2, l_min/4)`, so
   the lower bound wins and nothing panics. Regression test on a closed circular edge.

Should fix: all done except as noted. Brute force precompiles index maps (debug run of the
optimality tests drops from ~125 s to ~25 s even with 5x more cases). `first_violation` with a
zero count returns the first constraint that mentions the zero-count edge (documented).
`IiaSolver::solve` rejects unknown `IntervalConstraint` kinds with a typed `Infeasible`;
`first_violation` treats them as violations. Rows are processed in `ConstraintId` order.
Picker distances are quantized to 2^-24 rad. Item 10 (weighted change cost) is recorded as a
deviation in DESIGN.md and the module docs rather than changed. Item 6 is in DESIGN.md Known
gaps. Item 11 is stated in deviation 6 above.

Questions for the orchestrator (no code change):

* Confirm or reject deviations 3 and 4: hard-fixed classes contribute no goal term, and a class
  with any Soft count ignores its Goal terms. Both can change the argmin versus an "all goals"
  reading of plan T3.
* `auto_size` with the `l_min / 4` floor: factors 1..8 give the same size on a unit cube (the
  floor binds). Consistent with cubit-meshing-semantics section 2 and the plan, but it means the
  factor has no effect on compact parts. Is that intended?
* The picker's weighted change cost (soft 2, other 1) versus the plain "fewest types" of plan T3.

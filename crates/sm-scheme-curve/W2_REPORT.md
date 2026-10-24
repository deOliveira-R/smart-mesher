# W2 report: T4 sm-scheme-curve

## Summary

All five schemes are implemented and registered by `sm_scheme_curve::register`: `equal`, `bias`, `dualbias`, `stretch`, `curvature`. `equal` is the first-class path (exact on lines, 1e-12 on arcs, 1e-9 on NURBS) and also the baseline the others reduce to. Spacing is computed in arc length and converted to parameter fractions by an adaptive Gauss-Legendre map with a monotone inverse; `GenCtx::curve_patch` owns keys and positions. Parameters are typed serde structs in `SchemeChoice::params`. No other crate was touched.

Definition of done (run on this tree): `cargo build --workspace`, `cargo test -p sm-scheme-curve --all-features` (45 tests), the same with `RAYON_NUM_THREADS=1`, `cargo clippy -p sm-scheme-curve --all-targets --all-features -- -D warnings`, `RUSTDOCFLAGS="-D warnings" cargo doc -p sm-scheme-curve --no-deps`: all pass.

## Acceptance -> tests

| Acceptance bullet | Test(s) |
|---|---|
| equal on a line: nodes at `i L/n` within 1 ulp-scale | `engine::equal_on_a_line_is_i_l_over_n_to_an_ulp` (forward and reversed sense; tolerance 2 eps*(1+L)) |
| equal on a circular arc: equal arc lengths within 1e-12 L | `engine::equal_on_a_circular_arc_has_equal_arc_lengths` |
| equal on NURBS within 1e-9 relative | `engine::equal_on_nurbs_has_equal_arc_lengths`, `engine::equal_on_a_rational_quarter_circle_hits_the_analytic_angles` (analytic angles, 1e-10), `engine::equal_on_an_ellipse_has_equal_arc_lengths` |
| bias ratio r: segment ratios = r within 1e-12, total length exact | `spacing::bias_consecutive_edge_ratio_is_r`, `engine::bias_on_an_arc_has_edge_ratio_r_and_reverse_flips_it`, `driver::bias_ratio_through_the_driver` |
| bias `first_size` recovers r by Newton, closure checked | `spacing::bias_first_size_recovers_ratio_with_closure`, `engine::bias_first_size_on_a_nurbs_curve_closes`, `driver::bias_first_size_recovers_ratio_and_closes` |
| `start` on the far vertex reverses the sequence | `driver::bias_start_on_the_far_vertex_reverses_the_sequence`, `engine::bias_on_an_arc_has_edge_ratio_r_and_reverse_flips_it`, `spacing::reversed_mirrors_the_edges` |
| dualbias mirror-symmetric within 1e-12 (even and odd n) | `spacing::dualbias_even_and_odd_are_mirror_symmetric_with_ratio_r`, `engine::dualbias_on_an_ellipse_is_mirror_symmetric`, `driver::dualbias_is_mirror_symmetric_through_the_driver`, `spacing::dualbias_first_size_recovers_ratio` |
| stretch: first segment = `first_size` within 1e-12, ratio never exceeds `max_ratio` | `spacing::stretch_pins_first_and_last_and_bounds_the_ratio`, `engine::stretch_first_segment_equals_first_size`, `driver::stretch_pins_the_first_edge_and_derives_the_count`, refusal: `spacing::stretch_refuses_instead_of_exceeding_max_ratio`, `driver::stretch_beyond_max_ratio_fails_instead_of_relaxing` |
| curvature = equal on a line and a circle | `engine::curvature_reduces_to_equal_on_a_line_and_a_circle`, `driver::curvature_on_a_straight_edge_is_equal` |
| curvature on an ellipse: shorter where curvature is high, monotone | `engine::curvature_on_an_ellipse_is_monotone_and_smaller_where_curvature_is_high` (both senses, c > 0 and c < 0), `engine::curvature_density_matches_its_definition` |
| brick through `Driver::mesh` with the stub map: classified nodes on edges | `driver::equal_brick_edge_nodes_are_classified_and_analytic`, `driver::bias_ratio_through_the_driver` (classified count = n-1, mesh valid) |
| invalid params give `NotApplicable` with a reason, never a panic | `driver::invalid_params_are_not_applicable_with_a_reason`, `engine::degenerate_inputs_are_errors_not_panics`, `spacing::bias_first_size_errors_are_typed`, `spacing::bias_extreme_ratio_is_an_error_not_a_panic`, `engine::curvature_refuses_a_density_it_cannot_resolve` |
| determinism (1 vs N threads, concurrent) | `engine::node_params_are_bit_identical_for_1_and_n_threads` (rayon paths included), `driver::driver_output_is_bit_identical_across_executors_and_runs` |
| register / no auto list | `driver::register_adds_the_five_curve_schemes_and_no_auto_list` |
| derived counts | `driver::bias_fine_and_coarse_sizes_derive_the_count`, `driver::bias_ratio_and_first_size_derive_the_count`, `spacing::bias_count_from_ratio_matches_series_sum`, `spacing::bias_count_from_sizes_minimizes_the_length_mismatch` |
| closed edges | `driver::equal_outscores_the_other_schemes_and_closed_curves_refuse_stretch_and_curvature` (cylinder circle edge) |

The cold replay from root (`Session::state_at_from_root`) does not apply: this crate defines no ops and no layers, only pure schemes. The scheme-level determinism above is what it can offer.

## Deviations (with reasons)

1. **Fine + coarse count.** The `[Ours]` text iterates `n -> r(n) -> n`, which is a fixed point at every `n` (it never moves). Replaced by a doubling search and bisection for the integer minimizing `|series sum - L|` (the sum grows with the count). Same intent, well-defined.
2. **Bias first-size solve.** The plan says Newton, `[Ours]` says bisection; implemented as a bracketed Newton with bisection fallback (the plan wins and the robustness of bisection is kept).
3. **Extra mode `bias {ratio, first_size}`** (not in the plan's list): derives the count (Cubit's "fine size with a factor"), then builds the series from `ratio` over the plan's count.
4. **Derived counts are soft `Fixed` constraints** (bias fine+coarse, bias ratio+first, stretch first+last). The solver takes the max of soft counts on a class, so a conflicting user soft count can be overridden; a hard count always wins. `IntervalSink` has no goal-style input, which would be the better fit (see requests).
5. **`bias {first_size, last_size}` with a given count** pins `first_size` and lets the last edge follow (the count is the part `last_size` controls). `stretch {first, last}` pins both end edges for 3 or more intervals; with 2 intervals the second edge is `L - first`.
6. **`stretch` default `max_ratio` = 2.0** (my choice; the plan leaves the default open). A first-size-only stretch is a geometric series and is subject to the same limit.
7. **One interval = no interior nodes**: with `n = 1` every scheme returns no interior fractions without validating sizes.
8. **Arc-length convergence tolerance 1e-13** (cap 16384 panels), not machine precision: reference NURBS curves converge algebraically at knots. Measured against an independent 4000-panel rule the NURBS test passes at 1e-9.
9. The skeleton's `sm-math` dependency is kept but unused.
10. **No criterion bench**: the task names none.

## Known gaps

- Bias with no parameters ("use vertex sizes") is `NotApplicable`: the scheme API gives no vertex size access.
- Stretch with a bare `stretch_factor` (Cubit) is not in the plan's param set and is not implemented (use `bias {ratio}`).
- Periodic/closed curves: stretch and curvature are refused (as in Cubit). `bias.start` on a closed edge is refused as ambiguous.
- The arc map is rebuilt in `applicability`/`interval_constraints` (length only) and `generate`; `Requirements::set_data` caching would remove the repeats (negligible next to meshing).
- The adjusted first size after a derived count is not reported (no diagnostics channel in the scheme API).

## api_change_requests

None blocking. Suggestion (non-blocking): `IntervalSink::goal(edge, real)` exposing `IntervalConstraint::Goal`, so schemes that derive a count from sizes can state a real-valued goal instead of a soft integer.

## Delivery notes

Branch `claude/w2-sm-scheme-curve-52efyt` (the session's designated branch) and `claude/w2-sm-scheme-curve` (the name in the task) point at the same commit. The final commit subject starts with "w2 sm-scheme-curve: done".

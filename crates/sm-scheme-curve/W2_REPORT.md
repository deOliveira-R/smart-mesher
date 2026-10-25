# W2 report: T4 sm-scheme-curve

## Summary

All five schemes are implemented and registered by `sm_scheme_curve::register`: `equal`, `bias`, `dualbias`, `stretch`, `curvature`. `equal` is the first-class path (exact on lines, 1e-12 on arcs, 1e-9 on NURBS up to the knot-span limitation in Fix round 1) and also the baseline the others reduce to. Spacing is computed in arc length and converted to parameter fractions by an adaptive Gauss-Legendre map with a monotone inverse; `GenCtx::curve_patch` owns keys and positions. Parameters are typed serde structs in `SchemeChoice::params`. No other crate was touched.

Definition of done (run on this tree): `cargo build --workspace`, `cargo test -p sm-scheme-curve --all-features` (65 tests), the same with `RAYON_NUM_THREADS=1`, `cargo clippy -p sm-scheme-curve --all-targets --all-features -- -D warnings`, `RUSTDOCFLAGS="-D warnings" cargo doc -p sm-scheme-curve --no-deps`: all pass.

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
8. **Arc-length map is locally adaptive** (see fix round 0): per-panel tolerance 1e-14 of the initial length, up to 2^16 panels, `Geometry` error beyond that.
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

## Fix round 0 (review findings)

Must fix 1: `ArcMap` now refines panels locally (bisect while whole vs split estimates differ by more than 1e-14 L; non-uniform breakpoints kept in `tb`/`cum`; inversion runs in the refined panel; budget overrun is a `Geometry` error). New `tests/nurbs.rs`: double knot (quadratic) and triple knot (cubic) at 0.37, equal arc lengths within 1e-9 (both senses), and a bias `first_size` closure on the same curve.

Should fix, all done:
1. Line test asserts fractions are bit-exactly `(i+1)/n`; position check rescaled to `16 eps L`.
2. Curvature samples with zero/non-finite speed or curvature give `Geometry` (test: cusp).
3, 9. `spacing::stretch_count_from_sizes` caps the derived count at `MAX_COUNT`, now lowered to 2^22 for all count paths (typed `Unreachable`).
4. Dualbias `first_size` must be strictly below L/2 (no 1e-12 slack), matching `dualbias_ratio_from_first` for n >= 3. Side effect: `first_size = L/2` with n = 2 is refused at applicability.
5. `resolve` takes lengths from the same `ArcMap` as `node_params` (`engine::arc_length`).
6. `monotone_root` clamps the last bracket step to the cap (test: ln ratio = 600).
7. Stretch ratio test asserts the Constraint kind and the ratio-limit reason.
8. Curvature samples kappa at arc-length fractions through `ArcMap::taus`, integrates in arc length, inverts, then maps. Tests: NURBS quarter circle and non-uniform NURBS line match equal within 1e-12.
10. `applicability` derives counts (`want_count = true`). Note: the review's example `{ratio 0.5, first_size 0.9}` is reachable on a unit edge (first/(1-r) = 1.8); the test uses `first_size 0.4` (0.8 < L).
11. Determinism test runs Sequential vs `ScopedThreads(N)` and compares `check::mesh_hash` of the finished mesh plus the node distances.
12. End-to-end test builds with `sm_mesh::MeshBuilder`. The `sm-mesh-api` dev-dependency stays: `MeshRead`, `SerialAlloc` and `validate` are only exported there (not re-exported by `sm-mesh` or `sm-scheme-api`).
13. `sm-math` left declared, consistent with the skeleton (deviation 9).

## Fix round 1 (review findings)

Must fix 1 and 2 (arc map misses kinks near panel edges and short knot spans): `ArcMap::new` now starts from 64 panels and accepts a panel only if (a) whole vs halves agree (as before), (b) the 5-point Gauss-Legendre estimate agrees with a 5-point Gauss-Lobatto estimate within 1e-13 L (samples both panel endpoints), and (c) the chord `|C(b)-C(a)|` does not exceed the arc length (relative 1e-12 plus an absolute rounding slack). New tests in `tests/nurbs.rs` against a per-knot-span reference: case (a) double knots 0.3141/0.7183 with bias first-size and ratio/closure, case (b) degree-1 polyline with knots 0.123/0.456/0.789, case (c) double knot at 0.0013 (plus curvature sanity), short interior span [0.499, 0.501] for cubic C2 and C1 (with ratio closure), short end span at 0.002 (quadratic, and cubic C2 with {0.002, 0.3, 0.7}); all equal arc lengths within 1e-9, both senses.

Remaining limitation (honest): the map is not given the knots. A knot span narrower than 1/64 of the edge that falls between quadrature nodes, with no visible speed contrast and negligible displacement, can still be missed without an error. DESIGN.md and the claims above are qualified accordingly. A full guarantee needs knot breakpoints.

api_change_request (for the orchestrator, not done here): add `ParametricCurve::breakpoints(&self) -> Vec<f64>` (default empty; NURBS returns its distinct interior knots) so the arc map can split panels at them and integrate each smooth piece exactly.

Should fix:
1. Stretch ratio slack lowered from 1e-9 to 4 ulps (`max_ratio * (1 + 4 eps)`), covering only rounding of the computed ratio.
2. `node_params` returns `Unreachable` for `n` above `MAX_COUNT` (2^22) before any allocation (test: `huge_interval_counts_are_refused_before_allocation`, Equal and Curvature).
3. Not tightened: the 1e-12 ratio/closure bound is met on analytic curves (lines, arcs, ellipses). On rational NURBS with extreme weights (1, 30, 1, 0.05, 1 and a double knot) it is about 1e-12 to 1e-9; the NURBS bound is 1e-9 (Gauss-Legendre/Lobatto tolerances 1e-14/1e-13 of L and curve conditioning).
4. Curvature samples at arc-length fractions through the corrected map; case (c) is covered by a well-formedness test (finite, strictly increasing nodes). Curvature spikes narrower than a panel are still limited by the sampling and by the knot limitation above.
5. `sm-mesh-api` dev-dependency stays (MeshRead, SerialAlloc, validate are exported only there); the orchestrator should accept it explicitly or re-export from `sm-mesh`.
6. `sm-math` stays declared but unused; note for cleanup.
7. Cargo.lock at the repo root changed in the earlier round (serde, serde_json, thiserror, rayon); confirm at merge. Not touched here.
8. NURBS accuracy claims in DESIGN.md and this report are qualified.

## Integration fix: curve breakpoints (w2-int curve-bp)

The T4 review found the arc map missed `equal` within 1e-9 on NURBS with a very short knot span (1e-5..1e-7) and O(1) displacement, because adaptive panels were accepted at the absolute width floor `MIN_WIDTH = 1e-13`. Fixed with an additive contract change: `ParametricCurve::breakpoints(&self) -> Vec<f64>` (default empty; distinct interior parameters where the curve is not C-infinity), implemented for `NurbsCurve`, `PCurve`/`NurbsCurve2` (inherent), `sm_geom::PreparedCurve` and `CurveOnSurface` (pcurve breakpoints). No hash or persisted-format change.

- `ArcMap` seeds its initial panels at the breakpoints inside the edge range (merged with the 64-panel grid, repeated per period on a seam-spanning edge, cuts refined so they map back onto the knot exactly), so each smooth piece is integrated on its own. The absolute floor is replaced by a relative one (`b - a <= 4 eps max(|a|, |b|, 1)` or a midpoint not strictly inside). Two noise guards were required once panels get that narrow: Lobatto endpoint nodes shift inward by 8 parameter ulps, and the chord slack includes (parameter ulp x local speed).
- Four new tests in `tests/nurbs.rs` (now 13 in that file; 59 in the crate): degree-1 polyline with knots at 20/64+3e-4 and +1e-6, the same near the far end, quadratic double and cubic triple knots at the short span, and a seeded 200-curve probe (degree 1-3, spans 1e-7..1e-3, both senses) against a per-knot-span reference. All hold 1e-9 relative, plus one allowance: a node cannot be placed finer than one ulp of the parameter, so the probe adds `eps * max speed` of arc length (the 1e-7 spans reach ~1e7 speed, about 3e-9 absolute). Without that allowance no implementation using a double parameter can pass.
- The "knots unknown" limitation and the earlier `api_change_request` are closed. Remaining: a non-smooth point a curve does not report is found only by adaptive refinement.

### Fix round 1 (curve-bp review)

- Spurious `Geometry("did not converge")` on valid NURBS whose range is away from 0 (e.g. [100, 101], [-7.5, -6.5]) is fixed. Cause: basis divisions cancel at |t| ~ 100, so speed in a 1.6e-7 span has about 1e-7 relative noise and the 1e-14 / 1e-13 panel tolerances can never be met. Each panel now carries the parameter width of its smooth piece and the whole-vs-halves, Lobatto and chord tests include a floor of 16 x `eps * max(|lo|, |hi|) / width` (relative, times the panel integral). Minimal repro (quadratic, knots off+[0,0,0,a,a,a+1.63e-7,1,1,1], n = 6) now passes for off = 0, 100, -7.5, 1000; the shifted probe failed at the previous commit with exactly that error.
- New tests (19 in `tests/nurbs.rs`, 65 in the crate): the repro, shifted short spans (degrees 1-3, offsets 100 and -7.5), sub-range edges on a shifted curve, a periodic seam next to a 1e-6 span, a 70000-knot polyline and an edge over 1e12 periods (typed error), and a 300-curve probe over offsets {0, 3, 100, -7.5}, scales 0.5-10, rational and sub-range cases, both senses.
- Periodic curves: the seam `domain.lo + k * period` is seeded along with the interior breakpoints. Seed count is capped (2^20 seeds, typed error) and the panel budget grows with the seed count, so a dense knot vector no longer reports a misleading non-convergence.
- rustfmt glitch in `arc.rs` fixed.
- **Ruling requested (orchestrator):** the probes do not hold a strict 1e-9 relative bound. They hold 1e-9 relative plus one parameter ulp times the local max speed (`eps * max(|lo|, |hi|, 1) * vmax` of arc length). A node cannot be placed finer than one ulp of `t`, and a span of 1e-7 with O(1) displacement has speed ~1e7, so the ulp is ~1e-9 of arc length: no double-parameter implementation can do better. Against strict 1e-9 the reviewer measured a worst ratio of 160x on the 1e-7 spans; against the allowance 0.45. Plan T4's "within 1e-9 relative" needs an explicit amendment to that effect, or the case is declared out of scope.
- **Open (not done):** `CurveOnSurface::breakpoints` still reports only the pcurve's breakpoints. Crossings of surface knot lines by the pcurve are C^k breaks of the 3D curve; reporting them needs a surface-side breakpoint accessor in the frozen `sm-geom-api` plus root finding per crossing, so a short surface knot span can still show the original failure on curves on surfaces. Left for an orchestrator decision.

### Follow-up: curve sliver (w2-fu)

- Bug: `ArcMap::cuts` dropped any breakpoint within 4 eps (fraction) of `tau = 0` or `1` through `is_tiny`, so the few-ulp sliver past a knot was integrated at the speed of the neighbouring short fast piece (degree-1 knots ...0.5001080718327429, 0.5001081924016249, edge [0, 0.5001081924016252]: length 3.3e-9 too long, worst segment 5.6e-9 relative).
- Fix: breakpoints are always kept when the panel midpoints on both sides are representable; only uniform-grid cuts are dropped for crowding (and a grid cut just before a breakpoint is removed in its favour).
- New test `edge_ends_within_ulps_of_a_knot_get_their_own_panel` (edges ending -8..+8 ulps from either knot at both ends, both senses).
- Probe tolerance (`worst_ratio` in `tests/nurbs.rs`: 1e-9 relative plus `eps * max(|lo|, |hi|, 1) * vmax`) is the decision W2-10 wording (one fraction ulp times local speed); now documented in DESIGN.md.

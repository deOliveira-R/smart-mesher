# sm-sizing

Production interval matching, corner picking, size fields and auto size for the smart-mesh
meshing driver (W2 task T3). The crate implements `sm-scheme-api` traits only (`IntervalSolver`,
`CornerPicker`, `SizeField`); it registers no ops or layers and depends on no scheme crate. The
mesher receives it through `MesherConfig` injection.

## Positioning

Similar to Cubit's interval assignment (IIA: an integer program over curve intervals with
firmness, equal-sum rows and a lexicographic min-max objective), but different in how the answer
is certified. Cubit's solver is a heuristic ("global optimality is not guaranteed"). `IiaSolver`
is an exact integer branch and bound on the same constraint family with a *fully specified*,
tie-free objective, so the result is the unique minimizer: independent of thread count,
constraint order and component order, and checkable against exhaustive enumeration. That gives
reproducible meshes (a hard requirement for the timeline/replay design) and lets the test suite
prove optimality instead of sampling it. `AnglePicker` is Cubit's End/Side/Corner/Reversal typing
with an exact "fewest changes" repair instead of a greedy one.

## Interval solver (`IiaSolver`)

Objective (plan T3), lexicographic, over all goal terms `|n - g| / g`:

1. the maximum relative deviation,
2. the sum of relative deviations,
3. the smallest count vector in ascending edge order.

Reduction:

* `x - y = 0` rows merge edges into classes (named by their lowest edge). A class with a `Hard`
  count is fixed and has no term. Otherwise its terms are its `Soft` counts if any, else its
  `Goal`s and default-firmness counts, else the implicit goal 1. Soft therefore beats default
  with no arbitrary weight (the reference solver's 10x is not used).
* Remaining `Linear` rows become integer rows over classes; `EvenSum` rows become parity rows
  over the classes of odd multiplicity. Rows that reduce to nothing hold for every assignment.
* Components are the connected sets of classes sharing a row (hypergraph, not just pairs).

Exactness:

* Deviations are quantized to integers, `round(|n-g|/g * 2^40)`, so maxima and sums are exact
  and ties are real ties. No floating-point LP is used anywhere, so there is nothing to verify
  after the fact; the final assignment is nevertheless re-checked against every constraint in
  integer arithmetic (`first_violation`).
* Search: depth-first over classes in lowest-edge order, values tried outward from the minimizer
  of the max deviation, pruned by (a) interval bound propagation on every row, (b) parity
  propagation, (c) the objective lower bound against the incumbent (the max term, the sum term
  and the already-fixed prefix of the vector).
* Domains with no upper bound are windowed by a deviation cap `D = 0.5, 1, 2, ... 2^40`, whose
  windowed optimum is the global optimum as soon as one feasible point exists inside it.
* The max term couples components, so there are two phases: each component finds its optimum
  and thereby `D*`, the global maximum; components strictly below `D*` are then re-solved for
  (sum, vector) under the cap `D*`. The brute-force proptest found this: independent per-component
  optimization is wrong for this objective.

Parallelism and determinism: components are solved with rayon and collected in component order.
Tests compare 1 against N threads, repeated runs and permuted constraint order.

Limits and fallback: a component over `max_component_classes` (512) classes, or one that exhausts
`max_nodes` (400 000) search nodes, goes to `ReferenceIntervalSolver` restricted to that
component. The reference answer is verified exactly. No solution gives `Infeasible` with the
constraint ids and edges involved. A node-budget overrun in phase 2 keeps the phase-1 values
(feasible, optimal in the maximum only).

Infeasibility reports name the constraints (hard counts, ranges and merge rows of the involved
classes, plus the violated row) and the edges involved, never relaxing a hard count.

## Corner picker (`AnglePicker`)

Classes by interior angle: `End <= 135 deg`, `Side < 225`, `Corner < 315`, else `Reversal`.
`Map`/`Ends(n)` allow only End and Side (corner sum `n`); `Submap` allows all four (sum 4);
`Free` just classifies. Requests: `Hard` pins a corner; `Soft` and default requests start the
corner at the requested type with change cost 2 and 1.

An exact DP over the corner sum minimizes (weighted number of changed corners, total angular
distance to the nominal angles). Ties go to the lowest vertex id (the corner that is adjusted),
then to the type order. A hard type forbidden by the rule, or an unreachable sum, is an
`Infeasible` carrying the face, the constraint ids and the hard requests. Logical sides are cut
and oriented exactly as the reference picker does; a test pins equality with it.

## Size fields and auto size

`size_field`: `Constant` through the contract builder; `"linear"` and `"geometric"` interpolate
between two points (clamped projection), `"bias"` is the continuous geometric series
`h0 + (ratio - 1) s` along an axis. Parameters are validated; failures are `BadParams`.

`auto_size`: `clamp(0.01 d 2^((f-1)/2), l_min/4, d/2)` with `d` the bounding-box diagonal and
`l_min` the shortest edge of the selection (cubit-meshing-semantics section 2). Non-decreasing in
`f`; on a unit cube factors 1 to 8 give exactly 0.25.

## Performance

`cargo bench -p sm-sizing --bench solver` (criterion, release, development container, median):

| problem | `IiaSolver` | `ReferenceIntervalSolver` |
|---|---|---|
| 5000 independent quad faces (20 000 edges, parallel components) | 16.3 ms | 29.8 ms |
| 200 faces, 8 of them coupled by one `>=` row | 0.91 ms | 0.15 ms |
| 12 faces all coupled by one `>=` row (one 12-class branch and bound) | 7.0 ms | 0.010 ms |

The exact solver is faster than the reference on the structured case that real meshing produces
(many small components) and pays for optimality on coupled components, where the reference
returns a feasible but not optimal point. Coupled rows are rare outside user constraints.

## Tests

* `tests/optimality.rs`: seeded proptest against exhaustive enumeration (up to 6 variables,
  counts up to 8), and a larger-system property against the reference solver.
* `tests/iia.rs`: acceptance rows, stub-scenario agreement with the reference, determinism.
* `tests/picker.rs`, `tests/fields.rs`: analytic checks.

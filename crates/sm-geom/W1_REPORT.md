# W1 report: sm-geom

status: done (definition of done met; known gaps listed below)

Branch: `claude/w1-sm-geom-86oupw`, the branch the session designated and the only one pushed. The task text names `claude/w1-sm-geom`; it was not created, because the session instruction forbids pushing to any other branch. Merge by SHA (see "Commit").

## Summary

`sm-geom` implements the geometry kernel behind the frozen `sm-geom-api`:

- **Prepared NURBS** (`PreparedCurve`, `PreparedSurface`): homogeneous-pole storage, allocation-free de Boor evaluation with degree-sized scratch (jets to second order, position-only fast path), exact knot insertion / refinement / clamping / splitting / Bezier extraction for clamped **and periodic** data, global closest-point projection (Bezier-box branch and bound + safeguarded Newton), batched parallel evaluation and projection, iso-curve extraction, and a surface **ray hook** (the contract's reference NURBS surface returns `Unsupported`).
- **Exact rational NURBS** of circle, ellipse, arcs, cylinder, cone, sphere, torus, plane patch and conic arcs, tested against the analytic forms (implicit residual 1e-12, curvature, area by quadrature, arc length, projection).
- **`KernelStore`**: `GeomStore + GeomAppend + GeomStoreMut`, same handles and semantics as `MemoryStore` (differential tests), `truncate`, `StoreData` round trips, optional serde feature, prepared NURBS behind `curve(id)` / `surface(id)`.
- **`CurveOnSurface`**: pcurve lifted to space, second derivatives by the chain rule (the contract's `PCurve` has `eval` / `d1` only).
- **Intersections**: curve-plane; curve-surface (closed forms, exact trigonometric restriction for circles and ellipses, Bezier subdivision plus Newton for NURBS); surface-surface with exact curves for plane-plane, plane against every quadric (ellipse, circle, line pairs, point, and parabola / hyperbola as exact rational NURBS in a window; cone nappe handled), sphere-sphere, torus cut perpendicular to or through the axis; and a marching tracer for everything else (plane-torus in general, quadric pairs, any NURBS pair) with iso-curve seeding, closure, boundary and seam joining. `MarchedCurve` provides parameters on both surfaces, pcurves and an interpolating NURBS fit.
- Criterion bench, determinism test, property tests with `sm-testkit`.

## Test summary

`cargo test -p sm-geom`: 75 tests in 12 binaries, all passing (also with `RAYON_NUM_THREADS=1`); 4 proptest suites use the testkit's fixed seed (`config(64)` / `config(48)`).

| Binary | Tests | What |
|---|---|---|
| lib (unit) | 26 | roots (double roots, Brent), basis (partition of unity, FD), knot ops (open and periodic insertion, clamping, Bezier, de Casteljau), prepared curve/surface vs the reference evaluator, periodic surface editing and projection, periodic arc length, exact circle / ellipse / cylinder / sphere / torus / cone (implicit residual, curvature `K`, `H`, area by quadrature, length), pcurve helix, Bernstein roots, fit |
| `curve_surface` | 6 | line / circle / ellipse / NURBS against planes; line, circle, ellipse, NURBS against sphere, cylinder, torus, cone nappe; NURBS circle against NURBS sphere; line against NURBS torus (hand values) |
| `ssi_closed` | 5 | plane-plane / sphere / cylinder (circle, 45 deg ellipse `a = r sqrt 2`, lines, tangent) / cone (all conics, nappe, rays) / torus cuts / sphere-sphere / elliptic cylinder / ellipsoid |
| `ssi_march` | 6 | perpendicular cylinders (two closed loops, length vs quadrature 2e-5), Steinmetz (coverage of the exact ellipses), oblique plane-torus (closed loops), sphere-cylinder, NURBS sphere-plane (length vs `2 pi rho`), NURBS-NURBS pair, determinism |
| `ssi_props` | 4 | frame independence under random rigid / mirror transforms (testkit strategies); plane origin re-anchored in its plane (offsets to 1e6) |
| `nurbs_props` | 3 | random NURBS: prepared == reference, insertion / split keep shape, projection globally optimal vs dense scans. **Covers only clamped curves and surfaces with distinct interior knots and degree <= 5** (curves 1..5, surfaces 1..3); repeated knots, unclamped and periodic data are in `nurbs_props_general` (the original claim of exact periodic insertion rested on uniform-knot unit tests only) |
| `nurbs_props_general` | 4 | repeated interior knots (multiplicity up to p + 1), unclamped non-periodic, degrees 8..15, non-uniform periodic curves with repeated knots and ulp-different period copies: prepared == reference, insertion / split keep shape, plane hits lie on the plane |
| `fix0_curves` | 6 | C^-1 knot (curve and surface), NURBS contacts within tolerance, tiny spheres, circle / ellipse against NURBS plane and sphere over shifted and multi-period ranges (differential), periodic arc length over 1e6 periods |
| `ssi_near_tangent` | 7 | near-equal perpendicular cylinders (dr 1e-2..1e-4), gaps within `tol.linear` for plane-sphere, plane-cylinder, sphere-sphere (external and internal), plane re-anchoring, sphere touched by a cylinder, parabola entering the window twice |
| `store` | 4 | `KernelStore` vs `MemoryStore` (handles, defs, jets, domains, projections, replace, truncate, validation); testkit surface fixtures through the store (normals, `K`, `H`, closest points) |
| `misc` | 3 | NURBS ray hook vs analytic torus; marched pcurves, fitted NURBS; NURBS pcurve lifted onto a plane |
| `determinism` | 1 | `check_f64_bits(2, 4, ..)` over batch eval/projection, marched and closed-form intersections, store; hash equal for 1, 3, 4 threads |

Definition of done: `cargo build`, `cargo test -p sm-geom`, `RAYON_NUM_THREADS=1 cargo test -p sm-geom`, `cargo clippy -p sm-geom --all-targets --all-features -- -D warnings`, `RUSTDOCFLAGS="-D warnings" cargo doc -p sm-geom --no-deps`: all clean. `missing_docs` is satisfied for every public item.

## Deviations from the contracts' intent

1. **Analytic kinds are not re-implemented.** The contract crate already ships exact closed-form implementations of every analytic curve and surface (Q4 decision), the orphan rule forbids a second `impl` of its traits on its types, and `SurfaceDef::as_surface` always returns them. `KernelStore` therefore resolves analytic handles to those evaluators; sm-geom adds NURBS, the store, batching and intersections. Differential tests compare the store with the reference, so analytic results are bit-identical by construction.
2. **`sm-math` is an empty skeleton**, so "use sm-math for roots/predicates" was not possible. A private `roots` module (Brent, stable quadratic, polynomial roots with double-root flag, dense solve) lives in sm-geom; it should migrate to `sm-math` (coordination note, not an API change). The api's own `real_roots` / `solve_quadratic` are `pub(crate)`. No exact predicates (`orient*`, `incircle`) are needed by the geometry kernel.
3. **Branch**: developed and pushed on `claude/w1-sm-geom-86oupw` (the session-designated branch); `claude/w1-sm-geom` from the task text was not created. No worktree was used; this cloud checkout was the worktree.
4. **`Cargo.toml`**: dropped the unused `sm-math` and `thiserror` skeleton dependencies; added `smallvec`, `rayon`, optional `serde`, and dev-dependencies `sm-testkit`, `proptest`, `criterion` (all in the allowed list). `Cargo.lock` grew (criterion and its transitive dependencies): the orchestrator will need to re-resolve it when merging with other workers.
5. **Intersection result types live in sm-geom** (`SsiResult`, `IntersectionCurve`, `CurveGeometry`, `MarchedCurve`, `CurveSurface`): the contract defines no intersection trait. Downstream crates (webcut) depend on `sm-geom` for them.
6. Parabola and hyperbola have no `CurveDef` variant; they are returned as exact rational quadratic `CurveDef::Nurbs` chains inside the caller's window (see api_change_request 1).

## Known gaps

- **Marching limits** (also in `SsiOptions` rustdoc): components smaller than the seed grid that touch no iso-curve are missed; tangential contacts end a branch as `MarchEnd::Singular` (an isolated tangent point is reported as `SsiResult::Point` only when it is the whole result); branches crossing at a singular point come out as separate pieces; implicit-only against implicit-only pairs are `Unsupported`; a pole of the marching surface ends a branch (a regular surface is preferred as the marching surface); parameters of periodic directions are continuous along a branch, not wrapped.
- Closed forms missing (marched instead): quadric-quadric coaxial / parallel-axis special cases, torus cuts not perpendicular to or through the axis.
- Open conics need a window (`SsiOptions::with_window`); without one the call returns `GeomError::Invalid`. Each stretch of a parabola or hyperbola inside the window is a separate curve (face crossings are solved in closed form, so a small window far from the vertex is not missed).
- A curve running **on** a NURBS surface gives `GeomError::NoConvergence` (coincidence is detected only for planes and implicit surfaces).
- Periodic knot insertion needs more unique poles than the degree (`Unsupported` otherwise).
- Not provided: degree elevation / reduction, knot removal, curve-curve and self-intersection, NURBS-NURBS curve intersection, SIMD / GPU paths. The prepared jet is only about 1.2x faster than the reference (which is already allocation-free); the large wins are position-only evaluation (5.8x), the global projector (4-6x) and batching.
- `eval_many` / `jet_many` assert equal buffer lengths: a caller-owned precondition, not data-reachable input.
- `KernelStore::from(StoreData)` is infallible and silently leaves the prepared layer empty for an entry that fails validation (evaluation then uses the reference evaluator); `KernelStore::from_data` validates.
- Tangent contacts are now decided by a distance compared with `tol.linear` (closed forms) and the marcher's singular threshold is `100 * tol.angular` (1e-7 by default). The 1e-10..1e-12 relative thresholds of the conic reduction that remain only classify the shape (parabola against ellipse, linear against quadratic part), not tangency.
- Curve against a NURBS surface (`pair_hits`): box leaves stop at 1e-3 of the larger bounding diagonal and hits within `20 tol.linear` merge, so two contacts closer than a leaf in one box can come out as one (documented on `curve_surface`); `pair_hits` still deduplicates by scanning earlier hits (quadratic in the hit count, small in practice). `build_hits` merges by parameter within `1e-6 * span`, `trig_vs_implicit` flags the `t = pi` root as non-tangent, and `nurbs_plane` / `trig_vs_implicit` allocate small `Vec`s per segment or call. These are non-blocking and were left.
- The isolated-tangent-point search runs only when the marcher found no curve and the other surface is implicit (not for NURBS against NURBS).
- `insert_knot*` refuse the interval ends (the seam of a periodic direction); documented, not wrapped.
- Not attributable to sm-geom: the contract ray hook used for line-sphere drops a tangent line at a 1e-10 gap (0 hits), so it shares the tolerance-blind tangency behaviour.

## Fix round 0 (review findings)

Fixed, each with a test:

1. **Bezier extraction with an interior knot of multiplicity p + 1.** `Layout::bezier` / `bezierized` assumed `segments * p + 1` poles. A new `Layout::spans` walks the non-empty knot spans (`hi > lo`) and gives each its `p + 1` poles, so a C^-1 joint splits the pole run and empty spans are never emitted. The curve and surface (`decompose`) builders use it. Tests: degree-2 curve `[0,0,0,.5,.5,.5,1,1,1]` against `x = 3.5` (one hit at 0.625; the gap plane `x = 2.5` has none), degree-1 surface with `[0,0,.5,.5,1,1]` (two patches, ray hits on both quads, none in the gap), plus the plane-hit property.
2. **`curve_prepared_surface` for circles and ellipses.** Each remapped hit parameter is replicated with `periodic_in_range(t, TAU, r)`. Differential test against `curve_surface` over `None`, `[-1.5, 1.5]`, `[-pi, pi]`, `[0, 4 pi]`, `[2.5, 9]`, `[-7, -1]`, `[0.3, 0.9]`, for circle and ellipse against a plane patch and a sphere.
3. **Marcher branch jumping.** A step now needs the corrector to stay within `3 chord_tol` of the straight prediction and the corrected midpoint within `chord_tol` of the chord; the step is capped at `max_step * min(1, 4 sin(angle between normals))`. Regression: perpendicular cylinders with `dr` in 1e-2, 1e-3, 1e-4 give two closed loops with no z sign change and every chord midpoint within `chord_tol` of both surfaces (the test fails without the checks: one loop for `dr = 1e-4`). The cost is one extra corrector solve per accepted step.
4. **Tangency by distance.** `sphere_sphere` compares the actual gap (`d - (ra + rb)`, or `|ra - rb| - d` for nesting) with `tol.linear`; `plane_quadric` takes the gap `|q| / |grad q|` at the stationary point of the section. Unit sphere against a plane at `z = 1 + 1e-13 .. 5e-7` (both sides) is a `Point`, `1 + 1e-5` is `Empty`; same for plane-cylinder (tangent line) and sphere-sphere internal and external contact. A NURBS curve touching a plane within tolerance now gives a tangent hit: the Bernstein root finder dropped a minimum that sits exactly at a split point (both monotone halves were discarded); monotone pieces that end within `eps` of zero now vote, and two matching pieces make a tangent root.
5. **Periodic insertion order.** `kn_at` reads the stored window `[p, p + m)` and shifts by the period only outside it; the extended vector is made non-decreasing. Property test with random non-uniform periodic knots, repeated knots and period copies a few ulps off (it reproduces the "knots must be finite and non-decreasing" failure on the old code).
6. **Plane-quadric frame dependence.** The stationary values (`fp`, `gg`) are evaluated on the quadric at world points instead of from the origin-relative expansion, and the degeneracy tests are distances, so the answer no longer depends on the plane origin. Tests: unit sphere against `z = 0.5` with the frame origin at offsets up to 1e6, and a property test re-anchoring the plane origin inside the plane for sphere, cylinder, cone and torus.

Should-fix items: (1) `nurbs_vs_implicit` leaves are `min(20 tol.linear, 1e-4 extent)` and hits merge when within `5 tol.linear` or when the curve stays within `tol.linear` of the surface between them (straight cubic through spheres of radius 3e-4 .. 1e-5: two hits; a 1e-7 sphere is one contact); `pair_hits` limits documented. (2) open conics: one curve per stretch inside the window, break parameters solved in closed form from the six window faces (no sampling). (3) sphere touched by a cylinder gives `Point`. (4) `nurbs_props_general`. (5) `PreparedCurve::length` collects, sorts and dedups cuts, and integrates whole periods once (1e6 periods in bounded time). (6) tangency decisions tied to `tol.linear` / `tol.angular` (see Known gaps for what stays fixed). (7) left, listed under Known gaps. (8) done as 4. (9) empty spans are skipped in `spans`. (10) documented. (11) done here and in DESIGN. (12) recorded under Known gaps.

## api_change_requests

All additive and optional (`#[non_exhaustive]` enums / default methods); nothing blocks sm-geom.

1. **crate** `sm-geom-api`; **what** `CurveDef::Parabola` and `CurveDef::Hyperbola` (and `Curve` kinds behind them); **why** plane sections of cones (and cylinder-cone, etc.) yield them, and webcut would like an exact carrier instead of a NURBS chain; **proposed** `Parabola { frame: Frame, focal: f64 }` (`o + t x + t^2/(4 f) y`, domain the real line) and `Hyperbola { frame: Frame, a: f64, b: f64 }` (`o + a cosh t x + b sinh t y`, one branch), with projection and `for_each_value` tags 5 and 6.
2. **crate** `sm-geom-api`; **what** batch evaluation hooks on the traits; **why** schemes call through `&dyn` and a per-point virtual call defeats the batched / parallel paths; **proposed** default methods `fn eval_many(&self, uv: &[Uv], out: &mut [Point3])` on `ParametricSurface` and `fn eval_many(&self, t: &[f64], out: &mut [Point3])` on `ParametricCurve` (default loops; sm-geom overrides with the rayon versions it already has).
3. **crate** `sm-geom-api`; **what** `PCurve::d2` (second derivative); **why** `CurveOnSurface` needs `d2` for curvature of coedge carriers and the contract's `PCurve` only has `eval` / `d1`; **proposed** `pub fn d2(&self, t: f64) -> Uv` (zero for `Line2` and polylines, de Boor for `Nurbs2`, which sm-geom's embedded 3D evaluator already computes).
4. **crate** `sm-geom-api` (or `sm-math`, coordination); **what** make `real_roots` / `solve_quadratic` public (or move to `sm-math`); **why** two copies now exist (api internal, sm-geom `roots`); **proposed** `sm_math::roots::{poly_roots(&[f64], lo, hi) -> Vec<Root>, brent, quadratic}` with the same `Root { t, double }` semantics.
5. **crate** `sm-geom-api`; **what** an intersection trait or shared result types if non-sm-geom crates must call intersections without depending on `sm-geom`; **why** see deviation 5; **proposed** none now (consumers can depend on `sm-geom`).

## Commit

Code complete at `9848cddb909961d9b81ffe7dfea54b4a1b991ce9`; this report fix is one commit on top of it (the HEAD of the branch, which is the merge target).

# sm-geom: design

Implements the frozen `sm-geom-api` contract. Depends on `sm-core` and `sm-geom-api`, plus `smallvec` and `rayon`; `serde` is an optional feature that forwards to the contract's.

## Positioning

Similar to OpenCASCADE's `Geom` / `GeomAPI` / `IntPatch` / `Extrema` layers (and the curve and surface kernel under ACIS), sm-geom evaluates analytic and NURBS geometry, projects points, and intersects curves and surfaces. It differs in four ways: (1) analytic kinds are closed data resolved to the contract's exact closed-form evaluators instead of a virtual class hierarchy, and NURBS are *compiled* once into homogeneous-pole form with a cached Bezier decomposition; (2) every projection and intersection is a global search (Bezier-box branch and bound, then safeguarded Newton, or exact root isolation) rather than a local iteration from a guessed seed; (3) surface-surface intersection returns an **exact** curve (line, circle, ellipse, or rational NURBS for parabola and hyperbola branches) for every plane-quadric pair and the symmetric torus cuts, and marches only for the genuinely general cases; (4) all output is bit-identical for any thread count, by construction (no hash-order dependence, order-preserving parallel maps, no reductions across threads). That gives the webcut and meshing pipeline exact, reproducible edge carriers, a 6x faster NURBS projector and a 5.8x faster position-only evaluator than the contract's reference, and results that never depend on how many threads ran.

## What lives where

| Module | Contents |
|---|---|
| `nurbs::basis` | `find_span`, basis values (A2.2) and first two derivatives (A2.3), const-generic scratch (`4`, `8`, `16` wide by degree). |
| `nurbs::spline1` | 1D algorithms on homogeneous poles `[wx, wy, wz, w]`: Boehm insertion, **periodic** insertion, clamping, Bezier extraction, allocation-free de Casteljau halving. |
| `PreparedCurve`, `PreparedSurface` | NURBS compiled for the hot path; implement `ParametricCurve` / `ParametricSurface`. |
| `exact` | Exact rational NURBS of circle, ellipse, arcs, cylinder, cone, sphere, torus, plane patch, conic arc (P&T ch. 7 and 8). |
| `CurveOnSurface` | `S(pcurve(t))` with derivatives to second order by the chain rule. |
| `KernelStore` | `GeomStore` / `GeomAppend` / `GeomStoreMut` with prepared NURBS behind the trait objects. |
| `intersect` | `curve_plane`, `curve_surface`, `curve_prepared_surface`, `intersect_surfaces`; internals `bernstein`, `interval`, `conic`, `march`. |
| `fit` | Interpolating cubic NURBS through points (banded solve), used by `MarchedCurve::to_nurbs`. |
| `roots` (private) | Brent, stable quadratic, polynomial roots with a double-root flag, dense `N x N` solve. `sm-math` was empty when W1 started; this should migrate there. |

### Analytic kinds

The contract crate already provides exact, allocation-free closed-form implementations of every analytic curve and surface (line, circle, ellipse, plane, cylinder, cone, sphere, torus, elliptic cylinder and cone, quadrics, torus quartic) with analytic projections and ray hooks. Rust's orphan rule forbids implementing those traits again on those types, and a parallel set of newtypes would only duplicate tested code, so `KernelStore` resolves analytic handles to the contract's evaluators (`SurfaceDef::as_surface`) and adds value where the reference is thin: NURBS, intersection, batching and the store. See "Deviations" in `W1_REPORT.md`.

### NURBS

`PreparedCurve` / `PreparedSurface` keep the knot vectors and one contiguous array of homogeneous poles (array of four-wide poles: every basis term reads all four components, so AoS-of-homogeneous beats separate coordinate arrays here). Evaluation is allocation-free: the basis scratch is sized by degree class, `eval` skips the derivative bases, and `jet` uses the quotient rule exactly like the reference (agreement to 1e-12 on the property tests). Batch calls (`eval_many`, `jet_many`, `project_many`) use `rayon` with element-independent writes or an order-preserving collect, so the output is the same for any thread count.

Editing is exact and shape-preserving: `insert_knot` (multiplicity checked against the degree), `refine`, `clamped`, `split`, `bezier_segments`. **Periodic** curves and surfaces (the contract's unclamped-knot form) keep their periodicity through insertion: the knot and all its period copies are inserted by the Boehm formula on wrapped indices, and the canonical window of `unique poles + 1` new poles is read back with the first `degree` poles repeated (requires more unique poles than the degree). `clamped()` cuts a periodic direction at its seam.

Closest point is branch and bound over the cached Bezier pieces: the lower bound is the distance to the box of the Cartesian control polygon (convex-hull property, positive weights), pieces are expanded best-first with a deterministic tie-break, an early Newton polish tightens the upper bound, and a piece whose box is small against its distance from the query finishes with one Newton run from the closest of a small sample grid. Newton is safeguarded (steps halved until the distance or the stationarity residual falls) and, on surfaces, keeps an active set at parameter bounds. Global optimality is checked against dense scans in the property tests.

### Intersections

- **Curve-plane**: closed forms for line, circle, ellipse; for NURBS the signed distance is a polynomial in Bernstein form on every Bezier piece (homogeneous coordinates), and its roots are isolated by variation-diminishing subdivision plus Brent. Knot-coincident crossings and grazing contacts are handled and flagged `tangent`.
- **Curve-surface**: lines use the contract's ray hook (closed forms, `Crossing::Tangent` flags); circles and ellipses against quadrics and the torus use the exact trigonometric restriction (16 samples give the harmonics, `x = tan(t/2)` turns them into a polynomial); NURBS curves against implicit surfaces use Bezier subdivision with interval bounds of the quadric or quartic over the control box, then Newton; anything against a NURBS surface uses pairwise box subdivision of curve pieces and patches and a damped (Levenberg-Marquardt) 3x3 Newton. Cone nappes are filtered. The ray hook of `PreparedSurface` is built on the same machinery.
- **Surface-surface, closed forms**: plane-plane (a line), plane against **every quadric** by restriction to the plane and classification of the conic (ellipse, circle, parabola, hyperbola, line pairs, point, empty, with the cone's single nappe and rays from the apex), sphere-sphere, and the torus cut perpendicular to or through its axis. Open conics become rational quadratic NURBS chains exact to rounding (hyperbola arcs have weight `cosh(dt/2)`; the parabola is one Bezier piece) inside the caller's window.
- **Surface-surface, general**: marching. Seeds are the hits of iso-curves of the marching surface `M` with the other surface `T` (iso-curves are exact lines, circles and ellipses for analytic kinds and exact NURBS curves for NURBS), so seeding reuses the curve-surface machinery. A branch is traced in both directions by a predictor along `n_M x n_T` and a Newton corrector with a step-length constraint, in two unknowns (`T` implicit) or four (`T` parametric). Step size follows the turning angle against `chord_tol`; closure, window exit and parameter-boundary exit are detected; pieces cut by the seam of a closed non-periodic NURBS are re-joined. A first-order extension of the surface beyond a bounded range lets Newton cross the boundary smoothly.

### Determinism

No `HashMap`/`HashSet`; the spatial index of traced curves is a `BTreeMap`; heaps break ties on an insertion counter; parallel maps preserve order and write disjoint elements; there are no cross-thread reductions. `tests/determinism.rs` runs a workload (batch evaluation and projection, marched and closed-form intersections, store use) repeatedly, with 1 and 4 workers and from concurrent callers, and compares bit hashes (`sm_testkit::determinism::check_f64_bits`); `RAYON_NUM_THREADS=1 cargo test` is also green.

### Tolerance and units

All tolerances come from `ModelTolerance` (`linear` is the position accuracy of marched points and the coincidence threshold; `angular` decides parallel planes/lines). Root finding is relative; the documented absolute choices are the 1e-10..1e-12 classification thresholds of conic reduction and the 1e-7 (radian) tangency threshold of the marcher.

## Performance (criterion, release, one machine; see `benches/hot_path.rs`)

| Hot path | reference (`sm-geom-api`) | sm-geom |
|---|---|---|
| bicubic rational surface `jet`, 10k | 3.6-4.3 ms | 3.0-3.5 ms (1.2x) |
| same, position only `eval`, 10k | 3.9 ms | 0.68 ms (5.8x) |
| same, `eval_many` (rayon), 10k | - | 0.32 ms |
| closest point to a 12x12 bicubic net, 64 queries | 236-276 ms | 39-62 ms (4-6x, and globally optimal) |
| closest point to a NURBS circle, 64 queries | - | 2 ms |
| plane-sphere (closed form) | - | 160 ns |
| cylinder-cylinder, marched (two loops) | - | 0.4 ms |

The jet gain is modest because the reference evaluator is already allocation-free; the large wins are the value-only path, the global projector and batching.

## Limits (documented, tested where possible)

See `SsiOptions` for the marcher: loops smaller than the seed grid that touch no iso-curve are missed; tangential contacts stop a branch (`MarchEnd::Singular`) instead of being continued; branches crossing at a singular point (Steinmetz) come out as separate pieces; implicit-only against implicit-only pairs are `Unsupported`; a pole of the marching surface ends a branch (the engine prefers a regular surface as `M`). Open conics need a window. A curve lying on a NURBS surface reports `NoConvergence`. Periodic knot insertion needs more unique poles than the degree. Degree elevation, knot removal and curve-curve intersection are not provided.

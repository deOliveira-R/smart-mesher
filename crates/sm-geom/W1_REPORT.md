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

`cargo test -p sm-geom`: 56 tests in 9 binaries, all passing (also with `RAYON_NUM_THREADS=1`); 3 proptest suites use the testkit's fixed seed (`config(64)` / `config(48)`).

| Binary | Tests | What |
|---|---|---|
| lib (unit) | 25 | roots (double roots, Brent), basis (partition of unity, FD), knot ops (open and periodic insertion, clamping, Bezier, de Casteljau), prepared curve/surface vs the reference evaluator, periodic surface editing and projection, periodic arc length, exact circle / ellipse / cylinder / sphere / torus / cone (implicit residual, curvature `K`, `H`, area by quadrature, length), pcurve helix, Bernstein roots, fit |
| `curve_surface` | 6 | line / circle / ellipse / NURBS against planes; line, circle, ellipse, NURBS against sphere, cylinder, torus, cone nappe; NURBS circle against NURBS sphere; line against NURBS torus (hand values) |
| `ssi_closed` | 5 | plane-plane / sphere / cylinder (circle, 45 deg ellipse `a = r sqrt 2`, lines, tangent) / cone (all conics, nappe, rays) / torus cuts / sphere-sphere / elliptic cylinder / ellipsoid |
| `ssi_march` | 6 | perpendicular cylinders (two closed loops, length vs quadrature 2e-5), Steinmetz (coverage of the exact ellipses), oblique plane-torus (closed loops), sphere-cylinder, NURBS sphere-plane (length vs `2 pi rho`), NURBS-NURBS pair, determinism |
| `ssi_props` | 3 | frame independence under random rigid / mirror transforms (testkit strategies) |
| `nurbs_props` | 3 | random NURBS: prepared == reference, insertion / split keep shape, projection globally optimal vs dense scans |
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
- Open conics need a window (`SsiOptions::with_window`); without one the call returns `GeomError::Invalid`.
- A curve running **on** a NURBS surface gives `GeomError::NoConvergence` (coincidence is detected only for planes and implicit surfaces).
- Periodic knot insertion needs more unique poles than the degree (`Unsupported` otherwise).
- Not provided: degree elevation / reduction, knot removal, curve-curve and self-intersection, NURBS-NURBS curve intersection, SIMD / GPU paths. The prepared jet is only about 1.2x faster than the reference (which is already allocation-free); the large wins are position-only evaluation (5.8x), the global projector (4-6x) and batching.
- `eval_many` / `jet_many` assert equal buffer lengths: a caller-owned precondition, not data-reachable input.
- `KernelStore::from(StoreData)` is infallible and silently leaves the prepared layer empty for an entry that fails validation (evaluation then uses the reference evaluator); `KernelStore::from_data` validates.
- Tolerance constants inside conic classification (1e-10..1e-12 relative) and the marcher's 1e-7 rad tangency threshold are fixed, not taken from `ModelTolerance::angular`.

## api_change_requests

All additive and optional (`#[non_exhaustive]` enums / default methods); nothing blocks sm-geom.

1. **crate** `sm-geom-api`; **what** `CurveDef::Parabola` and `CurveDef::Hyperbola` (and `Curve` kinds behind them); **why** plane sections of cones (and cylinder-cone, etc.) yield them, and webcut would like an exact carrier instead of a NURBS chain; **proposed** `Parabola { frame: Frame, focal: f64 }` (`o + t x + t^2/(4 f) y`, domain the real line) and `Hyperbola { frame: Frame, a: f64, b: f64 }` (`o + a cosh t x + b sinh t y`, one branch), with projection and `for_each_value` tags 5 and 6.
2. **crate** `sm-geom-api`; **what** batch evaluation hooks on the traits; **why** schemes call through `&dyn` and a per-point virtual call defeats the batched / parallel paths; **proposed** default methods `fn eval_many(&self, uv: &[Uv], out: &mut [Point3])` on `ParametricSurface` and `fn eval_many(&self, t: &[f64], out: &mut [Point3])` on `ParametricCurve` (default loops; sm-geom overrides with the rayon versions it already has).
3. **crate** `sm-geom-api`; **what** `PCurve::d2` (second derivative); **why** `CurveOnSurface` needs `d2` for curvature of coedge carriers and the contract's `PCurve` only has `eval` / `d1`; **proposed** `pub fn d2(&self, t: f64) -> Uv` (zero for `Line2` and polylines, de Boor for `Nurbs2`, which sm-geom's embedded 3D evaluator already computes).
4. **crate** `sm-geom-api` (or `sm-math`, coordination); **what** make `real_roots` / `solve_quadratic` public (or move to `sm-math`); **why** two copies now exist (api internal, sm-geom `roots`); **proposed** `sm_math::roots::{poly_roots(&[f64], lo, hi) -> Vec<Root>, brent, quadratic}` with the same `Root { t, double }` semantics.
5. **crate** `sm-geom-api`; **what** an intersection trait or shared result types if non-sm-geom crates must call intersections without depending on `sm-geom`; **why** see deviation 5; **proposed** none now (consumers can depend on `sm-geom`).

## Commit

Code complete at `9848cddb909961d9b81ffe7dfea54b4a1b991ce9`; this report fix is one commit on top of it (the HEAD of the branch, which is the merge target).

# sm-geom-api: design (candidate "cellcomplex")

Exact-shape contract. Depends on `sm-core` (points, vectors, `EntityId`), `thiserror`, `smallvec`; `serde` behind a feature.

## Model

Shape is split into **three things that topology never owns**:

1. **Traits** that evaluate geometry: `ParametricCurve`, `ParametricSurface` (object-safe, `&dyn` for schemes).
2. **Data** that describes geometry: analytic structs (`Line`, `Circle`, `Ellipse`, `Plane`, `Cylinder`, `Cone`, `Sphere`, `Torus`), `NurbsCurve`/`NurbsSurface` (validated data only), and `CurveData`/`SurfaceData` enums for persistence.
3. **A handle resolver**, `GeometryStore` (the GMI-like seam). Topology holds `EntityId` handles (`Curve`/`Surface` kinds); the store turns a handle into `&dyn`. `MemoryStore` is the deterministic reference store.

CSG comes in through `SignedSurface` (implicit `f(p)`, gradient, ray hits), with `Quadric` (10-coefficient general quadric), `TorusImplicit` (quartic), `ImplicitForm` (a `Copy` enum of the two) and `HalfSpace { surface, sign }`. Every analytic surface returns its implicit form from `ParametricSurface::implicit()`, so one object serves B-rep and CSG.

## Key types

| Type | Role |
|---|---|
| `Domain1 { range, period }` | Parameter domain; `period = Some(p)` means periodic, and the interval ends are the **seam**. `wrap`, `nearest_to` pick representatives. |
| `Domain2 { u, v, singularities }` | Surface domain. `Singularity { along, at, point }` marks an iso-line that collapses to a point (sphere poles, cone apex). |
| `CurveJet`, `SurfaceJet` | Position + derivatives up to order 2 in one call (one evaluation per sample in hot loops). |
| `Ray`, `RayHit` | Tracking hook types. `RayHit { t, point, normal }`. |
| `Tolerance { linear, angular, parametric }` | The tolerance policy value shared with topology. |
| `Frame` | Right-handed orthonormal placement used by all analytic kinds. |
| `GeomError` | `OutOfDomain`, `Singular`, `NoConvergence`, `Unsupported`, `Invalid` (`#[non_exhaustive]`). |

## Conventions (binding)

- Surface normal is `du x dv`, normalized. For all analytic kinds this is outward (sphere, cylinder, cone, torus) or `+z` (plane) and equals the implicit gradient direction (tested). Face orientation is topology's `Sense` on its carrier, not the surface's business.
- Evaluation is infallible and continues analytically outside the domain; periodic kinds wrap. Failure-capable operations (`normal`, `project`, `ray_hits`) return `Result`.
- At a pole, `normal()` of analytic kinds returns the exact limit normal for the given `uv`; the trait default returns `Singular` when `|du x dv|` is below `1e-12 * scale^2`.
- `project` returns the closest point and may return `Unsupported` (callers fall back to a numeric projector from `sm-geom`). Non-unique minima (a point on a circle's axis, a sphere's centre) are handled deterministically: circles/cylinders/cone/torus pick `u = 0`; a sphere centre is `Singular`.
- Ray hook: `ParametricSurface::ray_hits(&Ray, Interval, &mut Vec<RayHit>)` appends hits in ascending `t` to a caller-owned buffer (no per-call allocation). Default goes through `implicit()`; implementors without one return `Unsupported` and the tracker facets that surface (NURBS). The cone overrides it to keep only one nappe.
- Arc length: curves provide `length(a, b)`; default is 32-panel 5-point Gauss-Legendre of `|C'|`, overridden exactly for line and circle.

## Requirement map

| Requirement | Where |
|---|---|
| ParametricCurve: eval, derivatives, param range, periodicity, projection, length | `curve.rs`; `jet`, `domain().period`, `project`, `length` |
| ParametricSurface: eval, 1st/2nd derivatives, normal, domain with seams and poles, inverse projection | `surface.rs`, `domain.rs`; `jet`, `normal`, `Domain2::singularities`, `project` |
| Analytic kinds | `analytic.rs`, all eight implement both traits' evaluation; closed-form projection for line, circle, plane, cylinder, cone, sphere, torus |
| NURBS as data | `nurbs.rs`: knot/weight/multiplicity validation; evaluation intentionally lives in `sm-geom` |
| CSG quadric half-spaces incl. torus/general quadric | `implicit.rs`: `Quadric::{from_local, plane, sphere, cylinder, cone}`, `TorusImplicit`, `HalfSpace` |
| Ray/surface hook for tracking | `RayHit`, `ParametricSurface::ray_hits`, `SignedSurface::ray_hits` (exact quadratic and quartic) |
| Tolerance policy type | `tol.rs` |
| Object safety | every trait is object-safe; tests use `Box<dyn ParametricCurve/Surface>` and `&dyn GeometryStore` |

## Tests (analytic, not smoke)

Finite-difference check of every first/second derivative of all curves and surfaces; closed-form values (circle, cylinder seam, sphere pole normal, cone apex); projection minimality against dense sampling plus normal-residual orthogonality; ray hits against hand values (sphere `t = 8, 12`; torus `t = 6, 8, 12, 14`; tangent torus contacts; one-nappe cone); implicit/parametric agreement (value, gradient direction equals the surface normal); polynomial root isolation; NURBS validation.

## Tradeoffs and open issues

- **Ellipse projection** is `Unsupported` (needs a quartic or guarded Newton). **Torus is ring-only** (`R > r`). **Cone is one nappe** parametrically, the double cone implicitly (the cone's ray hook filters).
- **NURBS cannot be a `&dyn ParametricCurve` here.** `CurveData::into_dyn` returns `Unsupported` for it; `sm-geom` supplies the evaluator. This keeps the API crate free of the largest numerical code.
- **Root finder** (`real_roots`) isolates roots with derivative critical points; tangential contacts are accepted when the critical value is within `1e-12` relative. Very close double roots can be missed or reported as one.
- `GeometryStore` is read-only and returns `Option<&dyn _>`; allocation of new handles is the store implementor's job (`MemoryStore` allocates sequentially per kind, so insertion order gives ids).
- `Tolerance` is a plain value; the per-entity override lives in topology (see sm-topo-api DESIGN).
- No `Hash`/`Eq` on f64-bearing types by design; content hashing happens in topology by bit pattern.

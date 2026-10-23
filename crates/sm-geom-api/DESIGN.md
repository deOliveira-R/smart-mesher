# sm-geom-api: design (candidate "occt-like")

Exact-shape contract. Holds no topology and no mesh. Everything topology needs from geometry is "evaluate, project, intersect a ray, tell me the period and the poles".

## Model

Shapes are **data** (`CurveDef`, `SurfaceDef`: closed enums, serde-able behind the `serde` feature) that implement **object-safe traits**. A `GeomStore` maps typed handles (`CurveId`, `SurfaceId`, wrapping `EntityKind::Curve`/`Surface` ids) to the data. Topology stores only handles, so one surface is shared by any number of faces (OCCT: shared `Geom_Surface` handles) and a shape never knows who uses it.

| Piece | What |
|---|---|
| `ParametricCurve` | `range`, `period`, `eval`, `d1`, `d2`, `derivs`, `closest_in(p, interval)`, `length(a, b, tol)`, `is_closed`. |
| `ParametricSurface` | `domain`, `period_u/v`, `singularities`, `eval`, `d1`, `d2`, `derivs`, `normal`, `project(p, hint)`. |
| Analytic | `Line`, `Circle`, `Ellipse`; `Plane`, `Cylinder`, `Cone`, `Sphere`, `Torus` (OCCT parametrizations; `u` is the angle about the frame z axis). |
| NURBS | `NurbsCurve`, `NurbsSurface` are plain data (degree, knots, poles, optional weights) with `validate()` and a reference evaluator (Piegl and Tiller A2.1/A2.3, rational derivatives to order 2, no heap allocation, degree <= 15). Non-periodic only. |
| CSG | `ImplicitSurface` (`value`, `gradient`, `side(p, tol)`), `Quadric` (`p^T A p + b.p + c`, constructors for plane/sphere/cylinder/cone/MCNP `GQ`), `ImplicitTorus` (quartic), `ImplicitDef`, `HalfSpace`. Negative = inside. |
| Tracking hook | `RayIntersect::intersect_ray(&Ray, t_range, &TolerancePolicy, &mut SmallVec<[RayHit;4]>)`. Exact for every analytic and CSG surface (quadratic or quartic via `poly_roots_in`); NURBS returns `GeomError::Unsupported` so the tracker falls back to facets (ir.md §5.2). |
| Tolerance | `TolerancePolicy { linear_min, linear_max, angular, parametric_min }` plus `coincident`, `grown`, `within_cap`, `param_tol`. |
| Seam support | `PCurve` / `Line2`: a curve in `(u, v)` space, same parameter as the edge curve. `unwrap_near` / `wrap_into` keep parameters continuous over a periodic seam. |

## How each requirement is met

- **Periodic seams.** `period_u/v` plus `domain` ending at the period. `project` normalizes into the domain; the caller picks the branch with `unwrap_near`. Topology puts the seam edge in the loop twice (pcurves at `u = 0` and `u = 2 pi`).
- **Singular poles.** `singularities()` returns collapsed parameter lines (`SingularLocus::V(±pi/2)` for the sphere, apex for the cone, degenerate pole rows for NURBS). `normal` returns `GeomError::Singular` there, except the sphere, which is defined by continuity. `project` takes a `hint` so the longitude at a pole is the caller's choice, not an accident.
- **Orientation.** The normal is `S_u x S_v`. An improper transform (mirror) gives a left-handed `Frame`, so the normal flips with it. Topology compensates through `Location::flips_orientation`. This is deliberate: geometry never silently re-orients; the one orientation decision lives in topology (`surface_sense`, `FaceUse`, `Location`). The *implicit* form always stays positive-outside.
- **Object safety.** All traits are object-safe (`&dyn ParametricCurve`, `&dyn ParametricSurface`, `&dyn ImplicitSurface`, `&dyn RayIntersect`, `&dyn GeomStore`); a test builds each from a store. Generic helpers (`closest_generic`, `project_generic`, `integrate_speed`) take `?Sized`.
- **Determinism.** No randomness, no threads, no hash maps. Samplers use fixed grids, ties pick the smaller parameter, roots are sorted. A test hashes `f64` bits of a battery of evaluations and projections twice.
- **Numerics tested against analytic results.** Points on the implicit form, normals parallel to the implicit gradient, finite-difference checks of `d1`/`d2` for every surface kind, projection round trips, circle and ellipse length (`9.688448220547672` for `a=2, b=1`), a rational quarter-arc NURBS that is exactly the circle, ray hits on sphere/cylinder/cone/torus with known `t`.

## Tradeoffs

- **Enums over open trait objects for storage.** Files, hashes and lineage need a closed, serializable set. The traits stay open for kernels that add kinds (`#[non_exhaustive]`).
- **Analytic closest-point where it is closed form** (line, circle, plane, cylinder, cone, sphere, torus), generic seeded Newton for ellipse and NURBS. Generic projection needs a finite domain and can miss a far-away global minimum on wildly oscillating NURBS; it is a reference, not a robust kernel.
- **Ring torus only** (`R > r`). Spindle/lemon tori are rejected.
- **Cone uses OCCT's parametrization** (`R` at `v = 0`, apex at `v = -R/sin a`): the double cone is one analytic surface, and both nappes are reachable.
- **Tangent ray contacts** are flagged `grazing` (vanishing slope or two roots within `linear_min`); a missed double root is possible only for degree >= 3 with a tangency not at a derivative root within `1e-12`.
- **Transforms** accept similarities only for analytic kinds (`NotSimilarity` otherwise); NURBS accept any affine map. `Quadric::transformed` handles a similarity including mirrors.

## Not here

Curve/curve and surface/surface intersection (`geom-intersect`), tessellation, offset surfaces, extrusion/revolution surfaces, B-spline periodicity, trimmed surfaces. The CSG cell algebra (union/intersection of half-spaces, universes) belongs to a CSG crate; this crate provides its shape providers (`HalfSpace`, `SurfaceDef::to_implicit`).

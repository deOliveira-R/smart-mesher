# sm-geom-api: exact-shape contract (candidate: radial-edge panel)

Status: candidate for review. Dependencies: `sm-core`, `thiserror`, `smallvec`, optional `serde`.
Tests: `cargo test -p sm-geom-api` (17 tests, all against analytic results or exact integer references).

## Model

Geometry is **exact and untrimmed**. Trimming (which part of a surface is a face) lives in topology.
Topology refers to geometry only by `EntityId` handles (`Curve`, `Surface` kinds), resolved through a
`ShapeStore`. Many entities may share a handle, handles are never reused for different geometry.

```
ParametricCurve   ParametricSurface   ImplicitSurface   RayIntersect   ShapeStore
  (dyn-safe)        (dyn-safe)          (dyn-safe)        (dyn-safe)    (dyn-safe)
        \               |                  |                /
         +---- analytic kinds (Copy data, implement all four) ---- Quadric (implicit + ray)
                NURBS = data only (validate), evaluation lives in sm-geom
```

## Key types

| Type | Role |
|---|---|
| `ParametricCurve` | `eval`, `d1`, `d2`, `tangent`, `domain`, `period`, `project` (closest point, with hint and periodic branch), `length`, `kind`, `analytic()` |
| `ParametricSurface` | `eval(u,v)`, `d1 -> (Su,Sv)`, `d2 -> {Suu,Suv,Svv}`, `normal` (limit at poles), `project` (inverse, returns wrapped `(u,v)`), `principal_curvatures`, `wrap`, `poles`, `domain`, `ray_hook` |
| `SurfaceDomain` | `u`, `v` ranges plus `u_period`/`v_period`. A periodic direction has range length == period, so the two boundary iso-curves are the **seam** |
| `Pole`, `DomainEdge` | A whole domain edge collapsing to a point (sphere poles, cone apex). `Su` vanishes; `normal` returns the limit; the default `normal` returns `Singular` |
| `Line, Circle, Ellipse` / `Plane, Cylinder, Cone, Sphere, Torus` | Analytic kinds with closed-form eval, derivatives, projection. `AnalyticCurve`/`AnalyticSurface` are the closed enums |
| `NurbsCurve`, `NurbsSurface` | Data (degree, knots, control, weights) and `validate()` |
| `ImplicitSurface`, `Quadric`, `HalfSpace` | CSG shape providers. Negative inside. `Quadric` is the general second-degree surface with `transformed(&Transform)` (improper transforms allowed). The torus quartic is `ImplicitSurface for Torus` |
| `Ray`, `RayHit`, `RayIntersect`, `NormalCrossing` | Ray/surface hook for tracking. Hits ascending in `t`; double root reported once as `Tangent`; hit carries unit surface normal, optional `(u,v)`, and `AlongNormal/AgainstNormal` so the tracker maps it to a face side without a tree search |
| `ModelTolerance` | The tolerance policy (below) |
| `Sense` | `Forward/Reversed`, shared by geometry and topology |
| `PCurve` | Parameter-space curve for coedges (seams, poles, trimmed periodic faces) |
| `orient2d`, `orient3d`, `Sign` | Exact predicates: float filter, fallback to expansion arithmetic (`two_sum`/`two_prod`). Reference implementation; `sm-math` may replace it behind the same signatures |
| `ShapeStore`, `ShapeLibrary` | Handle resolution; reference store |
| `Frame` | Right-handed orthonormal frame with deterministic x-axis choice |

## Conventions (all documented on the types)

| Surface | `S(u,v)` | Domain |
|---|---|---|
| Plane | `o + u x + v y` | R x R |
| Cylinder | `(R cos u, R sin u, v)` | u in [0,2pi] periodic, v in R |
| Cone (single nappe) | `(v k cos u, v k sin u, v)`, `k = tan(alpha)` | u periodic, v in [0, inf), apex pole at v = 0 |
| Sphere | `R (cos v cos u, cos v sin u, sin v)` | u periodic, v in [-pi/2, pi/2], two poles |
| Torus | `((R + r cos v) cos u, (R + r cos v) sin u, r sin v)` | both periodic |

Natural normal = `Su x Sv`, always outward for the closed kinds. The implicit forms use the same
orientation (gradient points the same way), so parametric and CSG views agree. The cone's *implicit*
form is the double cone while its *parametric/ray* view is one nappe; ray hits on the lower nappe are
filtered out of the parametric hook only.

## Tolerance policy

`ModelTolerance { resolution, angular, param_rel }`: one global value per model, chosen at project
creation or import (`for_extent`). It decides identity (same point, parallel directions), and
`hash_quantum() = resolution / 100` quantizes coordinates for cross-platform state hashes.
There is no per-entity tolerance and no op-scoped tolerance. Rationale in `sm-topo-api/DESIGN.md`.

## Requirement coverage

| Requirement | Where |
|---|---|
| Curve: eval, derivatives, param range, periodicity, closest point, length | `ParametricCurve` |
| Surface: eval, 1st/2nd derivatives, normal, domain, periodic seams, poles, inverse projection | `ParametricSurface`, `SurfaceDomain`, `Pole`, `project` |
| Analytic kinds | `analytic.rs`, `analytic_surf.rs` |
| NURBS as data | `nurbs.rs` |
| CSG quadric half-spaces incl. torus and general quadric | `csg.rs` (`Quadric`, `HalfSpace`), `Torus: ImplicitSurface` |
| Ray/surface hook for tracking | `ray.rs`, `ParametricSurface::ray_hook`, `ImplicitSurface::ray_hook` |
| Tolerance policy type | `tol.rs` |
| Object safe | `objects_are_dyn_compatible` test; all traits take `&self`, no generics, `Send + Sync` |
| Exact predicates | `predicates.rs` (verified against `i128` determinants) |
| Determinism | No randomness, fixed seed grids in Newton solvers, ordered roots; `deterministic_repeat` test. No rayon in this crate, so thread count cannot matter |

## Tradeoffs

- **Untrimmed shapes, trimming in topology.** Keeps geometry stateless and shareable; costs an extra
  point-in-face test in the tracker (done in topology, not here).
- **Closed enums for analytic kinds plus open traits.** Importers produce `Copy` data that serializes
  and hashes trivially; extension kinds (offset, procedural) implement the trait and use
  `project_by_newton` for the default projection.
- **Cone is one nappe**, so its apex is a boundary pole and every parametrization has a rectangular
  domain. A double cone is two faces' worth of surface (or the implicit form).
- **Roots by derivative bracketing** (`poly_roots`) is deterministic and needs no complex arithmetic,
  but can miss a tangential quartic root (torus grazing); quadric paths report tangency explicitly.
- **Newton projection** is seeded from a fixed grid, so it is deterministic but not guaranteed global
  for exotic surfaces; analytic kinds use closed forms.
- **Predicates here are a reference**; they allocate on the exact path only (rare).

## Open issues

- NURBS evaluation, surface/surface and curve/surface intersection are `sm-geom` work (`geom-intersect`).
- No offset/swept/procedural kinds yet; `CurveKind::Other`/`SurfaceKind::Other` reserve the slot.
- Torus grazing rays can be missed by the quartic bracketing; needs a tangency pass in `sm-geom`.

# sm-geom-api: final design (frozen after W0c)

Exact-shape contract. Depends on `sm-core` (points, vectors, `EntityId`), `thiserror`, `smallvec`; `serde` behind the `serde` feature (which also turns on `sm-core/serde` and `smallvec/serde`). Base: panel candidate `cellcomplex`, with the grafts and must-fix items of `docs/design/w0/geomtopo-recommendation.md` and the W0c decisions in `docs/design/w0/decisions.md`.

## Model

Shape is split into **three things that topology never owns**:

1. **Traits** that evaluate geometry (object-safe, `&dyn` for schemes): `ParametricCurve`, `ParametricSurface`, `ImplicitSurface`.
2. **Closed, serializable data** that describes geometry: `CurveDef { Line, Circle, Ellipse, Nurbs }`, `SurfaceDef { Plane, Cylinder, Cone, Sphere, Torus, Nurbs, Implicit(ImplicitDef), EllipticCylinder, EllipticCone }`, `ImplicitDef { Quadric, Torus(ImplicitTorus) }`, `PCurve { Line2, Polyline2, Nurbs2 }`. The defs are `#[non_exhaustive]` enums, so adding a kind later is not a breaking change for readers that have a wildcard arm.
3. **A handle resolver**: `GeomStore` (read) and `GeomStoreMut` (`GeomAppend`: add; `GeomStoreMut`: add + replace; `MemoryStore::truncate` is an inherent method, not on a trait). Topology holds `EntityId` handles (`Curve`, `Surface` kinds). `MemoryStore` is the deterministic reference store: it stores the defs themselves (no `Box<dyn>`), so it clones, compares and serializes, and it caches `Option<ImplicitDef>` per surface so `implicit(id)` can return a borrow. `curve(id)` / `surface(id)` return `&dyn` straight from the stored def. An implicit-only surface (`SurfaceDef::Implicit`: a general quadric, the torus quartic) has no parametrization: `surface(id)` is `None`, `implicit(id)` is `Some`. This is what lets a `HalfSpace` reference a general quadric.

CSG: `HalfSpace { surface, sign }`, `CsgCell { halfspaces }` (an intersection) with `contains(store, p, tol) -> Containment { Inside, On, Outside }` and `ray_hits(store, ray, range, out)`. Cell hits carry the **cell-outward normal** and a `Crossing` relative to it, and an edge or corner hit is reported once. Every analytic surface returns its implicit form from `ParametricSurface::implicit()`, so one object serves the boundary representation, the CSG form and the default ray hook.

## Key types

| Type | Role |
|---|---|
| `Domain1 { range, period }` | Parameter domain; `period = Some(p)` means periodic and the interval ends are the **seam**. `wrap`, `nearest_to` pick representatives (used to unwrap parameters along a loop). |
| `Domain2 { u, v, singularities }` | Surface domain. `Singularity { along, at, point }` marks an iso-line that collapses to a point (sphere poles, cone apex); `normal()` of an analytic kind returns the exact limit normal there. |
| `CurveJet`, `SurfaceJet` | Position and derivatives up to order 2 in one call. |
| `Ray`, `RayHit { t, point, normal, uv: Option<Uv>, crossing }` | Tracking hook. `Crossing { AlongNormal, AgainstNormal, Tangent }`; a double root is reported once, as `Tangent`. |
| `ModelTolerance { linear, angular, parametric, linear_max }` | The one tolerance policy: `for_extent`, `coincident(d, ta, tb)` (`d <= ta + tb` and `ta + tb <= linear_max`), and the hash quantization (below). |
| `Frame` | Right-handed orthonormal placement used by all analytic kinds. |
| `GeomError` | `OutOfDomain`, `Singular`, `NoConvergence`, `Unsupported`, `Invalid` (`#[non_exhaustive]`). Every constructor returns `Result<_, GeomError>`. |

## Conventions (binding)

- Surface normal is `du x dv`, normalized. For all analytic kinds this is outward (sphere, cylinder, cone, torus) or `+z` (plane) and equals the implicit gradient direction (tested). Face orientation is topology's `Sense` on its carrier.
- Evaluation is infallible and continues analytically outside the domain; periodic kinds wrap. Failure-capable operations (`normal`, `project`, `length`, `ray_hits`) return `Result`.
- `project(p, hint)` returns the closest point within the domain. Non-unique minima are deterministic (circle/cylinder/cone/torus pick `u = 0`; a sphere centre is `Singular`). Ellipse projection is `Unsupported`.
- Ray hook: `ray_hits(&Ray, Interval, &mut Vec<RayHit>)` appends hits in ascending `t` to a caller-owned buffer. The default goes through `implicit()` and fills `uv` by projection; the cone keeps only its parametric nappe; NURBS surfaces return `Unsupported` (the tracker facets them).
- `ParametricCurve::length(range, tol)` refines composite Gauss-Legendre until two successive estimates agree to `tol` (`NoConvergence` after 4096 panels); line and circle are exact.
- Tangent roots (**D16**): `real_roots` returns each root with a double-root flag (from near-zero critical values of the derivative chain), so the quartic torus reports a grazing contact once as `Tangent`. Tested against the analytic graze at the tube top (`t = 7, 13`) and the four simple crossings just below it.

## Hash quantization rule (Q2, binding, cross-platform)

`ModelTolerance::quantum() = linear * 2^-10` quantizes lengths, `param_quantum() = parametric * 2^-10` quantizes parameter-valued floats; `quantize(x) = round_half_away_from_zero(x / q)` as `i64` (NaN -> `i64::MIN`, infinities saturate, `-0 == +0`). Topology's `canonical_hash` and the mesh `geometry_hash` hash these integers, never raw float bits. Angles and direction components (frame axes, cone half-angle) use `angular * 2^-10`. Whole shape definitions are visited through `CurveDef::for_each_value` / `SurfaceDef::for_each_value` (`ShapeValue` tags the class; `ModelTolerance::quantize_shape` applies the rule). Scale-free shape values (knots, weights, quadric coefficients, line direction) use `quantize_scale_free`: relative, 40-bit mantissa packed with exponent and sign, zero below `param_quantum()`, never saturating, and stable across a power-of-two boundary (the rounding carry goes into the exponent). Content hashes go through `CurveDef::quantized_values` / `SurfaceDef::quantized_values` (W0d follow-up, done in W0f), which apply `quantize_scale_free_in(x, scale)` with `scale` = the largest scale-free magnitude of the same definition: a value below `max(param_quantum(), scale * 2^-40)` (one relative step of the largest value) hashes as zero, so cancellation noise after a large translation (a cross term of a translated quadric) does not change the hash. The plain `quantize_scale_free` keeps the absolute cutoff. A value within a small fraction of a step of another hashes equal unless the pair straddles a rounding boundary; platform noise (a few ulp) is far below one step, so the only residual risk is a value within noise of a half-step boundary.

## Transforms (W0f)

`CurveDef::transformed(&Transform) -> Result<CurveDef>`, `SurfaceDef::transformed(&Transform) -> Result<(SurfaceDef, Sense)>`, `ImplicitDef::transformed`, `ImplicitTorus::transformed` (plus `Line`, `NurbsCurve`, `NurbsSurface::transformed`; `Quadric::transformed` already existed). A similarity (rotation, translation, uniform scale, optional reflection; detected to `1e-12` relative) keeps every kind: frames mapped, lengths scaled. A non-uniform map keeps lines, planes and NURBS (control points mapped: affine invariance), turns circles and ellipses into ellipses (principal axes of the image), (elliptic) cylinders into `EllipticCylinder` (section perpendicular to the image axis, in principal axes), (elliptic) cones into `EllipticCone` (the principal axes of the image quadric, same nappe, `v = 0` section through the image of the old section centre), and spheres into their implicit quadric; a torus is `Unsupported`; a singular map is `Singular`. Curves keep their parameter under a similarity (a line's is scaled by `|M dir|`); surfaces of revolution keep their axis (`z' = R z`, so `u` runs backwards under a reflection); a plane keeps `x`, `y`. `Sense` (new in this crate; topology converts with `From`) says whether the new natural normal is opposite to `M^-T n`, the image of the old normal: `Reversed` only for planes and NURBS surfaces under an improper map. Tested pointwise (image points on the image, normals against `M^-T n`) and against analytic values (stretched circle -> ellipse `a = 2, b = 1`, stretched unit sphere -> ellipsoid, scaled sphere and torus radii).

## Schemas (W0f)

The `schemars` feature (implies `serde`, enables `sm-core/schemars`) derives `JsonSchema` on every serde value type (`CurveDef`, `SurfaceDef`, `ImplicitDef`, `PCurve`, `CsgCell`, the analytic kinds, NURBS, domains, `ModelTolerance`, `Sense`, ...); `SmallVec` fields are described as `Vec<T>`. `MemoryStore` (persisted through `StoreData`) has no schema. `ImplicitDef` is `#[non_exhaustive]` (W0f), like `CurveDef` / `SurfaceDef`.

## NURBS (Q4)

`NurbsCurve` / `NurbsSurface` / `NurbsCurve2` are validated data **and** evaluable here: a small allocation-free de Boor evaluator (Piegl and Tiller A2.1/A2.3, degree <= 15, first and second derivatives, rational via the quotient rule) implements `ParametricCurve` / `ParametricSurface`, plus a numeric closest-point projector (dense sampling then safeguarded Newton). Non-periodic only; a boundary row of coincident control points is reported as a pole. `sm-geom` may provide faster implementations behind the same traits.

## Requirement map

| Requirement | Where |
|---|---|
| Curve: eval, derivatives, range, periodicity, projection, length | `curve.rs`, `analytic.rs`, `nurbs.rs` |
| Surface: eval, 1st/2nd derivatives, normal, seams and poles, inverse projection | `surface.rs`, `domain.rs` |
| Analytic kinds (ring torus only, single-nappe parametric cone plus double-nappe implicit cone: Q10) | `analytic.rs` |
| NURBS as data and evaluator | `nurbs.rs` |
| CSG: general quadric (incl. improper transforms via `Quadric::transformed`), torus quartic, half-spaces, cells | `implicit.rs` |
| Closed serializable defs, shared surfaces by handle | `store.rs` |
| Pcurves (hashed by content in topology) | `pcurve.rs` |
| Ray hook with `Crossing` | `ray.rs`, `implicit.rs` |
| Tolerance policy and hash rule | `tol.rs` |

## Tests

Analytic, not smoke: finite-difference checks of every first/second derivative; closed-form values; projection minimality and orthogonality against dense sampling; ray hits against hand values (sphere `t = 8, 12`; torus `t = 6, 8, 12, 14`; grazing contacts as `Tangent`); implicit/parametric agreement; exact rational quarter circle (radius 1 to 1e-14, length pi/2); polynomial roots with multiplicity; quadric transforms (rotation, translation, mirror) against pointwise evaluation; a CSG cell with a general quadric stored as `SurfaceDef::Implicit` (containment, ray entry and exit with outward normals); store handles and validation; pcurve variants; the quantization rule; and a determinism test (repeated runs and four threads give bit-identical results and ids).

## Deviations from `geomtopo-recommendation.md` (honest list)

- `Domain2` keeps cellcomplex's `singularities: SmallVec<[Singularity; 2]>` (an iso-line list) instead of the four `u_lo/u_hi/v_lo/v_hi` enum fields of the §5.1 sketch; the information is the same and it is what the analytic kinds already produce.
- `CsgCell::ray_hits` takes an extra `range: Interval` (a tracker always has one). The optional "union of cells" region is not provided.
- `ParametricCurve` keeps `d1`, `tangent`, `is_closed` helpers beyond the §5.1 sketch.
- Names follow the sketch (`ImplicitSurface`, `ImplicitDef`, `ImplicitTorus`, `GeomStore`, `GeomStoreMut`, `ModelTolerance`); the cellcomplex names (`SignedSurface`, `TorusImplicit`, `ImplicitForm`, `GeometryStore`, `Tolerance`) are gone.

## Open issues

- Ellipse projection is `Unsupported` (needs a quartic); the torus is ring-only (spindle and horn tori later, Q10); NURBS surfaces have no ray hook.
- Root finding accepts a tangency when the critical value is within `1e-12` relative; two roots closer than `1e-9` relative merge.
- No `Hash`/`Eq` on f64-bearing types by design; content hashing happens in topology.

## W0f gaps 2: elliptic quadrics

`SurfaceDef::EllipticCylinder(EllipticCylinder { frame, a, b })` (`o + a cos u x + b sin u y + v z`) and `SurfaceDef::EllipticCone(EllipticCone { frame, a, b, half_angle })` (`o + k(v)(a cos u x + b sin u y) + v cos h z`, `k = 1 + v sin h / a`: the `v = 0` section is the ellipse `(a, b)`, `h` is the half-angle in the `x`-`z` plane, apex at `v = -a / sin h`; one nappe, like `Cone`). Both implement `ParametricSurface` (outward `du x dv` in a right-handed frame, exact normal at the cone apex since the normal is constant along a generator) and `implicit()` (quadric, negative inside; the cone's is the double cone and `ray_hits` keeps the `+z` nappe). `EllipticCylinder::project` is exact (closest point on the section ellipse by bisection, Eberly); `EllipticCone::project` is `Unsupported` (no closed form; `sm-geom` supplies one), like `Ellipse`. They are the elliptic frustum primitive of PARITY B and the images of non-uniformly scaled cylinders and cones in `SurfaceDef::transformed`, which no longer returns a parameterless `Implicit` for those (spheres still do; tori stay `Unsupported`). Hash tags 8 and 9 in `for_each_value`. This changes `transformed()` output, so it lands before W1 pins geometry hashes. Tested: points on the quadric, outward normals, finite-difference derivatives, `a = b` against `Cylinder` / `Cone`, projection minimality against dense sampling, ray hits, the Jacobi eigen-solver, and analytic images (stretched cylinder `a = 2, b = 1`, sheared cylinder axis and section, stretched 45° cone half-angles `atan 2` and 45°).

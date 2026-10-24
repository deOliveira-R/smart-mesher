# sm-scheme-curve: design

Five curve schemes for the `sm-scheme-api` contract: `equal`, `bias`, `dualbias`, `stretch`, `curvature`.

## Positioning

Similar to **Cubit's curve schemes** (equal, bias, dualbias, stretch, curvature) and to **Gmsh's transfinite progressions** (`Progression`, `Bump`), it places edge nodes by a one-dimensional spacing law along arc length. It differs in where the law lives: the schemes work in **normalized arc length** and convert to **curve-parameter fractions** with an adaptive Gauss-Legendre arc-length map and a monotone safeguarded-Newton inverse, so every law is exact on lines, circles, ellipses and NURBS alike, and node keys, positions and parameters stay with the driver (`GenCtx::curve_patch`). That gives analytic closure (consecutive-edge ratio, first size, mirror symmetry) to 1e-12 on any curve, bit-identical output for any thread count, and typed errors (never a silent relaxation) when sizes cannot be honoured.

## Layers

| Module | Role |
|---|---|
| `spacing` | Pure math in arc-length fractions: geometric series, ratio/first-size solving, count derivation, dual series, stretch profile. No geometry. |
| `arc` (private) | `ArcMap`: arc length as a function of the edge fraction. |
| `engine` | `Spec` + `node_params(curve, carrier, n, spec)`: arc fractions to parameter fractions; curvature density. |
| `params` | Typed serde structs carried in `SchemeChoice::params`. |
| `schemes` | `CurveScheme` (one `Scheme` implementation parametrized by `CurveKind`), `register`. |

## Arc-length map

`g(tau) = |C'(t(tau))| * |hi - lo|` with `t(tau)` from `geom::edge_param`, so edge sense is handled once. Eight equal panels are integrated with 5-point Gauss-Legendre and refined locally: a panel is bisected while its estimate and the sum of its two halves differ by more than 1e-14 times the initial length (depth-first, fixed order), so kinks and jumps at repeated knots (any non-dyadic position) cost a few dozen extra panels, not a global doubling. Panels narrower than 1e-13 are accepted; more than 2^16 panels is a `Geometry` error. Cumulative lengths and the (non-uniform) breakpoints give the inverse: binary search for the panel, then bracketed Newton on `integral(lo, tau) = target` inside it (derivative `g`, bisection fallback). If the speed spread over all quadrature nodes is below 1e-13 relative the parametrization is arc-length proportional (line, circle) and the map is the identity, which makes equal spacing on a line exact to rounding. Node inversions run under rayon above a fixed size threshold; each element is independent and the panel refinement is sequential, so output is bit-identical for any thread count.

## Spacing laws (binding decisions from `docs/research/cubit-meshing-semantics.md`)

- **Bias** (`e_{k+1} = r e_k`). Nodes `s_k = (r^k - 1)/(r^n - 1)`, evaluated with `exp_m1` and scaled exponentials (no cancellation near `r = 1`, no overflow). First size to ratio: monotone root of `ln(s_1(lambda)) = ln(a/L)` in `lambda = ln r` by bracketed Newton (finite-difference slope, bisection when the step leaves the bracket). Count from `(L, a, r)`: real count rounded half-to-even, minimum 1. Count from fine and coarse sizes: the series sum grows with the count, so a doubling search then bisection finds the integer minimizing `|sum - L|` (the Ours text iterates `n -> r -> n`, which is a fixed point at every `n`).
- **Dualbias.** Even `n = 2m`: `m` edges per half. Odd `n = 2m+1`: middle edge `a r^m`. Nodes are built from the half-series cumulative sum and mirrored (`1 - s`), so symmetry is exact. `first_size` solved the same way as bias.
- **Stretch.** First size only: bias with that first size. First and last: end edges pinned, interior linear between them and scaled to close the length (n >= 3). The ratio of consecutive edges is checked against `max_ratio` (default 2); a spacing that needs more is a typed `RatioLimit` error. The count is derived softly as `round(2L/(first+last))`.
- **Curvature.** Density `rho = (1 + |c| L k)^sign(c)`; `max(64 n, 512)` samples at equal **arc-length** fractions (positions from the arc map), trapezoid cumulative integral of `rho` over arc length, linear inversion to arc fractions, then the arc map. Constant curvature gives equal spacing on any parametrization (to rounding). Zero speed or non-finite curvature at a sample is a `Geometry` error. `|c| L kmax > 1e6` is an error. Closed (periodic) edges are refused for stretch and curvature.

## Interval counts

Schemes that imply a count (bias with `{first_size, last_size}` or `{ratio, first_size}`, stretch with `last_size`) emit a **soft** `Fixed` constraint from `interval_constraints`. A hard count on the edge wins; otherwise the solver sees it as a preference. All other modes take the plan's count.

## Failures

`applicability` rejects bad parameters with a reason (`NotApplicable`). `generate` maps `SpacingError` to `FailureKind`: parameter problems `NotApplicable`, unreachable sizes or ratio limits `Constraint`, geometry problems `Geometry`; none is `Internal`, so the driver may fall back. Nothing panics on user input.

## Determinism

All functions are pure. Parallel loops are per-element maps collected in index order followed by sequential sums. Tests compare bit patterns at 1 and N threads and across concurrent calls (`sm_testkit::determinism::check_f64_bits`).
